// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdint.h>
#include <threads.h>

#include <bits/limits.h>
#include <ddk/debug.h>
#include <hw/reg.h>

#include <zircon/assert.h>
#include <zircon/types.h>

#include "a113-bus.h"

// Phsical Base Address for Pinmux/GPIO Control
#define GPIO_BASE_PHYS 0xff634400
#define GPIO_BASE_PAGE ((~0xFFF) & GPIO_BASE_PHYS)

// Offsets for Various Control Registers

// Pinmux Control Register Offsets
#define PERIPHS_PIN_MUX_0 0x20
#define PERIPHS_PIN_MUX_1 0x21
#define PERIPHS_PIN_MUX_2 0x22
#define PERIPHS_PIN_MUX_3 0x23
#define PERIPHS_PIN_MUX_4 0x24
#define PERIPHS_PIN_MUX_5 0x25
#define PERIPHS_PIN_MUX_6 0x26
// NOTE: PERIPHS_PIN_MUX_7 is not specified by the manual
#define PERIPHS_PIN_MUX_8 0x28
#define PERIPHS_PIN_MUX_9 0x29
// NOTE: PERIPHS_PIN_MUX_A is not specified by the manual
#define PERIPHS_PIN_MUX_B 0x2b
#define PERIPHS_PIN_MUX_C 0x2c
#define PERIPHS_PIN_MUX_D 0x2d

#define AO_RTI_PIN_MUX_REG0 0x05
#define AO_RTI_PIN_MUX_REG1 0x06

// Offsets from GPIO_BASE_PHYS are defined as follows.
#define REGADDR(r) ((GPIO_BASE_PHYS - GPIO_BASE_PAGE) + (4 * r))

#define A113_PINMUX_ALT_FN_MAX 15

typedef struct pinmux_block {
    uint32_t start_pin;
    uint32_t pin_count;
    uint32_t ctrl_block_offset;
    mtx_t lock;
} pinmux_block_t;

static pinmux_block_t pinmux_blocks[] = {
    // GPIO X Block
    {
        .start_pin = (A113_GPIOX_START + 0),
        .pin_count = 8,
        .ctrl_block_offset = PERIPHS_PIN_MUX_4,
        .lock = MTX_INIT,
    },
    {
        .start_pin = (A113_GPIOX_START + 8),
        .pin_count = 8,
        .ctrl_block_offset = PERIPHS_PIN_MUX_5,
        .lock = MTX_INIT,
    },
    {
        .start_pin = (A113_GPIOX_START + 16),
        .pin_count = 7,
        .ctrl_block_offset = PERIPHS_PIN_MUX_6,
        .lock = MTX_INIT,
    },

    // GPIO A Block
    {
        .start_pin = (A113_GPIOA_START + 0),
        .pin_count = 8,
        .ctrl_block_offset = PERIPHS_PIN_MUX_B,
        .lock = MTX_INIT,
    },
    {
        .start_pin = (A113_GPIOA_START + 8),
        .pin_count = 8,
        .ctrl_block_offset = PERIPHS_PIN_MUX_C,
        .lock = MTX_INIT,
    },
    {
        .start_pin = (A113_GPIOA_START + 16),
        .pin_count = 5,
        .ctrl_block_offset = PERIPHS_PIN_MUX_D,
        .lock = MTX_INIT,
    },

    // GPIO Boot Block
    {
        .start_pin = (A113_GPIOB_START + 0),
        .pin_count = 8,
        .ctrl_block_offset = PERIPHS_PIN_MUX_0,
        .lock = MTX_INIT,
    },
    {
        .start_pin = (A113_GPIOB_START + 8),
        .pin_count = 7,
        .ctrl_block_offset = PERIPHS_PIN_MUX_1,
        .lock = MTX_INIT,
    },

    // GPIO Y Block
    {
        .start_pin = (A113_GPIOY_START + 0),
        .pin_count = 8,
        .ctrl_block_offset = PERIPHS_PIN_MUX_8,
        .lock = MTX_INIT,
    },
    {
        .start_pin = (A113_GPIOY_START + 8),
        .pin_count = 8,
        .ctrl_block_offset = PERIPHS_PIN_MUX_9,
        .lock = MTX_INIT,
    },

    // GPIO Z Block
    {
        .start_pin = (A113_GPIOZ_START + 0),
        .pin_count = 8,
        .ctrl_block_offset = PERIPHS_PIN_MUX_2,
        .lock = MTX_INIT,
    },
    {
        .start_pin = (A113_GPIOZ_START + 8),
        .pin_count = 3,
        .ctrl_block_offset = PERIPHS_PIN_MUX_3,
        .lock = MTX_INIT,
    },

    // GPIO AO Block
    {
        .start_pin = (A113_GPIOAO_START + 0),
        .pin_count = 8,
        .ctrl_block_offset = AO_RTI_PIN_MUX_REG0,
        .lock = MTX_INIT,
    },
    {
        .start_pin = (A113_GPIOAO_START + 8),
        .pin_count = 6,
        .ctrl_block_offset = AO_RTI_PIN_MUX_REG1,
        .lock = MTX_INIT,
    },
};

static zx_status_t a113_pin_to_block(const uint32_t pinid, pinmux_block_t** result) {
    ZX_DEBUG_ASSERT(result);

    for (size_t i = 0; i < countof(pinmux_blocks); i++) {
        const uint32_t end_pin = pinmux_blocks[i].start_pin + pinmux_blocks[i].pin_count;
        if (pinid >= pinmux_blocks[i].start_pin && pinid < end_pin) {
            *result = &(pinmux_blocks[i]);
            return ZX_OK;
        }
    }

    return ZX_ERR_NOT_FOUND;
}

// Configure a pin for an alternate function specified by fn
zx_status_t a113_config_pinmux(void* ctx, const uint32_t pin, const uint32_t fn) {
    ZX_DEBUG_ASSERT(ctx);

    if (fn > A113_PINMUX_ALT_FN_MAX) {
        dprintf(ERROR, "a113_config_pinmux: pin mux alt config out of range %u\n", fn);
        return ZX_ERR_OUT_OF_RANGE;
    }

    zx_status_t status;
    a113_bus_t* bus = (a113_bus_t*)ctx;

    pinmux_block_t* block;
    status = a113_pin_to_block(pin, &block);
    if (status != ZX_OK) {
        dprintf(ERROR, "a113_config_pinmux: pin not found %u\n", pin);
        return status;
    }

    const size_t register_offset = REGADDR(block->ctrl_block_offset);
    volatile uint32_t* reg = (uint32_t*)(io_buffer_virt(&bus->periphs_reg) + register_offset);

    const uint32_t pin_offset = pin - block->start_pin;
    const uint32_t mux_mask = ~(0xf << (pin_offset*4));
    const uint32_t fn_val = fn << (pin_offset*4);

    mtx_lock(&block->lock);

    uint8_t regval = readl(reg);
    regval &= mux_mask;     // Remove the previous value for the mux
    regval |= fn_val;       // Assign the new value to the mux
    writel(regval, reg);

    mtx_unlock(&block->lock);

    return ZX_OK;
}

zx_status_t a113_init_pinmux(a113_bus_t* bus) {
    ZX_DEBUG_ASSERT(bus);

    zx_handle_t resource = get_root_resource();
    zx_status_t status;

    status = io_buffer_init_physical(&bus->periphs_reg, GPIO_BASE_PAGE, PAGE_SIZE,
                                     resource, ZX_CACHE_POLICY_UNCACHED_DEVICE);

    if (status != ZX_OK) {
        dprintf(ERROR, "a113_init_pinmux: io_buffer_init_physical failed %d\n", status);
        return status;
    }

    return ZX_OK;
}