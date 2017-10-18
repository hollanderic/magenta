// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <unistd.h>

#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/protocol/platform-defs.h>

#include <zircon/process.h>
#include <zircon/syscalls.h>
#include <zircon/assert.h>

#include "a113-bus.h"
#include "aml-i2c.h"


static zx_status_t a113_bus_get_protocol(void* ctx, uint32_t proto_id, void* out) {
    return ZX_ERR_NOT_SUPPORTED;
}

static pbus_interface_ops_t a113_bus_bus_ops = {
    .get_protocol = a113_bus_get_protocol,
};

static void a113_bus_release(void* ctx) {
    a113_bus_t* bus = ctx;
    free(bus);
}

static zx_protocol_device_t a113_bus_device_protocol = {
    .version = DEVICE_OPS_VERSION,
    .release = a113_bus_release,
};

static zx_status_t a113_bus_bind(void* ctx, zx_device_t* parent, void** cookie) {
    a113_bus_t* bus = calloc(1, sizeof(a113_bus_t));
    if (!bus) {
        return ZX_ERR_NO_MEMORY;
    }

    if (device_get_protocol(parent, ZX_PROTOCOL_PLATFORM_BUS, &bus->pbus) != ZX_OK) {
        free(bus);
        return ZX_ERR_NOT_SUPPORTED;
    }

    device_add_args_t args = {
        .version = DEVICE_ADD_ARGS_VERSION,
        .name = "a113-bus",
        .ops = &a113_bus_device_protocol,
        .flags = DEVICE_ADD_NON_BINDABLE,
    };

    zx_status_t status = device_add(parent, &args, NULL);
    if (status != ZX_OK) {
        goto fail;
    }

    pbus_interface_t intf;
    intf.ops = &a113_bus_bus_ops;
    intf.ctx = bus;
    pbus_set_interface(&bus->pbus, &intf);

    // Initialize Pin mux subsystem.
    a113_init_pinmux(bus);
    a113_config_pinmux(bus, A113_GPIOAO(0), 0);
    a113_config_pinmux(bus, A113_GPIOAO(1), 0);

//GPIOZ 8,9 ->config to I2C_B
    printf("A113: setting pinmux for i2c-b\n");
    a113_config_pinmux(bus, A113_GPIOZ(8), 1);
    a113_config_pinmux(bus, A113_GPIOZ(9), 1);


    aml_i2c_dev_t *i2cb_dev;

    status = aml_i2c_init(&i2cb_dev, bus, AML_I2C_B);
    if (status != ZX_OK) {
        printf("Could not initialize i2c device!\n");
    }
    //aml_i2c_set_slave_addr(i2cb_dev,0x18);
    aml_i2c_dumpstate(i2cb_dev);

    //aml_i2c_write(i2cb_dev, NULL, 2);
/*
    printf("A113: register block base address (virt) = %p\n",reg);
    printf("A113: register block base address (phys) = %lx\nsize=%lu\n",bus->i2c_b_regs.phys,
                                                                        bus->i2c_b_regs.size);
    reg[0] = reg[0] |  (1 << 22);
    reg[0] = reg[0] & ~(3 << 23);

    printf("A113: regs[0] - 0x%08x\n",reg[0]);
    printf("A113: regs[1] - 0x%08x\n",reg[1]);
    printf("A113: regs[2] - 0x%08x\n",reg[2]);
    printf("A113: regs[3] - 0x%08x\n",reg[3]);

*/

    aml_i2c_test(i2cb_dev);




    return ZX_OK;

fail:
    printf("a113_bus_bind failed %d\n", status);
    a113_bus_release(bus);
    return status;
}

static zx_driver_ops_t a113_bus_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = a113_bus_bind,
};

ZIRCON_DRIVER_BEGIN(a113_bus, a113_bus_driver_ops, "zircon", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PROTOCOL, ZX_PROTOCOL_PLATFORM_BUS),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_AMLOGIC),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_PID, PDEV_PID_AMLOGIC_A113),
ZIRCON_DRIVER_END(a113_bus)
