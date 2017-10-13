// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/protocol/platform-bus.h>
#include <ddk/io-buffer.h>

#define A113_GPIOX_START 0
#define A113_GPIOA_START 23
#define A113_GPIOB_START (A113_GPIOA_START + 21)
#define A113_GPIOY_START (A113_GPIOB_START + 15)
#define A113_GPIOZ_START (A113_GPIOY_START + 16)
#define A113_GPIOAO_START (A113_GPIOZ_START + 11)

#define A113_GPIOX(n) (A113_GPIOX_START + n)
#define A113_GPIOA(n) (A113_GPIOA_START + n)
#define A113_GPIOB(n) (A113_GPIOB_START + n)
#define A113_GPIOY(n) (A113_GPIOY_START + n)
#define A113_GPIOZ(n) (A113_GPIOZ_START + n)
#define A113_GPIOAO(n) (A113_GPIOAO_START + n)

typedef struct {
    platform_bus_protocol_t pbus;
    io_buffer_t periphs_reg;
    // more coming soon
} a113_bus_t;

zx_status_t a113_config_pinmux(void* ctx, const uint32_t pin, const uint32_t fn);
zx_status_t a113_init_pinmux(a113_bus_t* bus);
