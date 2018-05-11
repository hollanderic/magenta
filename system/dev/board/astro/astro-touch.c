// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/protocol/platform-bus.h>
#include <ddk/protocol/platform-defs.h>

#include <soc/aml-s905d2/s905d2-gpio.h>
#include <soc/aml-s905d2/s905d2-hw.h>

#include <limits.h>

#include "astro.h"

static const pbus_gpio_t touch_gpios[] = {
    {
        // touch interrupt
        .gpio = S905D2_GPIOZ(4),
    },
    {
        // touch reset
        .gpio = S905D2_GPIOZ(9),
    },
};

static const pbus_i2c_channel_t touch_i2c[] = {
    {
        .bus_id = 1,
        .address = 0x38,
    },
};

static pbus_dev_t touch_dev = {
    .name = "astro-touch",
    .vid = PDEV_VID_GOOGLE,
    .pid = PDEV_PID_ASTRO,
    .did = PDEV_DID_ASTRO_FOCALTOUCH,
    .i2c_channels = touch_i2c,
    .i2c_channel_count = countof(touch_i2c),
    .gpios = touch_gpios,
    .gpio_count = countof(touch_gpios),
};

zx_status_t astro_touch_init(aml_bus_t* bus) {
    zx_status_t status = pbus_device_add(&bus->pbus, &touch_dev, 0);
    if (status != ZX_OK) {
        zxlogf(ERROR, "astro_touch_init: pbus_device_add failed: %d\n", status);
        return status;
    }

    return ZX_OK;
}
