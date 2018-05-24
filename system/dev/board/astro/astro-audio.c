// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/metadata.h>

#include <ddk/protocol/platform-bus.h>
#include <ddk/protocol/platform-defs.h>

#include <soc/aml-common/aml-tdm.h>
#include <soc/aml-s905d2/s905d2-gpio.h>
#include <soc/aml-s905d2/s905d2-hw.h>

#include <limits.h>

#include "astro.h"


static char teststr[] = "Eric Holland\0";

static pbus_boot_metadata_t meta[] = {
    {
        .type = AML_TDM_METADATA,
        .data = teststr,
        .extra = 0,
        .len = sizeof(teststr)
    },
};

#if 0
static const pbus_i2c_channel_t tdm_i2cs[] = {
    {
        .bus_id = AML_I2C_B,
        .address = 0x4C
    },
    {
        .bus_id = AML_I2C_B,
        .address = 0x4D
    },
    {
        .bus_id = AML_I2C_B,
        .address = 0x4E
    },
};

static const pbus_irq_t tdm_irqs[] = {
    {
        .irq = (90 + 32),
        .mode = ZX_INTERRUPT_MODE_EDGE_HIGH,
    },
};

static const pbus_bti_t tdm_btis[] = {
    {
        .iommu_index = 0,
        .bti_id = BTI_AUDIO_OUT,
    },
};

static const pbus_dev_t gauss_tdm_audio_dev = {
    .name = "gauss-tdm-audio",
    .vid = PDEV_VID_GOOGLE,
    .pid = PDEV_PID_GAUSS,
    .did = PDEV_DID_GAUSS_AUDIO_OUT,
    .irqs = tdm_irqs,
    .irq_count = countof(tdm_irqs),
    .mmios = tdm_audio_mmios,
    .mmio_count = countof(tdm_audio_mmios),
    .i2c_channels = tdm_i2cs,
    .i2c_channel_count = countof(tdm_i2cs),
    .btis = tdm_btis,
    .bti_count = countof(tdm_btis),
};


#endif

static pbus_dev_t aml_tdm_dev = {
    .name = teststr,
    .vid = PDEV_VID_AMLOGIC,
    .pid = PDEV_PID_AMLOGIC_S905D2,
    .did = PDEV_DID_AMLOGIC_TDM,
    .boot_metadata = meta,
    .boot_metadata_count = countof(meta)
};

zx_status_t astro_tdm_init(aml_bus_t* bus) {

    zx_status_t status = pbus_device_add(&bus->pbus, &aml_tdm_dev, 0);
    if (status != ZX_OK) {
        zxlogf(ERROR, "astro_touch_init(ft3x27): pbus_device_add failed: %d\n", status);
        return status;
    }

    return ZX_OK;
}
