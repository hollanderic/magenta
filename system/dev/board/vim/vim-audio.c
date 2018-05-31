// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/protocol/platform-defs.h>
#include <hw/reg.h>
#include <soc/aml-s912/s912-hw.h>
#include <soc/common/aml-tdm.h>
#include <zircon/assert.h>
#include <limits.h>

#include "vim.h"

static const pbus_mmio_t tdm_audio_mmios[] = {
    {
        .base = A113_TDM_PHYS_BASE,
        .length = 4096
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

static const pbus_dev_t vim_tdm_audio_dev = {
    .name = "vim-tdm-audio",
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

zx_status_t vim_audio_init(gauss_bus_t* bus) {

    ZX_DEBUG_ASSERT(bus);
    zx_status_t status;

    zxlogf(INFO,"Adding the tdm device\n");
    if ((status = pbus_device_add(&bus->pbus, &gauss_tdm_audio_dev, 0)) != ZX_OK) {
        zxlogf(ERROR, "a113_audio_init could not add gauss_tdm_audio_dev: %d\n", status);
    }

    return ZX_OK;
}
