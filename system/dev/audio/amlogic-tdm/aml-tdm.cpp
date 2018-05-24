// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <audio-proto-utils/format-utils.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <fbl/limits.h>
#include <string.h>
#include <zircon/device/audio.h>
#include <lib/zx/vmar.h>
#include <ddk/protocol/platform-device.h>

#include "aml-tdm.h"

fbl::unique_ptr<AmlTdmDevice> AmlTdmDevice::Create(platform_device_protocol_t* pdev,
                                  uint32_t mmio_idx) {

    auto tdm_dev = fbl::unique_ptr<AmlTdmDevice>(new AmlTdmDevice());

    size_t mmio_size;
    void *regs;
    zx_status_t res = pdev_map_mmio(pdev, mmio_idx, ZX_CACHE_POLICY_UNCACHED_DEVICE,
                        &regs, &mmio_size,
                        tdm_dev->regs_vmo_.reset_and_get_address());

    if (res != ZX_OK) {
        zxlogf(ERROR, "%s: failed to map mmio - %d\n", __func__, res);
        return nullptr;
    }

    tdm_dev->regs_ = static_cast<aml_tdm_regs_t*>(regs);

    return tdm_dev;
}



AmlTdmDevice::~AmlTdmDevice() {

}