// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include <ddk/debug.h>
//#include <ddktl/mmio.h>

#include <ddktl/pdev.h>
#include <fbl/alloc_checker.h>
#include <zircon/assert.h>
#include <lib/zx/interrupt.h>

namespace ddk {

void Pdev::ShowInfo() {
    zxlogf(INFO, "Flags               = %08x\n",pdev_info_.flags);
    zxlogf(INFO, "VID:PID:DID         = %04x:%04x:%04x\n",pdev_info_.vid,
                                                  pdev_info_.pid,
                                                  pdev_info_.did);
    zxlogf(INFO, "mmio count          = %d\n", pdev_info_.mmio_count);
    zxlogf(INFO, "irq count           = %d\n", pdev_info_.irq_count);
    zxlogf(INFO, "gpio count          = %d\n", pdev_info_.gpio_count);
    zxlogf(INFO, "i2c channel count   = %d\n", pdev_info_.i2c_channel_count);
    zxlogf(INFO, "clk count           = %d\n", pdev_info_.clk_count);
    zxlogf(INFO, "bti count           = %d\n", pdev_info_.bti_count);
    zxlogf(INFO, "boot metadata count = %d\n", pdev_info_.boot_metadata_count);
}

MmioBlock Pdev::GetMmio(uint32_t index) {

    void* ptr;
    size_t len;
    zx::vmo vmo;

    zx_status_t res = pdev_map_mmio(&pdev_, index,
        ZX_CACHE_POLICY_UNCACHED_DEVICE, &ptr,
        &len, vmo.reset_and_get_address());
    if (res != ZX_OK) {
        return MmioBlock(MmioBlock(nullptr, 0, 0));
    }
    return MmioBlock(ptr, len, vmo.release());
}


fbl::RefPtr<Pdev> Pdev::Create(zx_device_t* parent) {

    fbl::AllocChecker ac;

    auto pdev = fbl::AdoptRef(new (&ac) Pdev(parent));
    if (!ac.check()) {
        return nullptr;
    }

    zx_status_t status = device_get_protocol(parent,
                                             ZX_PROTOCOL_PLATFORM_DEV,
                                             &pdev->pdev_);
    if (status != ZX_OK) {
        return nullptr;
    }

    status = pdev_get_device_info(&pdev->pdev_, &pdev->pdev_info_);
    if (status != ZX_OK) {
        return nullptr;
    }

    pdev->ShowInfo();

    return fbl::move(pdev);
}






} //namespace ddk