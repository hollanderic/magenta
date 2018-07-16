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

fbl::unique_ptr<AmlTdmDevice> AmlTdmDevice::Create(ddk::MmioBlock&& mmio) {

    auto tdm_dev = fbl::unique_ptr<AmlTdmDevice>(new AmlTdmDevice());
    zxlogf(INFO,"before\n");
    mmio.Info();
    tdm_dev->mmio_ = mmio.release();
    zxlogf(INFO,"after\n");
    mmio.Info();
    tdm_dev->mmio_.Info();

#if 0
    tdm_dev->uregs_ = ddk::MmioBlock<uint32_t>::Create(pdev, mmio_idx);
    if (!tdm_dev->uregs_) {
        zxlogf(ERROR,"AmlTdm: Failed to map mmio\n");
        return nullptr;
    }
#endif
    tdm_dev->InitRegs();

    return tdm_dev;
}


void AmlTdmDevice::InitRegs() {

    //uregs_->SetBits(0x00002000, AML_TDM_CLK_GATE_EN);


}

AmlTdmDevice::~AmlTdmDevice() {

}