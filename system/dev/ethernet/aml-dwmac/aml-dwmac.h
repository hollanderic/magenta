// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/device.h>
#include <ddk/protocol/test.h>
#include <ddktl/device.h>
#include <ddktl/protocol/ethernet.h>
#include <ddktl/protocol/test.h>
#include <zircon/compiler.h>
#include <zircon/types.h>
#include <zx/socket.h>
#include <zx/vmo.h>
#include <fbl/mutex.h>
#include <fbl/unique_ptr.h>
#include <threads.h>

namespace eth {

class AmlDWMacDevice : public ddk::Device<AmlDWMacDevice, ddk::Unbindable>,
                       public ddk::EthmacProtocol<AmlDWMacDevice> {
  public:
    AmlDWMacDevice(zx_device_t* device);

    static zx_status_t Create(zx_device_t* device);

    void DdkRelease();
    void DdkUnbind();

    zx_status_t EthmacQuery(uint32_t options, ethmac_info_t* info);
    void EthmacStop();
    zx_status_t EthmacStart(fbl::unique_ptr<ddk::EthmacIfcProxy> proxy);
    zx_status_t EthmacQueueTx(uint32_t options, ethmac_netbuf_t* netbuf);
    zx_status_t EthmacSetParam(uint32_t param, int32_t value, void* data);

    //int Thread();

  private:
    //zx_status_t UpdateLinkStatus(zx_signals_t observed);
    //zx_status_t Recv(uint8_t* buffer, uint32_t capacity);

    // designware mac options
    uint32_t options_ = 0;

    // ethermac fields
    uint32_t features_ = 0;
    uint32_t mtu_ = 0;
    uint8_t mac_[6] = {};

    platform_device_protocol_t pdev_;

    zx_vaddr_t periph_regs_;
    size_t periph_regs_size_;
    zx::vmo periph_regs_vmo_;

    zx_vaddr_t dwmac_regs_;
    size_t dwmac_regs_size_;
    zx::vmo dwmac_regs_vmo_;

    fbl::Mutex lock_;
    fbl::unique_ptr<ddk::EthmacIfcProxy> ethmac_proxy_ __TA_GUARDED(lock_);

    // Only accessed from Thread, so not locked.
    bool online_ = false;
    zx::socket data_;

    thrd_t thread_;
};

}  // namespace eth
