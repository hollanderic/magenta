// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/io-buffer.h>
#include <ddk/protocol/i2c.h>
#include <ddk/protocol/platform-device.h>
#include <ddktl/device.h>
#include <ddktl/device-internal.h>
#include <ddktl/mmio.h>
#include <zircon/listnode.h>
#include <lib/zx/bti.h>
#include <lib/zx/vmo.h>
#include <fbl/mutex.h>
#include <fbl/ref_ptr.h>
#include <fbl/unique_ptr.h>
#include <fbl/vector.h>

#include <soc/aml-common/aml-tdm.h>


class AmlTdmDevice : public fbl::unique_ptr<AmlTdmDevice> {

public:
    static fbl::unique_ptr<AmlTdmDevice> Create(ddk::MmioBlock&& mmio);


private:
    //static int IrqThread(void* arg);

    friend class fbl::unique_ptr<AmlTdmDevice>;

    AmlTdmDevice() { };
    void InitRegs();

    ddk::MmioBlock mmio_;
    //fbl::unique_ptr<ddk::MmioBlock<uint32_t>> uregs_;

    virtual ~AmlTdmDevice();


#if 0
    fbl::Mutex lock_;
    fbl::Mutex req_lock_ __TA_ACQUIRED_AFTER(lock_);

    // Dispatcher framework state
    fbl::RefPtr<dispatcher::Channel> stream_channel_ __TA_GUARDED(lock_);
    fbl::RefPtr<dispatcher::Channel> rb_channel_     __TA_GUARDED(lock_);
    fbl::RefPtr<dispatcher::ExecutionDomain> default_domain_;

    uint32_t ring_buffer_phys_  = 0;
    uint32_t ring_buffer_size_  = 0;
#endif
};
