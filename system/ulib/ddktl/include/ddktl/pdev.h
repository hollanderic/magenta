// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/protocol/platform-device.h>
#include <ddk/device.h>
#include <ddktl/mmio.h>
#include <fbl/ref_counted.h>
#include <fbl/ref_ptr.h>
#include <fbl/vector.h>


namespace ddk {


class Pdev : public fbl::RefCounted<Pdev> {

public:
    static fbl::RefPtr<Pdev> Create(zx_device_t* parent);
    void ShowInfo();

    MmioBlock GetMmio(uint32_t index);




private:

    Pdev(zx_device_t* parent) : parent_(parent) {};

    zx_device_t* parent_;
    platform_device_protocol_t pdev_;

    pdev_device_info_t pdev_info_;

    uint32_t mmio_count_ = 0;
    fbl::Vector<MmioBlock> mmio_;



};


} //namespace ddk


