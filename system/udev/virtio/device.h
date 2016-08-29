// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#pragma once

#include <magenta/types.h>

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/protocol/pci.h>

namespace virtio {

class Device {
public:
    Device();
    virtual ~Device();

    mx_device_t *device() { return device_; }

    mx_status_t Bind(mx_driver_t *, mx_device_t *, pci_protocol_t *, mx_handle_t pci_config_handle, const pci_config_t *);

private:
    mx_driver_t *driver_ = nullptr;
    mx_device_t *device_ = nullptr;
    pci_protocol_t *pci_ = nullptr;
    mx_handle_t pci_config_handle_ = 0;
    const pci_config_t *pci_config_ = nullptr;

};

};
