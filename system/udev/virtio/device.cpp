// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "device.h"

#include "trace.h"

#include <stdio.h>

namespace virtio {

Device::Device() {
}

Device::~Device() {
    if (pci_config_handle_)
        mx_handle_close(pci_config_handle_);
    // TODO: close pci protocol and other handles
}

mx_status_t Device::Bind(mx_driver_t *driver, mx_device_t *device, pci_protocol_t *pci,
        mx_handle_t pci_config_handle, const pci_config_t *pci_config) {
    TRACE_ENTRY;

    // save off handles to things
    driver_ = driver;
    device_ = device;
    pci_ = pci;
    pci_config_handle_ = pci_config_handle;
    pci_config_ = pci_config;

    // claim the pci device
    mx_status_t r;
    r = pci->claim_device(device);
    if (r < 0)
        return r;

    // try to set up our IRQ mode
    if (pci->set_irq_mode(device, MX_PCIE_IRQ_MODE_MSI, 1)) {
        if (pci->set_irq_mode(device, MX_PCIE_IRQ_MODE_LEGACY, 1)) {
            TRACEF("failed to set irq mode\n");
            return -1;
        } else {
            TRACEF("using legacy irq mode\n");
        }
    }
    mx_handle_t irqh = pci->map_interrupt(device, 0);
    if (irqh < 0) {
        TRACEF("failed to map irq\n");
        return -1;
    }

    TRACE_EXIT;

    return NO_ERROR;
}

};
