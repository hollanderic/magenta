// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/binding.h>

#include <magenta/compiler.h>
#include <magenta/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// virtio is the /dev/virtio device.

// implemented in virtio.cpp
extern mx_status_t virtio_init(mx_driver_t* driver);
extern mx_status_t virtio_bind(mx_driver_t* driver, mx_device_t* device);

static mx_bind_inst_t binding[] = {
    BI_ABORT_IF(NE, BIND_PROTOCOL, MX_PROTOCOL_PCI),
    BI_ABORT_IF(NE, BIND_PCI_VID, 0x1af4),
    //BI_MATCH_IF(EQ, BIND_PCI_DID, 0x1000), // Network device
    BI_MATCH_IF(EQ, BIND_PCI_DID, 0x1001), // Block device
    BI_ABORT(),
};

mx_driver_t _driver_virtio BUILTIN_DRIVER = {
    .name = "virtio",
    .ops = {
        .init = virtio_init,
        .bind = virtio_bind,
    },
    .binding = binding,
    .binding_size = sizeof(binding),
};
