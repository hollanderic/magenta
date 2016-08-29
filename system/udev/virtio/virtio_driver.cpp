// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/binding.h>
#include <ddk/protocol/pci.h>

#include <mxtl/unique_ptr.h>

#include <magenta/compiler.h>
#include <magenta/new.h>
#include <magenta/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "device.h"

#include "block/block.h"

extern "C" ssize_t virtio_read(mx_device_t* dev, void* buf, size_t count, mx_off_t off) {
    return 0;
}

extern "C" ssize_t virtio_write(mx_device_t* dev, const void* buf, size_t count, mx_off_t off) {
    return count;
}

#if 0
static mx_protocol_device_t virtio_device_proto = {
    .read = virtio_read,
    .write = virtio_write,
};
#endif

// implement driver object:

extern "C" mx_status_t virtio_init(mx_driver_t* driver) {
    printf("virtio_init: driver %p\n", driver);
#if 0
    mx_device_t* dev;
    if (device_create(&dev, driver, "virtio", &virtio_device_proto) == NO_ERROR) {
        if (device_add(dev, NULL) < 0) {
            free(dev);
        }
    }
#endif
    return NO_ERROR;
}

extern "C" mx_status_t virtio_bind(mx_driver_t* driver, mx_device_t* device) {
    printf("virtio_bind: driver %p, device %p\n", driver, device);

    /* grab the pci device and configuration */
    pci_protocol_t* pci;
    if (device_get_protocol(device, MX_PROTOCOL_PCI, (void**)&pci)) {
        printf("no pci protocol\n");
        return -1;
    }

    const pci_config_t *config;
    mx_handle_t config_handle = pci->get_config(device, &config);
    if (config_handle < 0) {
        printf("failed to grab config handle\n");
        return -1;
    }

    printf("pci %p\n", pci);
    printf("0x%x:0x%x\n", config->vendor_id, config->device_id);

    mxtl::unique_ptr<virtio::Device> vd = nullptr;
    AllocChecker ac;
    switch (config->device_id) {
        case 0x1001:
            printf("found block device\n");
            vd.reset(new (&ac) virtio::BlockDevice());
            if (!ac.check())
                return ERR_NO_MEMORY;
            break;
        default:
            printf("unhandled device id, how did this happen?\n");
            return -1;
    }

    auto status = vd->Bind(driver, device, pci, config_handle, config);
    if (status < 0)
        return status;

    return -1;

    return NO_ERROR;
}


