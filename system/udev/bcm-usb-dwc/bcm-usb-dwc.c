// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "../bcm-common/usb_dwc_regs.h"

#define PAGE_MASK_4K       (0xFFF)
#define USB_PAGE_START     (USB_BASE & (~PAGE_MASK_4K))
#define USB_PAGE_SIZE      (0x4000)
#define PAGE_REG_DELTA     (USB_BASE - USB_PAGE_START)

static volatile struct dwc_regs* regs;

static mx_status_t usb_dwc_bind(mx_driver_t* drv, mx_device_t* dev) {
    mx_handle_t irq_handle = MX_HANDLE_INVALID; 
    mx_status_t st;

    // Carve out some device memory for this device.
    st = mx_mmap_device_memory(
        get_root_resource(), USB_PAGE_START, (uint32_t)USB_PAGE_SIZE, 
        MX_CACHE_POLICY_UNCACHED_DEVICE, (void*)(&regs)
    );
    if (st != NO_ERROR) {
        goto error_return;
    }

    // Create an IRQ Handle for this device.
    irq_handle = mx_interrupt_create(get_root_resource(), INTERRUPT_VC_USB, 
                                     MX_FLAG_REMAP_IRQ);
    if (irq_handle < 0) {
        st = ERR_NO_RESOURCES;
        goto error_return;
    }

    // TODO(gkalsi): Turn On USB Power Here.

    if ((st = usb_dwc_softreset_core()) != NO_ERROR) {
        goto error_return;
    }

    if ((st = usb_dwc_setupcontroller()) != NO_ERROR) {
        goto error_return;
    }

    // Initialize each of the host channels.
    for (size_t i = 0; i < NUM_HOST_CHANNELS; i++) {
        dwc_init_host_channel(&regs->host_channels[i]);
    }

    // Initialize the list of endpoint contexts.
    for (size_t i = 0; i < MAX_DEVICE_COUNT; i++) {
        list_initialize(&device_eps[i]);
    }

    dwc->irq_handle = irq_handle;
    dwc->parent = dev;

    device_init(&dwc->device, drv, "bcm-usb-dwc", &dwc_device_proto);

    dwc->device.protocol_id = MX_PROTOCOL_USB_HCI;
    dwc->device.protocol_ops = &dwc_hci_protocol;

    // TODO(gkalsi): Lock this list with a mutex?
    list_initialize(&pending_intr_reqs);

    thrd_t irq_thread;
    thrd_create_with_name(&irq_thread, dwc_irq_thread, dwc, "dwc_irq_thread");
    thrd_detach(irq_thread);

    dwc_start_scheduler_thread((struct dwc_regs*)regs);

    return NO_ERROR;

error_return:
    if (dwc)
        free(dwc);

    return st;
}

mx_driver_t _driver_usb_dwc = {
    .ops = {
        .bind = usb_dwc_bind,
    },
};

MAGENTA_DRIVER_BEGIN(_driver_usb_dwc, "bcm-usb-dwc", "magenta", "0.1", 3)
    BI_ABORT_IF(NE, BIND_SOC_VID, SOC_VID_BROADCOMM),
    BI_MATCH_IF(EQ, BIND_SOC_DID, SOC_DID_BROADCOMM_MAILBOX),
MAGENTA_DRIVER_END(_driver_usb_dwc)