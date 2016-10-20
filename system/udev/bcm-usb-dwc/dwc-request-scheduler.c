// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include "dwc-request-scheduler.h"

// #include <compiler.h>
#include <endian.h>
#include <hexdump/hexdump.h>
#include <magenta/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <threads.h>
#include <assert.h>

#include <ddk/completion.h>
#include <ddk/protocol/usb.h>

#include "../bcm-common/bcm28xx.h"
#include "../bcm-common/usb_dwc_regs.h"
#include "../bcm-common/usb_std_defs.h"
#include "../bcm-common/usb_hub_defs.h"
#include "../bcm-common/usb_core_driver.h"
#include "dwc-constants.h"

#define IS_WORD_ALIGNED(ptr) ((ulong)(ptr) % sizeof(ulong) == 0)
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

static volatile struct dwc_regs* regs;

enum dwc_usb_pid {
    DWC_USB_PID_DATA0 = 0,
    DWC_USB_PID_DATA1 = 2,
    DWC_USB_PID_DATA2 = 1,
    DWC_USB_PID_SETUP = 3,
};

mtx_t pending_transfer_mtx;
list_node_t pending_transfer_list;
completion_t pending_transfer_completion;

// static mtx_t pending_req_mtx;
// static struct dwc_transfer_request* channel_pending_xfers[DWC_NUM_CHANNELS];

static bool is_control_transfer(struct dwc_transfer_request* req);
static bool is_setup_phase(struct dwc_transfer_request* req);
static bool is_data_phase(struct dwc_transfer_request* req);
static bool is_status_phase(struct dwc_transfer_request* req);

#define ALL_CHANNELS_FREE 0xff
mtx_t free_channel_mtx;
completion_t free_channel_completion;
static uint8_t free_channels = ALL_CHANNELS_FREE;
static uint acquire_channel_blocking(void);

uint32_t prep_hc_characteristics(void) {
    union dwc_host_channel_characteristics characteristics;
    
    // Reset all parameters to 0
    characteristics.val = 0;

    characteristics.endpoint_number = 0;
    characteristics.low_speed = 0;
    characteristics.endpoint_type = USB_TRANSFER_TYPE_CONTROL;
    characteristics.packets_per_frame = 1;
    characteristics.device_address = 0;
    characteristics.odd_frame = 0;
    characteristics.channel_disable = 0;
    characteristics.channel_enable = 0;

    return characteristics.val;
}

void dwc_channel_start_transaction(struct dwc_transfer_request* req, uint32_t channel) {
    volatile struct dwc_host_channel *chanptr = &regs->host_channels[channel];
    union dwc_host_channel_split_control split_control;
    union dwc_host_channel_characteristics characteristics;
    union dwc_host_channel_interrupts interrupt_mask;
    unsigned int next_frame;

    chanptr->interrupt_mask.val = 0;
    chanptr->interrupts.val = 0xffffffff;

    split_control = chanptr->split_control;
    split_control.complete_split = req->complete_split;
    chanptr->split_control = split_control;

    next_frame = (regs->host_frame_number & 0xffff) + 1;

    characteristics = chanptr->characteristics;
    characteristics.odd_frame = next_frame & 1;
    characteristics.channel_enable = 1;

    chanptr->characteristics = characteristics;

    interrupt_mask.val = 0;
    interrupt_mask.channel_halted = 1;
    chanptr->interrupt_mask = interrupt_mask;
    regs->host_channels_interrupt_mask |= 1 << channel;
}

// Start a USB transfer on the specified USB channel.
static void dwc_start_transfer(struct dwc_transfer_request* req, uint32_t channel) {
    // printf("dwc_start_transfer req = %p, channel = %u\n", req, channel);
    assert(channel < DWC_NUM_CHANNELS);
    assert(req);

    iotxn_t* txn = req->txn;
    device_context_t* dev = req->dev_ctx;
    usb_protocol_data_t* pdata = iotxn_pdata(txn, usb_protocol_data_t);
    usb_setup_t* setup = (pdata->ep_address == 0 ? &pdata->setup : NULL);
    volatile struct dwc_host_channel *chanptr = &regs->host_channels[channel];

    if (txn == NULL) {
        printf("Iotxn is null in dwc_start_transfer\n");
    }

    union dwc_host_channel_split_control split_control;
    split_control.val = 0;

    union dwc_host_channel_characteristics characteristics;
    characteristics.val = prep_hc_characteristics();

    characteristics.device_address = 0;
    characteristics.max_packet_size = dev->max_packet_size;

    union dwc_host_channel_transfer transfer;
    transfer.val = 0;

    req->more_data = false;

    void *dataptr;

    if (is_control_transfer(req)) {
        if (is_setup_phase(req)) {
            transfer.size = sizeof(struct usb_control_setup_data);
            transfer.packet_id = DWC_USB_PID_SETUP;
            
            // Why do we need this second iotxn? There's a bug with this USB 
            // controller that doesn't allow us to use the same physical address
            // for both the SETUP transaction and the DATA transaction. 
            // TODO(gkalsi): Figure out a cleaner way to do this.
            // TODO(gkalsi): Free this iotxn.
            iotxn_t *setuptxn;
            iotxn_alloc(&setuptxn, 0, sizeof(struct usb_control_setup_data), 0);

            // TODO(gkalsi): Is this safe? What if there's a data out phase?
            characteristics.endpoint_direction = USB_DIRECTION_OUT;
            setuptxn->ops->copyto(setuptxn, setup, sizeof(struct usb_control_setup_data), 0);
            mx_paddr_t phys_addr;
            setuptxn->ops->physmap(setuptxn, &phys_addr);
            dataptr = (void*)phys_addr;

            // printf("Setup packet:\n");
            // printf("  bmRequestType=%u\n", setup->bmRequestType);
            // printf("  bRequest=%u\n", setup->bRequest);
            // printf("  wValue=%u\n", setup->wValue);
            // printf("  wIndex=%u\n", setup->wIndex);
            // printf("  wLength=%u\n", setup->wLength);
            // printf("  channel=%u\n", channel);
            // printf("  device_address=%u\n", pdata->device_id);

        } else if (is_data_phase(req)) {
            characteristics.endpoint_direction = (setup->bmRequestType >> 7);

            // TODO(gkalsi): What if the transaction is longer than 1 packet?
            // We need to send multiple data packets here.
            if (req->actual_size) {
                // printf("Actual size = %lu\n", req->actual_size);
                // size_t data_left = txn->length - req->actual_size;
                // uint8_t* temp = malloc(data_left);
                // txn->ops->copyfrom(txn, temp, data_left, req->actual_size);
                // txn->ops->copyto(txn, temp, data_left, 0);
                // free(temp);
                transfer.packet_id = req->next_data_pid;
            } else {
                transfer.packet_id = DWC_USB_PID_DATA1;
            }

            // printf("Data Txn pid = %u\n", transfer.packet_id);

            mx_paddr_t phys_addr;
            txn->ops->physmap(txn, &phys_addr);
            dataptr = (void*)phys_addr;
            dataptr += req->actual_size;
            transfer.size = txn->length - req->actual_size;
        } else if (is_status_phase(req)) {
            if ((setup->bmRequestType >> 7) == USB_DIRECTION_OUT || 
                setup->wLength == 0) {
                characteristics.endpoint_direction = USB_DIRECTION_IN;
            } else {
                characteristics.endpoint_direction = USB_DIRECTION_OUT;
            }
            dataptr = NULL;
            transfer.size = 0;
            transfer.packet_id = DWC_USB_PID_DATA1;
        } else {
            // TODO(gkalsi): Signal failure here with an assert or something.
            printf("[ERROR] Control should never reach here\n");
            return;
        }
    } else {
        uint8_t direction = pdata->ep_address & USB_ENDPOINT_DIR_MASK;
        if (direction == USB_DIR_IN) {
            characteristics.endpoint_direction = 1;
        } else {
            characteristics.endpoint_direction = 0;
        }

        mx_paddr_t phys_addr;

        txn->ops->physmap(txn, &phys_addr);
        dataptr = (void*)phys_addr;
        transfer.size = txn->length;
        transfer.packet_id = 0;
        
        characteristics.endpoint_type =
            req->ep_ctx->descriptor.bmAttributes & 0x3;
        characteristics.endpoint_number = pdata->ep_address & 0xF;
        characteristics.max_packet_size =
            req->ep_ctx->descriptor.wMaxPacketSize & 0x7ff;
        characteristics.packets_per_frame = 1;

        if (dev->speed == USB_SPEED_HIGH)
        {
            characteristics.packets_per_frame +=
                        ((req->ep_ctx->descriptor.wMaxPacketSize >> 11) & 0x3);
        }

        // printf("Non Control Transfer:\n");
        // printf("  characteristics.endpoint_type:%d\n", characteristics.endpoint_type);
        // printf("  characteristics.endpoint_number:%d\n", characteristics.endpoint_number);
        // printf("  characteristics.max_packet_size:%d\n", characteristics.max_packet_size);
    }

    // printf("Setup = %p\n", setup);
    // if (setup && pdata->device_id == 4) {
    //     if (
    //         setup->bmRequestType == 128 &&
    //         setup->bRequest == 6 &&
    //         // setup->wValue == 256 &&
    //         // setup->wLength == 18 &&
    //         setup->wIndex == 0
    //     ) {
    //         printf("Starting Trasnaction for bad packet\n");
    //     }
    // }

    if (pdata->device_id == 4 && characteristics.endpoint_number != 0 &&
        characteristics.endpoint_type == USB_TRANSFER_TYPE_INTERRUPT) {
        printf("Scheduling interrupt transfer for devid = 4\n");
    }

    characteristics.device_address = pdata->device_id;

    // if (dev->speed != USB_SPEED_HIGH) {
    if (dev->speed == USB_SPEED_LOW) {
        split_control.port_address = dev->parent_port;
        split_control.hub_address = dev->parent_hub;
        split_control.split_enable = 1;

        if (transfer.size > characteristics.max_packet_size) {
            transfer.size = characteristics.max_packet_size;
            req->more_data = true;
        }

        characteristics.low_speed = 1;
    }

    // if (!IS_WORD_ALIGNED(dataptr)) {
    //     printf("[ERROR] Data pointer is not word aligned, DMA will fail."
    //            "ptr = %p\n", dataptr);
    //     return;
    // }

    // TODO(gkalsi): This should be computed. See xinu/usb_dwc_hcd.c:1020 for
    // an example.
    transfer.packet_count = DIV_ROUND_UP(transfer.size, characteristics.max_packet_size);
    transfer.packet_count = transfer.packet_count == 0 ? 1 : transfer.packet_count;

    req->attempted_size = transfer.size;
    req->bytes_remaining = transfer.size;
    req->packets_remaining = transfer.packet_count;

    // Bus addresses must be 32 bits. If there are high bits set in the address
    // something is very wrong. 
    // TODO(gkalsi): This is not recoverable. Maybe make this an ASSERT?
    if (0xffffffff00000000 & ((uintptr_t)dataptr)) {
        printf("[ERROR] Data pointer is outside the 32bit address range. "
               "ptr = %p\n", dataptr);
        return;
    } 
    uintptr_t dma_ptr = ((uintptr_t)dataptr) & 0xffffffff;
    chanptr->dma_address = (uint32_t)(dma_ptr);

    // Stash this request for later.
    dwc_save_request(req, channel);

    chanptr->characteristics = characteristics;
    chanptr->split_control   = split_control;
    chanptr->transfer        = transfer;


    dwc_channel_start_transaction(req, channel);
}

static int dwc_xfer_scheduler_thread(void *arg) {
    regs = (struct dwc_regs*)arg;

    list_initialize(&pending_transfer_list);

    while (true) {
        // Wait for somebody to tell us to schedule a transfer.
        completion_wait(&pending_transfer_completion, MX_TIME_INFINITE);

        // Wait for a channel to become available.
        uint channel = acquire_channel_blocking();

        mtx_lock(&pending_transfer_mtx);
        struct dwc_transfer_request* req =
            list_remove_head_type(&pending_transfer_list, 
                                  struct dwc_transfer_request, node);

        if (list_is_empty(&pending_transfer_list)) {
            completion_reset(&pending_transfer_completion);
        }

        mtx_unlock(&pending_transfer_mtx);

        if (!req) {
            printf("Null request?\n");
            continue;
        }


        dwc_save_request(req, channel);

        dwc_start_transfer(req, channel);
    }

    __UNREACHABLE;
}

mx_status_t dwc_queue_transfer(struct dwc_transfer_request* req) {

    mtx_lock(&pending_transfer_mtx);
    
    assert(req);

    list_add_tail(&pending_transfer_list, &req->node);

    mtx_unlock(&pending_transfer_mtx); 

    completion_signal(&pending_transfer_completion);

    return NO_ERROR;
}

mx_status_t dwc_start_scheduler_thread(struct dwc_regs* regs) {
    thrd_t xfer_scheduler_thread;
    thrd_create_with_name(&xfer_scheduler_thread, dwc_xfer_scheduler_thread, 
                          (void*)regs, "dwc_xfer_scheduler_thread");
    thrd_detach(xfer_scheduler_thread);

    return NO_ERROR;
}

void do_channel_callback(uint channel) {
    struct dwc_transfer_request* req = dwc_get_request(channel);

    if (!req || !req->cb)
        return;

    req->cb(NO_ERROR, (void*)req);
}

static bool is_control_transfer(struct dwc_transfer_request* req) {
    iotxn_t *txn = req->txn;
    usb_protocol_data_t* pdata = iotxn_pdata(txn, usb_protocol_data_t);

    return pdata->ep_address == 0;

}

static inline bool is_setup_phase(struct dwc_transfer_request* req) {
    return req->txn_phase == PHASE_SETUP;
}

static inline bool is_data_phase(struct dwc_transfer_request* req) {
    return req->txn_phase == PHASE_DATA;
}

static inline bool is_status_phase(struct dwc_transfer_request* req) {
    return req->txn_phase == PHASE_STATUS;
}

uint acquire_channel_blocking(void) {
    int next_channel = -1;

    while (true) {
        mtx_lock(&free_channel_mtx);

        // A quick sanity check. We should never mark a channel that doesn't 
        // exist on the system as free.
        assert((free_channels & ALL_CHANNELS_FREE) == free_channels);

        // Is there at least one channel that's free?
        next_channel = -1;
        if (free_channels) {
            next_channel = __builtin_ctz(free_channels);
            
            // Mark the bit in the free_channel bitfield = 0, meaning the
            // channel is in use.
            free_channels &= (ALL_CHANNELS_FREE ^ (1 << next_channel)); 
        }

        mtx_unlock(&free_channel_mtx);

        if (next_channel >= 0) {
            return next_channel;
        }

        // We couldn't find a free channel, wait for somebody to tell us to 
        // wake up and attempt to acquire a channel again.
        completion_wait(&free_channel_completion, MX_TIME_INFINITE);
    }

    __UNREACHABLE;
}

void release_channel(uint ch) {
    assert(ch < DWC_NUM_CHANNELS);

    mtx_lock(&free_channel_mtx);

    free_channels |= (1 << ch);

    dwc_save_request(NULL, ch);

    mtx_unlock(&free_channel_mtx);

    completion_signal(&free_channel_completion);
}

