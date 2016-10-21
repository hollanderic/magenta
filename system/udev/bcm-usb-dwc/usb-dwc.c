// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <threads.h>
#include <string.h>
#include <assert.h>

#include <ddk/binding.h>
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/completion.h>
#include <ddk/common/usb.h>
#include <ddk/protocol/bcm.h>
#include <ddk/protocol/usb-bus.h>
#include <ddk/protocol/usb-hci.h>
#include <ddk/protocol/usb.h>
#include <magenta/hw/usb-hub.h>
#include <magenta/hw/usb.h>
#include <magenta/listnode.h>
#include <magenta/types.h>
#include <mxio/util.h>
#include <hexdump/hexdump.h>

#include "../bcm-common/bcm28xx.h"
#include "../bcm-common/usb_dwc_regs.h"
#include "../bcm-common/usb_std_defs.h"
#include "../bcm-common/usb_hub_defs.h"
#include "../bcm-common/usb_core_driver.h"
#include "dwc-request-scheduler.h"
#include "dwc-constants.h"

#define PAGE_MASK_4K       (0xFFF)
#define USB_PAGE_START     (USB_BASE & (~PAGE_MASK_4K))
#define USB_PAGE_SIZE      (0x4000)
#define PAGE_REG_DELTA     (USB_BASE - USB_PAGE_START)
#define MAX_DEVICE_COUNT   64
#define ROOT_HUB_COUNT     1
#define ROOT_HUB_DEVICE_ID MAX_DEVICE_COUNT + 1


#if LTRACE
#define xprintf(fmt...) printf(fmt)
#else
#define xprintf(fmt...) \
    do {                \
    } while (0)
#endif


static mtx_t root_port_status_mtx;
static volatile struct dwc_regs* regs;
static volatile usb_port_status_t root_port_status;
static volatile bool root_port_status_changed = false;

static mtx_t stashed_request_mtx;
static struct dwc_transfer_request* stashed_requests[NUM_HOST_CHANNELS];

// TODO(gkalsi): Hand these out and reclaim them dynamically.
static uint next_device_address = 2;

list_node_t pending_intr_reqs;

mtx_t pending_transfer_mutex;
completion_t pending_transfer_completion;

typedef struct usb_dwc {
    mx_device_t device;

    mx_device_t* bus_device;
    usb_bus_protocol_t* bus_protocol;

    mx_handle_t irq_handle;
    thrd_t irq_thread;
    thrd_t device_thread;

    mx_device_t* parent;
} usb_dwc_t;

list_node_t device_eps[MAX_DEVICE_COUNT];

mtx_t devctx_mtx;
device_context_t* devctxs[MAX_DEVICE_COUNT];

typedef void (*dwc_transfer_complete_cb)(mx_status_t result, void* data);

enum dwc_intr_status {
    XFER_COMPLETE            = 0,
    XFER_FAILED              = 1,
    XFER_NEEDS_DEFERRAL      = 2,
    XFER_NEEDS_RESTART       = 3,
    XFER_NEEDS_TRANS_RESTART = 4,
};

// typedef struct {
//     iotxn_t* txn;
//     // int ep_num;
//     // uint16_t length;
//     // usb_setup_t* setup;
//     dwc_transfer_complete_cb callback;
//     uint8_t control_phase;
//     list_node_t node;
// } dwc_transfer_request;

// device descriptor for USB 2.0 root hub
// represented as a byte array to avoid endianness issues
static const uint8_t dwc_rh_descriptor[sizeof(usb_device_descriptor_t)] = {
    sizeof(usb_device_descriptor_t),    // bLength
    USB_DT_DEVICE,                      // bDescriptorType
    0x00, 0x02,                         // bcdUSB = 2.0
    USB_CLASS_HUB,                      // bDeviceClass
    0,                                  // bDeviceSubClass
    0,                                  // bDeviceProtocol = Single TT
    64,                                 // bMaxPacketSize0
    0xD1, 0x18,                         // idVendor = 0x18D1 (Google)
    0x02, 0xA0,                         // idProduct = 0xA002
    0x00, 0x01,                         // bcdDevice = 1.0
    0,                                  // iManufacturer
    1,                                  // iProduct
    0,                                  // iSerialNumber
    1,                                  // bNumConfigurations
};

#define CONFIG_DESC_SIZE sizeof(usb_configuration_descriptor_t) + \
                         sizeof(usb_interface_descriptor_t) + \
                         sizeof(usb_endpoint_descriptor_t)

// we are currently using the same configuration descriptors for both USB 2.0 and 3.0 root hubs
// this is not actually correct, but our usb-hub driver isn't sophisticated enough to notice
static const uint8_t dwc_rh_config_desc[CONFIG_DESC_SIZE] = {
    // config descriptor
    sizeof(usb_configuration_descriptor_t),    // bLength
    USB_DT_CONFIG,                             // bDescriptorType
    CONFIG_DESC_SIZE, 0,                       // wTotalLength
    1,                                         // bNumInterfaces
    1,                                         // bConfigurationValue
    0,                                         // iConfiguration
    0xE0,                                      // bmAttributes = self powered
    0,                                         // bMaxPower
    // interface descriptor
    sizeof(usb_interface_descriptor_t),         // bLength
    USB_DT_INTERFACE,                           // bDescriptorType
    0,                                          // bInterfaceNumber
    0,                                          // bAlternateSetting
    1,                                          // bNumEndpoints
    USB_CLASS_HUB,                              // bInterfaceClass
    0,                                          // bInterfaceSubClass
    0,                                          // bInterfaceProtocol
    0,                                          // iInterface
    // endpoint descriptor
    sizeof(usb_endpoint_descriptor_t),          // bLength
    USB_DT_ENDPOINT,                            // bDescriptorType
    USB_ENDPOINT_IN | 1,                        // bEndpointAddress
    USB_ENDPOINT_INTERRUPT,                     // bmAttributes
    4, 0,                                       // wMaxPacketSize
    0xff,                                       // bInterval
};

#define dev_to_usb_dwc(dev) containerof(dev, usb_dwc_t, device)
static mx_status_t dwc_do_iotxn_queue(iotxn_t* txn);
static void dwc_control_status_complete(mx_status_t result, void* data);
static void dwc_control_data_complete(mx_status_t result, void* data);
static void dwc_control_setup_complete(mx_status_t result, void* data);

static mx_status_t usb_dwc_softreset_core(void) {
    while (!(regs->core_reset & DWC_AHB_MASTER_IDLE));

    regs->core_reset = DWC_SOFT_RESET;
    while (regs->core_reset & DWC_SOFT_RESET);

    return NO_ERROR;
}

static bool dwc_is_root_hub(uint32_t device_id) {
    return device_id > MAX_DEVICE_COUNT;
}

static bool usb_is_control_request(struct dwc_transfer_request* req) {
    // XXXXXXXXXXXXXXX
    // TODO(gkalsi): THIS IS COMPLETELY BROKEN. 
    return req->ep_ctx == NULL;
}

void dwc_save_request(struct dwc_transfer_request* req, unsigned int chan) {
    mtx_lock(&stashed_request_mtx);
    stashed_requests[chan] = req;
    mtx_unlock(&stashed_request_mtx);
}

struct dwc_transfer_request* dwc_get_request(unsigned int chan) {
    assert(chan < NUM_HOST_CHANNELS);
    struct dwc_transfer_request* result = NULL;

    mtx_lock(&stashed_request_mtx);
    result = stashed_requests[chan];
    mtx_unlock(&stashed_request_mtx);

    return result;
}

static mx_status_t usb_dwc_setupcontroller(void) {
    const uint32_t rx_words = 1024;
    const uint32_t tx_words = 1024;
    const uint32_t ptx_words = 1024;

    regs->rx_fifo_size = rx_words;
    regs->nonperiodic_tx_fifo_size = (tx_words << 16) | rx_words;
    regs->host_periodic_tx_fifo_size = (ptx_words << 16) | (rx_words + tx_words);

    regs->ahb_configuration |= DWC_AHB_DMA_ENABLE | BCM_DWC_AHB_AXI_WAIT;

    union dwc_core_interrupts core_interrupt_mask;

    regs->core_interrupt_mask.val = 0;
    regs->core_interrupts.val = 0xffffffff;

    core_interrupt_mask.val = 0;
    core_interrupt_mask.host_channel_intr = 1;
    core_interrupt_mask.port_intr = 1;
    regs->core_interrupt_mask = core_interrupt_mask;

    regs->ahb_configuration |= DWC_AHB_INTERRUPT_ENABLE;

    return NO_ERROR;
}

static union dwc_host_port_ctrlstatus dwc_get_host_port_ctrlstatus(void) {
    union dwc_host_port_ctrlstatus hw_status = regs->host_port_ctrlstatus;

    hw_status.enabled = 0;
    hw_status.connected_changed = 0;
    hw_status.enabled_changed = 0;
    hw_status.overcurrent_changed = 0;
    return hw_status;
}

static void
dwc_power_on_host_port(void) {
    union dwc_host_port_ctrlstatus hw_status;

    hw_status = dwc_get_host_port_ctrlstatus();
    hw_status.powered = 1;
    regs->host_port_ctrlstatus = hw_status;
}

static void dwc_reset_host_port(void) {
    union dwc_host_port_ctrlstatus hw_status = regs->host_port_ctrlstatus;
    hw_status.enabled = 0;
    hw_status.connected_changed = 0;
    hw_status.enabled_changed = 0;
    hw_status.overcurrent_changed = 0;

    hw_status.reset = 1;
    regs->host_port_ctrlstatus = hw_status;
    usleep(60000);
    hw_status.reset = 0;
    regs->host_port_ctrlstatus = hw_status;
    usleep(60000);
}

static mx_status_t dwc_complete_root_port_status_txn(void) {

    if (!root_port_status.wPortChange) 
        return NO_ERROR;

    iotxn_t* txn = list_remove_head_type(&pending_intr_reqs, iotxn_t, node);
    if (!txn)
        return NO_ERROR;


    // txn->ops->copyto(txn, (void*)&root_port_status, sizeof(root_port_status), 0);
    uint16_t val = 0x2;
    txn->ops->copyto(txn, (void*)&val, sizeof(val), 0);
    txn->ops->complete(txn, NO_ERROR, sizeof(val));
    return NO_ERROR;
}

static enum dwc_intr_status dwc_handle_channel_halted(struct dwc_transfer_request* req, unsigned int channel) {
    assert(channel < NUM_HOST_CHANNELS);

    volatile struct dwc_host_channel* chanptr = &regs->host_channels[channel];
    union dwc_host_channel_interrupts interrupts = chanptr->interrupts;

    uint packets_remaining = chanptr->transfer.packet_count;
    uint packets_transferred = req->packets_remaining - packets_remaining;

    if (packets_transferred == 0) {
        if (interrupts.ack_response_received && chanptr->split_control.split_enable && !req->complete_split) {

        } else {
            xprintf("packets_transferred = %u, packets_remaining = %u, req->packets_remaining = %lu\n",
                   packets_transferred, packets_remaining, req->packets_remaining);
        }
    }

    if (packets_transferred != 0) {
        size_t bytes_transferred = 0;
        union dwc_host_channel_characteristics characteristics =
                                            chanptr->characteristics;
        uint max_packet_size = characteristics.max_packet_size;
        enum usb_direction dir = characteristics.endpoint_direction;
        enum usb_transfer_type type = characteristics.endpoint_type;

        if (dir == USB_DIRECTION_IN) {
            bytes_transferred = req->bytes_remaining - chanptr->transfer.size;
            // TODO(gkalsi): Is buffer word aligned?
        } else {
            if (packets_transferred > 1) {
                bytes_transferred += max_packet_size * (packets_transferred - 1);
            }

            if (packets_remaining == 0 &&
                (req->attempted_size % max_packet_size != 0 || req->attempted_size == 0)) {
                bytes_transferred += req->attempted_size % max_packet_size;
            } else {
                bytes_transferred += max_packet_size;
            }
        }


        req->packets_remaining -= packets_transferred;
        req->bytes_remaining -= bytes_transferred;
        req->data_index += bytes_transferred;

        // This is an artefact of reusing the same transfer request between 
        // phases of a transfer
        if (usb_is_control_request(req) && req->txn_phase == PHASE_SETUP) {
            req->data_index = 0;
        }

        if (req->packets_remaining == 0 || 
            (dir == USB_DIRECTION_IN &&
             bytes_transferred < packets_transferred * max_packet_size)) {
            if (!interrupts.transfer_completed) {
                xprintf("TRANSFER FAILED!");
                return XFER_FAILED;
            }

            // xprintf("Return req = %p, is_control = %u, chan = %u\n", req, usb_is_control_request(req), channel);

            if (req->more_data && req->bytes_remaining == 0 &&
                type != USB_TRANSFER_TYPE_INTERRUPT) {
                req->complete_split = false;
                req->next_data_pid = chanptr->transfer.packet_id;
                // xprintf("Restart trasnfer with pid = %u\n", req->next_data_pid);
                if (!usb_is_control_request(req) || req->txn_phase == PHASE_DATA) {
                    req->actual_size = req->data_index;
                }
                if (req->cb == dwc_control_data_complete) {
                    req->cb = dwc_control_setup_complete;
                }
                // xprintf("Restart Xfer 1\n");
                return XFER_NEEDS_RESTART;
            }

            // TODO(gkalsi): Put this in the callback.
            if (usb_is_control_request(req) && req->txn_phase < PHASE_STATUS) {
                req->complete_split = false;
                // xprintf("Restart Xfer 2\n");
                return XFER_NEEDS_RESTART;
            }

            // xprintf("Complete req = %p, is_control = %u\n", req, usb_is_control_request(req));

            return XFER_COMPLETE;
        } else {
            if (chanptr->split_control.split_enable) {
                req->complete_split = !req->complete_split;
                // xprintf("Inverting Complete Split. Complete split is now %d\n", req->complete_split);
            }
            return XFER_NEEDS_TRANS_RESTART;
        }
    } else {
        if (interrupts.ack_response_received &&
            chanptr->split_control.split_enable && !req->complete_split) {
            req->complete_split = true;
            // xprintf("Enabling Complete Split. Complete split is now %d\n", req->complete_split);
            return XFER_NEEDS_TRANS_RESTART;
        } else {
            xprintf("Xfer Failed:\n");
            xprintf("  interrupts.ack_response_received: %d\n", interrupts.ack_response_received);
            xprintf("  interrupts.val: 0x%x\n", interrupts.val);
            xprintf("  chanptr->split_control.split_enable: %d\n", chanptr->split_control.split_enable);
            xprintf("  req->complete_split: %d\n", req->complete_split);
            return XFER_FAILED;
        }
    }


}

bool dwc_handle_host_channel_interrupt(unsigned int channel) {
    volatile struct dwc_host_channel* chanptr = &regs->host_channels[channel];
    union dwc_host_channel_interrupts interrupts = chanptr->interrupts;

    // xprintf("Channel interrupt = 0x%x\n", chanptr->interrupts.val);
    enum dwc_intr_status intr_status;

    struct dwc_transfer_request* req = dwc_get_request(channel);

    if ((uintptr_t)req < 0x30) {
        xprintf("Request is Null! - %p\n",req);
    }

    if ((uintptr_t)req->txn < 0x30) {
        xprintf("Request Transaction is Null! Channel = %u\n", channel);
    }

    // iotxn_t* txn = req->txn;
    // usb_protocol_data_t* pdata = iotxn_pdata(txn, usb_protocol_data_t);
    // if (pdata->device_id == 4 && pdata->ep_address == 0) {
    //     xprintf("txn devid = %d, ep = %d\n", pdata->device_id, pdata->ep_address);
    // }

    // iotxn_t* txn = request->txn;
    // usb_protocol_data_t* data = iotxn_pdata(txn, usb_protocol_data_t);
    // usb_setup_t* setup = (data->ep_address == 0 ? &data->setup : NULL);

    // if (chanptr->characteristics.device_address == 4) {
    //     xprintf("Intr for DevID = 4, intr = 0x%x\n", interrupts.val);
    // }

    if (interrupts.stall_response_received || interrupts.ahb_error ||
        interrupts.transaction_error || interrupts.babble_error ||
        interrupts.excess_transaction_error || interrupts.frame_list_rollover ||
        (interrupts.nyet_response_received && !req->complete_split) ||
        (interrupts.data_toggle_error &&
         chanptr->characteristics.endpoint_direction == USB_DIRECTION_OUT))
    {
        xprintf("Transfer failed with status = 0x%x\n", interrupts.val);
        xprintf("  characteristics = 0x%x\n", chanptr->characteristics.val);
        xprintf("    split_control = 0x%x\n", chanptr->split_control.val);
        xprintf("         transfer = 0x%x\n", chanptr->transfer.val);
        return true;   
    } else if (interrupts.frame_overrun)
    {
        intr_status = XFER_NEEDS_TRANS_RESTART;
    }
    else if (interrupts.nyet_response_received)
    {
        req->complete_split = false;
        intr_status = XFER_NEEDS_TRANS_RESTART;
    }
    else if (interrupts.nak_response_received)
    {
        intr_status = XFER_NEEDS_DEFERRAL;
        req->complete_split = false;
    } 
    else {
        intr_status = dwc_handle_channel_halted(req, channel);
    }


    iotxn_t* txn = req->txn;
    usb_protocol_data_t* pdata = iotxn_pdata(txn, usb_protocol_data_t);
    if (pdata->device_id == 4 && chanptr->characteristics.endpoint_number != 0 &&
        chanptr->characteristics.endpoint_type == USB_TRANSFER_TYPE_INTERRUPT) {
        // xprintf("IRQ for devid = 4. Interrupts = %x\n", interrupts.val);
    }

    // usb_setup_t* setup = (pdata->ep_address == 0 ? &pdata->setup : NULL);
    // if (setup) {
    //     if (
    //         setup->bmRequestType == 128 &&
    //         setup->bRequest == 6 &&
    //         // setup->wValue == 256 &&
    //         // setup->wLength == 18 &&
    //         setup->wIndex == 0
    //     ) {
    //         xprintf("X\n");
    //     }
    // }

    // xprintf("Handle Channel Halted Complete\n");

    // if (pdata->device_id == 4) {
    //     xprintf("txn devid = %d, ep = %d\n", pdata->device_id, pdata->ep_address);
    // }

    if (intr_status == XFER_COMPLETE) {
        union dwc_host_channel_characteristics characteristics;
        characteristics.val = 0;
        chanptr->characteristics = characteristics;
        chanptr->interrupt_mask.val = 0;
        chanptr->interrupts.val = 0xffffffff;

        //xprintf("Complete txn req = %p with length = %lu\n", req, req->txn->length);

        req->txn->ops->complete(req->txn, NO_ERROR, req->txn->length);
        return true;
    } else if (intr_status == XFER_NEEDS_TRANS_RESTART) {
        // xprintf("Restarting transaction with CSplit\n");
        dwc_channel_start_transaction(req, channel);
        return false;
    } else if (intr_status == XFER_NEEDS_RESTART) {
        union dwc_host_channel_characteristics characteristics;
        characteristics.val = 0;
        chanptr->characteristics = characteristics;
        chanptr->interrupt_mask.val = 0;
        chanptr->interrupts.val = 0xffffffff;

        return true;
    } else if (intr_status == XFER_NEEDS_DEFERRAL) {
        union dwc_host_channel_characteristics characteristics;
        characteristics.val = 0;
        chanptr->characteristics = characteristics;
        chanptr->interrupt_mask.val = 0;
        chanptr->interrupts.val = 0xffffffff;

        iotxn_t* txn = req->txn;
        usb_protocol_data_t* pdata = iotxn_pdata(txn, usb_protocol_data_t);
        if (pdata->device_id == 4 && chanptr->characteristics.endpoint_number != 0 &&
            chanptr->characteristics.endpoint_type == USB_TRANSFER_TYPE_INTERRUPT) {
            xprintf("Deferring Transfer for devid = 4, interrupts = %x\n", interrupts.val);
        }

        release_channel(channel);
        if (req == NULL) {
            // dwc_queue_transfer(req);
            xprintf("Can't defer xfer because request is null!\n");
            return false;
        }

        // Get the endpoint context.
        endpoint_context_t* ep_ctx = req->ep_ctx;

        if (ep_ctx == NULL) {
            dwc_queue_transfer(req);
            // xprintf("Can't defer xfer because ep context is null!\n");
            return false;
        }

        // Set the pending request.
        ep_ctx->pending_request = req;

        // Signal the completion.
        completion_signal(&ep_ctx->intr_ep_completion);

        return false;
    }

    return true;
}

mx_status_t dwc_alloc_request(struct dwc_transfer_request** req) {
    struct dwc_transfer_request* result = calloc(1, sizeof(*result));
    if (!result) 
        return ERR_NO_MEMORY;

    *req = result;
    return NO_ERROR;
}

static void dwc_control_status_complete(mx_status_t result, void* data) {
    // xprintf("STATUS transaction completed\n");
}

static void dwc_control_data_complete(mx_status_t result, void* data) {
    // xprintf("DATA txn complete, starting STATUS txn\n");
    struct dwc_transfer_request* req = (struct dwc_transfer_request*)data;
    req->txn_phase = PHASE_STATUS;
    req->cb = dwc_control_status_complete;

    dwc_queue_transfer(req);
}

static void dwc_control_setup_complete(mx_status_t result, void* data) {
    // xprintf("SETUP txn complete. ");
    struct dwc_transfer_request* req = (struct dwc_transfer_request*)data;
    iotxn_t *txn = req->txn;

    if (txn->length) {
        // xprintf("Starting DATA txn.\n");
        req->txn_phase = PHASE_DATA;
        req->cb = dwc_control_data_complete;
    } else {
        // xprintf("Starting STATUS txn.\n");
        req->txn_phase = PHASE_STATUS;
        req->cb = dwc_control_status_complete;
    }
    dwc_queue_transfer(req);
}

static inline ulong first_set_bit(ulong word) {
    return 31 - __builtin_clz(word);
}

uint32_t dwc_handle_interrupt(void) {
    union dwc_core_interrupts interrupts = regs->core_interrupts;

    union dwc_core_interrupts core_interrupt_ack;
    core_interrupt_ack.val = 0;
    uint32_t result = 0;

    if (interrupts.port_intr) {
        // Clear the interrupt.
        union dwc_host_port_ctrlstatus hw_status = regs->host_port_ctrlstatus;

        mtx_lock(&root_port_status_mtx);

        root_port_status.wPortChange = 0;
        root_port_status.wPortStatus = 0;

        // This device only has one port.
        if (hw_status.connected)
            root_port_status.wPortStatus |= USB_PORT_CONNECTION;
        if (hw_status.enabled)
            root_port_status.wPortStatus |= USB_PORT_ENABLE;
        if (hw_status.suspended)
            root_port_status.wPortStatus |= USB_PORT_SUSPEND;
        if (hw_status.overcurrent)
            root_port_status.wPortStatus |= USB_PORT_OVER_CURRENT;
        if (hw_status.reset)
            root_port_status.wPortStatus |= USB_PORT_RESET;
        if (hw_status.speed == USB_SPEED_LOW)
            root_port_status.wPortStatus |= USB_PORT_LOW_SPEED;
        if (hw_status.speed == USB_SPEED_HIGH)
            root_port_status.wPortStatus |= USB_PORT_HIGH_SPEED;

        if (hw_status.connected_changed)
            root_port_status.wPortChange |= USB_PORT_CONNECTION;
        if (hw_status.enabled_changed)
            root_port_status.wPortChange |= USB_PORT_ENABLE;
        if (hw_status.overcurrent_changed)
            root_port_status.wPortChange |= USB_PORT_OVER_CURRENT;

        mtx_unlock(&root_port_status_mtx);

        // Clear the interrupt.
        hw_status.enabled = 0;
        regs->host_port_ctrlstatus = hw_status;

        dwc_complete_root_port_status_txn();

        core_interrupt_ack.port_intr = 1;
    }

    if (interrupts.host_channel_intr) {
        uint32_t chintr = regs->host_channels_interrupt;
        xprintf("Channel interrupts = 0x%x\n", chintr);
        do {
            unsigned int chan = first_set_bit(chintr);
            if (dwc_handle_host_channel_interrupt(chan))
                result |= (1 << chan);

            chintr ^= (1 << chan);
        } while (chintr != 0);

        core_interrupt_ack.host_channel_intr = 1;
    }

    regs->core_interrupts = core_interrupt_ack;

    return result;
}

static int dwc_irq_thread(void* arg) {
    usb_dwc_t* dwc = (usb_dwc_t*)arg;

    device_add(&dwc->device, dwc->parent);
    dwc->parent = NULL;

    while (1) {
        mx_status_t wait_res;

        wait_res = mx_handle_wait_one(dwc->irq_handle, MX_SIGNAL_SIGNALED,
                                      MX_TIME_INFINITE, NULL);
        if (wait_res != NO_ERROR)
            xprintf("dwc_irq_thread::mx_handle_wait_one(irq_handle) returned "
                   "error code = %d\n", wait_res);

        uint32_t completed_channels = dwc_handle_interrupt();

        mx_interrupt_complete(dwc->irq_handle);

        for (int i = 0; i < NUM_HOST_CHANNELS; i++) {
            if (completed_channels & (1 << i)) {
                do_channel_callback(i);
                release_channel(i);
            }
        }

    }

    xprintf("dwc_irq_thread done.\n");
    return 0;
}

static void dwc_add_root_hub(usb_dwc_t* dwc) {
    dwc->bus_protocol->add_device(dwc->bus_device, MAX_DEVICE_COUNT + 1, 0,
                                  USB_SPEED_HIGH);
}

static void dwc_set_bus_device(mx_device_t* device, mx_device_t* busdev) {
    usb_dwc_t* dwc = dev_to_usb_dwc(device);
    dwc->bus_device = busdev;
    if (busdev) {
        device_get_protocol(busdev, MX_PROTOCOL_USB_BUS, (void**)&dwc->bus_protocol);
        dwc_reset_host_port();
        dwc_add_root_hub(dwc);
    } else {
        dwc->bus_protocol = NULL;
    }
}

static size_t dwc_get_max_device_count(mx_device_t* device) {
    // xprintf("[UNIMPLEMENTED]: dwc_get_max_device_count\n");
    return MAX_DEVICE_COUNT + ROOT_HUB_COUNT + 1;
}

static int dwc_intr_retry_thread(void* arg) {
    endpoint_context_t* ep_ctx = (endpoint_context_t*)arg;
    uint32_t txns_queued = 0;
    xprintf("Top of dwc_intr_retry_thread.\n");
    while (true) {
        completion_wait(&ep_ctx->intr_ep_completion, MX_TIME_INFINITE);
        completion_reset(&ep_ctx->intr_ep_completion);

        txns_queued++;

        struct dwc_transfer_request* req = ep_ctx->pending_request;
        if (!req) {
            xprintf("intr compleition signalled but no request pending?\n");
            continue;
        }


        usb_protocol_data_t* proto_data = iotxn_pdata(req->txn, usb_protocol_data_t);

        if (txns_queued % 250 == 0 && proto_data->device_id == 4) {
            xprintf("%u transactions queued\n", txns_queued);
        }

        uint32_t timeout_ms = 0;
        if (req->dev_ctx->speed == USB_SPEED_HIGH) {
            timeout_ms = (1 << (ep_ctx->descriptor.bInterval - 1)) / 8;
        } else {
            timeout_ms = ep_ctx->descriptor.bInterval;
        }
        timeout_ms = timeout_ms / 2;
        timeout_ms = timeout_ms ? timeout_ms : 1;

        // xprintf("Retrying req = %p after %ums\n", req, timeout_ms);

        // Wait before retrying the transfer.
        mx_nanosleep(MX_MSEC(timeout_ms));

        dwc_queue_transfer(req);
    }

    return -1;
}

static mx_status_t dwc_enable_ep(mx_device_t* hci_device, uint32_t device_id,
                                  usb_endpoint_descriptor_t* ep_desc, bool enable) {
    xprintf("Enabling Endpoint on Device = %u\n", device_id);
    xprintf("  bLength = %u\n", ep_desc->bLength);
    xprintf("  bDescriptorType = %u\n", ep_desc->bDescriptorType);
    xprintf("  bEndpointAddress = %u\n", ep_desc->bEndpointAddress);
    xprintf("  bmAttributes = %u\n", ep_desc->bmAttributes);
    xprintf("  wMaxPacketSize = %u\n", ep_desc->wMaxPacketSize);
    xprintf("  bInterval = %u\n", ep_desc->bInterval);

    // Do nothing for root hubs.
    if (dwc_is_root_hub(device_id)) {
        return NO_ERROR;
    }

    if (enable) {
        // Create a new endpoint context.
        endpoint_context_t* ep_ctx = calloc(1, sizeof(*ep_ctx));
        memcpy(&ep_ctx->descriptor, ep_desc, sizeof(*ep_desc));
        list_add_tail(&device_eps[device_id], &ep_ctx->node);

        // For interrupt endpoinds, we also need to start a thread to retry 
        // requests that are NAK'd by the device.

        if (usb_ep_type(&ep_ctx->descriptor) == USB_ENDPOINT_INTERRUPT) {
            thrd_create(&ep_ctx->intr_ep_worker, dwc_intr_retry_thread, ep_ctx);
        }
    } else {
        // TODO(gkalsi);
        // Remove the endpoint from the list of endpoints from this device and
        // free the memory that it has allocated.
    }


    return NO_ERROR;
}

static uint64_t dwc_get_frame(mx_device_t* hci_device) {
    xprintf("[UNIMPLEMENTED]: dwc_get_frame\n");
    return NO_ERROR;
}

mx_status_t dwc_config_hub(mx_device_t* hci_device, uint32_t device_id, usb_speed_t speed,
                            usb_hub_descriptor_t* descriptor) {
    xprintf("[UNIMPLEMENTED]: dwc_config_hub\n");
    return NO_ERROR;
}

static void usb_control_complete(iotxn_t* txn, void* cookie) {
    completion_signal((completion_t*)cookie);
}

#define SWAP_16(x) \
    ((((uint16_t)(x) & 0xff) << 8) | ((uint16_t)(x) >> 8))

mx_status_t dwc_hub_device_added(mx_device_t* hci_device, uint32_t hub_address, int port,
                                  usb_speed_t speed) {
    usb_dwc_t* dwc = dev_to_usb_dwc(hci_device);
    xprintf("dwc_hub_device_added: hub_address = %u, port = %d, speed = %d "
           "hci_device = %p\n", hub_address, port, speed, hci_device);

    // TODO(gkalsi): Maybe do all this in a thread?
    iotxn_t *getdesc_txn;

    mx_status_t status = iotxn_alloc(&getdesc_txn, 0, 64, 0);
    if (status != NO_ERROR) {
        xprintf("iotxn_alloc failed\n");
        return status;
    }

    getdesc_txn->protocol = MX_PROTOCOL_USB;
    usb_protocol_data_t* proto_data = iotxn_pdata(getdesc_txn, usb_protocol_data_t);
    completion_t completion = COMPLETION_INIT;
    completion_t completion2 = COMPLETION_INIT;
    proto_data->ep_address = 0;
    proto_data->frame = 0;
    proto_data->device_id = 0;
    getdesc_txn->complete_cb = usb_control_complete;
    getdesc_txn->cookie = &completion;

    // Get Descriptor to determine MPS.
    proto_data->setup.bmRequestType = 0x80;
    proto_data->setup.bRequest = 0x06;
    proto_data->setup.wValue = SWAP_16(0x1);    // Device Descriptor
    proto_data->setup.wIndex = 0;
    proto_data->setup.wLength = 8;            // Minimum possible packet size.
    getdesc_txn->length = 8;

    device_context_t* dev_ctx = calloc(1, sizeof(*dev_ctx));
    dev_ctx->parent_port = port;
    dev_ctx->parent_hub = hub_address;
    dev_ctx->speed = speed;
    dev_ctx->max_packet_size = 8;   // Until we can determine what the MPS really is.
    
    struct dwc_transfer_request* req;
    dwc_alloc_request(&req);    // TODO(gkalsi): Check return value.
    req->txn = getdesc_txn;
    req->txn_phase = PHASE_SETUP;
    req->cb = dwc_control_setup_complete;
    req->ep_ctx = NULL;
    req->dev_ctx = dev_ctx;

    xprintf("Queue transfer to read short descriptor\n");

    dwc_queue_transfer(req);
    completion_wait(&completion, MX_TIME_INFINITE);

    xprintf("Attempting to copy %lu bytes\n", getdesc_txn->actual);

    usb_device_descriptor_t short_descriptor;
    getdesc_txn->ops->copyfrom(getdesc_txn, &short_descriptor, getdesc_txn->actual, 0);

    // xprintf("Got 8 byte Device Descriptor. Device MPS is %u\n", short_descriptor.bMaxPacketSize0);
    dev_ctx->max_packet_size = short_descriptor.bMaxPacketSize0;
    xprintf("Got MPS = %d\n", dev_ctx->max_packet_size);

    // getdesc_txn->ops->release(getdesc_txn);
    completion_reset(&completion);
    // Set the Device Address
    iotxn_t *setaddr_txn;
    status = iotxn_alloc(&setaddr_txn, 0, 64, 0);
    if (status != NO_ERROR) {
        xprintf("iotxn alloc failed for setaddr_txn\n");
        return status;
    }

    setaddr_txn->protocol = MX_PROTOCOL_USB;
    proto_data = iotxn_pdata(setaddr_txn, usb_protocol_data_t);
    proto_data->ep_address = 0;
    proto_data->frame = 0;
    proto_data->device_id = 0;
    setaddr_txn->complete_cb = usb_control_complete;
    setaddr_txn->cookie = &completion2;

    proto_data->setup.bmRequestType = 0x00;
    proto_data->setup.bRequest = 0x05;
    proto_data->setup.wValue = next_device_address;
    proto_data->setup.wIndex = 0;
    proto_data->setup.wLength = 0;
    setaddr_txn->length = 0;

    // memset(req, 0, sizeof(*req));
    struct dwc_transfer_request* req2;
    dwc_alloc_request(&req2);

    req2->txn = setaddr_txn;
    req2->txn_phase = PHASE_SETUP;
    req2->cb = dwc_control_setup_complete;
    req2->ep_ctx = NULL;
    req2->dev_ctx = dev_ctx;

    dwc_queue_transfer(req2);
    completion_wait(&completion2, MX_TIME_INFINITE);

    // setaddr_txn->ops->release(setaddr_txn);

    // We need to wait 50ms for this address to be valid.
    // mx_nanosleep(MX_MSEC(50));

    free(req);
    free(req2);

    // Set Address Completed. Store this device context.
    mtx_lock(&devctx_mtx);
    devctxs[next_device_address] = dev_ctx;
    mtx_unlock(&devctx_mtx);

    xprintf("dwc->bus_protocol->add_device(%p, %d, %d, %d)\n", dwc->bus_device, next_device_address, hub_address, speed);
    dwc->bus_protocol->add_device(dwc->bus_device, next_device_address, hub_address, speed);

    next_device_address++;

    return NO_ERROR;
}

mx_status_t dwc_hub_device_removed(mx_device_t* hci_device, uint32_t hub_address, int port) {
    xprintf("[UNIMPLEMENTED]: dwc_hub_device_removed\n");
    return NO_ERROR;
}

static mx_status_t dwc_rh_get_descriptor(uint8_t request_type, uint16_t value,
                                          uint16_t index, size_t length, iotxn_t* txn) {
    uint8_t type = request_type & USB_TYPE_MASK;
    uint8_t recipient = request_type & USB_RECIP_MASK;

    if (type == USB_TYPE_STANDARD && recipient == USB_RECIP_DEVICE) {
        uint8_t desc_type = value >> 8;
        if (desc_type == USB_DT_DEVICE && index == 0) {
            if (length > sizeof(usb_device_descriptor_t)) length = sizeof(usb_device_descriptor_t);
            txn->ops->copyto(txn, dwc_rh_descriptor, length, 0);
            txn->ops->complete(txn, NO_ERROR, length);
            return NO_ERROR;
        } else if (desc_type == USB_DT_CONFIG && index == 0) {
            usb_configuration_descriptor_t* config_descriptor = 
                    (usb_configuration_descriptor_t*)dwc_rh_config_desc;
            uint16_t desc_length = le16toh(config_descriptor->wTotalLength);
            if (length > desc_length) length = desc_length;
            txn->ops->copyto(txn, dwc_rh_config_desc, length, 0);
            txn->ops->complete(txn, NO_ERROR, length);
            return NO_ERROR;
        } else if (value >> 8 == USB_DT_STRING) {
            // uint8_t string_index = value & 0xFF;
        }
    }
    else if (type == USB_TYPE_CLASS && recipient == USB_RECIP_DEVICE) {
        if ((value == USB_HUB_DESC_TYPE_SS << 8 || value == USB_HUB_DESC_TYPE << 8) && index == 0) {

            // return hub descriptor
            usb_hub_descriptor_t desc;
            memset(&desc, 0, sizeof(desc));
            desc.bDescLength = sizeof(desc);
            desc.bDescriptorType = value >> 8;
            desc.bNbrPorts = 1;
            desc.bPowerOn2PwrGood = 50;
            // TODO - fill in other stuff. But usb-hub driver doesn't need anything else at this point.

            if (length > sizeof(desc)) length = sizeof(desc);
            txn->ops->copyto(txn, &desc, length, 0);
            txn->ops->complete(txn, NO_ERROR, length);
            return NO_ERROR;
        }
    }

    xprintf("dwc_rh_get_descriptor unsupported value: %d index: %d\n", value, index);
    txn->ops->complete(txn, ERR_NOT_SUPPORTED, 0);
    return ERR_NOT_SUPPORTED;
}

// Interrupt request to the root hub.
static mx_status_t dwc_usb_rh_intr(usb_protocol_data_t* data, iotxn_t* txn) {
    // If the hub's status has changed since the last time we were querried, we
    // complete the transaction, otherwise we wait until the hub's status has 
    // changed.

    assert(data && txn);
    list_add_tail(&pending_intr_reqs, &txn->node);
    return dwc_complete_root_port_status_txn();
}


static mx_status_t dwc_usb_rh_control(usb_protocol_data_t* data, iotxn_t* txn) {
    uint8_t request_type = data->setup.bmRequestType;
    uint8_t request = data->setup.bRequest;
    uint16_t value = le16toh(data->setup.wValue);
    uint16_t index = le16toh(data->setup.wIndex);

    // xprintf("dwc_usb_rh_control: 0x%02X req: %d value: %d index: %d length: %d\n",
    //         request_type, request, value, index, le16toh(data->setup.wLength));

    // TODO(gkalsi): Fix this.
    if (request == USB_DEVICE_REQUEST_SET_ADDRESS) {
        txn->ops->complete(txn, NO_ERROR, 0);
        return NO_ERROR;
    }

    if ((request_type & USB_DIR_MASK) == USB_DIR_IN && request == USB_REQ_GET_DESCRIPTOR) {
        return dwc_rh_get_descriptor(request_type, value, index,
                                     le16toh(data->setup.wLength), txn);
    } else if ((request_type & ~USB_DIR_MASK) == (USB_TYPE_CLASS | USB_RECIP_PORT)) {
        // index is 1-based port number
        if (request == USB_REQ_SET_FEATURE) {
            if (value == USB_FEATURE_PORT_POWER) {
                dwc_power_on_host_port();
                txn->ops->complete(txn, NO_ERROR, 0);
                return NO_ERROR;
            } else if (value == USB_FEATURE_PORT_RESET) {
                dwc_reset_host_port();
                txn->ops->complete(txn, NO_ERROR, 0);
                return NO_ERROR;
            }
        } else if (request == USB_REQ_CLEAR_FEATURE) {
            mtx_lock(&root_port_status_mtx);
            switch (value) {
                case USB_FEATURE_C_PORT_CONNECTION:
                    root_port_status.wPortChange &= ~USB_PORT_CONNECTION;
                    break;
                case USB_FEATURE_C_PORT_ENABLE:
                    root_port_status.wPortChange &= ~USB_PORT_ENABLE;
                    break;
                case USB_FEATURE_C_PORT_SUSPEND:
                    root_port_status.wPortChange &= ~USB_PORT_SUSPEND;
                    break;
                case USB_FEATURE_C_PORT_OVER_CURRENT:
                    root_port_status.wPortChange &= ~USB_PORT_OVER_CURRENT;
                    break;
                case USB_FEATURE_C_PORT_RESET:
                    root_port_status.wPortChange &= ~USB_PORT_RESET;
                    break;
            }
            mtx_unlock(&root_port_status_mtx);

            txn->ops->complete(txn, NO_ERROR, 0);
            return NO_ERROR;
        } else if ((request_type & USB_DIR_MASK) == USB_DIR_IN &&
                   request == USB_REQ_GET_STATUS && value == 0) {
            mtx_lock(&root_port_status_mtx);
            txn->ops->copyto(txn, (void*)&root_port_status, sizeof(root_port_status), 0);
            txn->ops->complete(txn, NO_ERROR, sizeof(root_port_status));
            mtx_unlock(&root_port_status_mtx);
            return NO_ERROR;
        } else {
            xprintf("UNKNOWN USB PORT REQUEST");
        }
    } else if (request_type == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE) &&
               request == USB_REQ_SET_CONFIGURATION && txn->length == 0) {
        // nothing to do here
        txn->ops->complete(txn, NO_ERROR, 0);
        return NO_ERROR;
    }

    // TODO(gkalsi): do we need to txn->ops->complete(...) here?
    xprintf("UNKNOWN USB REQUEST");
    return ERR_NOT_SUPPORTED;

}

uint8_t dwc_endpoint_index(uint8_t ep_address) {
    if (ep_address == 0) return 0;
    uint32_t index = 2 * (ep_address & ~USB_ENDPOINT_DIR_MASK);
    if ((ep_address & USB_ENDPOINT_DIR_MASK) == USB_ENDPOINT_OUT)
        index--;
    return index;
}

mx_status_t dwc_rh_iotxn_queue(iotxn_t* txn) {
    usb_protocol_data_t* data = iotxn_pdata(txn, usb_protocol_data_t);
    uint8_t ep_index = dwc_endpoint_index(data->ep_address);

    if (ep_index == 0) {
        // Root hub transaction going to the control endpoint.
        return dwc_usb_rh_control(data, txn);
    } else if (ep_index == 2) {
        // Root hub transaction going to the interrupt endpoint.
        return dwc_usb_rh_intr(data, txn);
    }

    txn->ops->complete(txn, ERR_NOT_SUPPORTED, 0);
    return ERR_NOT_SUPPORTED;
}

// static void dwc_iotxn_callback(mx_status_t result, void* cookie) {
//     iotxn_t* txn = (iotxn_t *)cookie;
//     mx_status_t status;
//     size_t actual;

//     if (result > 0) {
//         actual = result;
//         status = NO_ERROR;
//     } else {
//         actual = 0;
//         status = result;
//     }
//     free(txn->context);
//     txn->context = NULL;

//     txn->ops->complete(txn, status, actual);
// }

static mx_status_t dwc_do_iotxn_queue(iotxn_t* txn) {
    usb_protocol_data_t* data = iotxn_pdata(txn, usb_protocol_data_t);
    //xprintf("dwc_do_iotxn_queue, txn = %p, pdata = %p\n", txn, data);

    if (dwc_is_root_hub(data->device_id)) {
        return dwc_rh_iotxn_queue(txn);
    }

    struct dwc_transfer_request* req;
    dwc_alloc_request(&req);  // TODO(gkalsi): Check return value;
    if (!req) {
        xprintf("Failed to allocate request!\n");
    }

    endpoint_context_t* ep_ctx = NULL;
    endpoint_context_t* target_ctx = NULL;

    // This only applies to devices that have been enumerated. This does not 
    // apply for control transfers.
    if (data->device_id && data->ep_address) {
        list_for_every_entry (&device_eps[data->device_id], ep_ctx, endpoint_context_t, node) {
            if (ep_ctx->descriptor.bEndpointAddress == data->ep_address) {
                target_ctx = ep_ctx;
                break;
            }
        }

        if (!target_ctx) {
            xprintf("Looking for ep_address = %u on device = %u\n", data->ep_address, data->device_id);
            xprintf("Requested endpoint does not exist on this device!\n");
            return ERR_INTERNAL;
        }
    } else {
        req->cb = dwc_control_setup_complete;
        req->txn_phase = PHASE_SETUP;
    }

    if (txn == NULL) {
        xprintf("Got a NULL iotxn in dwc_do_iotxn_queue\n");
    }

    req->txn = txn;
    req->ep_ctx = target_ctx;
    mtx_lock(&devctx_mtx);
    req->dev_ctx = devctxs[data->device_id];
    mtx_unlock(&devctx_mtx);

    if (!req->dev_ctx) {
        xprintf("DEVICE CONTEXT WAS NULL!\n");
    }

    return dwc_queue_transfer(req);
}

static void dwc_iotxn_queue(mx_device_t* hci_device, iotxn_t* txn) {
    if (!hci_device || !txn) {
        xprintf("hci_device = %p, txn = %p. Aborting.\n", hci_device, txn);
        return;
    }

    mx_status_t status = dwc_do_iotxn_queue(txn);
    if (status != NO_ERROR && status != ERR_BUFFER_TOO_SMALL) {
        xprintf("Completing transaction with failure = %d\n", status);
        txn->ops->complete(txn, status, 0);
    }
}

static void dwc_unbind(mx_device_t* dev) {
    xprintf("[UNIMPLEMENTED]: dwc_unbind\n");
}

static mx_status_t dwc_release(mx_device_t* device) {
    xprintf("[UNIMPLEMENTED]: dwc_release\n");
    return NO_ERROR;
}

static void dwc_init_host_channel(volatile struct dwc_host_channel *hc)
{
    union dwc_host_channel_characteristics characteristics;
    union dwc_host_channel_interrupts interrupts;

    interrupts.val = 0xFFFFFFFF;

    characteristics.val = 0;

    characteristics.channel_disable = 1;
    characteristics.channel_enable  = 1;
    characteristics.endpoint_direction = 0;

    hc->characteristics = characteristics;
    hc->interrupts = interrupts;

    while (hc->characteristics.channel_enable) {
        mx_nanosleep(MX_MSEC(50));
    }
}

static mx_protocol_device_t dwc_device_proto = {
    .iotxn_queue = dwc_iotxn_queue,
    .unbind = dwc_unbind,
    .release = dwc_release,
};

static usb_hci_protocol_t dwc_hci_protocol = {
    .set_bus_device = dwc_set_bus_device,
    .get_max_device_count = dwc_get_max_device_count,
    .enable_endpoint = dwc_enable_ep,
    .get_current_frame = dwc_get_frame,
    .configure_hub = dwc_config_hub,
    .hub_device_added = dwc_hub_device_added,
    .hub_device_removed = dwc_hub_device_removed,
};

static mx_status_t usb_dwc_bind(mx_driver_t* drv, mx_device_t* dev) {
    mx_handle_t irq_handle = MX_HANDLE_INVALID; 
    usb_dwc_t* dwc = NULL;
    mx_status_t st;

    dwc = calloc(1, sizeof(*dwc));
    if (!dwc) {
        st = ERR_NO_MEMORY;
        goto error_return;
    }

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