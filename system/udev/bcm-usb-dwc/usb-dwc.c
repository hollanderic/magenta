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

static volatile struct dwc_regs* regs;
static volatile usb_port_status_t root_port_status;
static volatile bool root_port_status_changed = false;
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
    return req->ep_ctx == NULL;
}

void dwc_save_request(struct dwc_transfer_request* req, unsigned int chan) {
    stashed_requests[chan] = req;
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

static void dwc_start_root_hub(void) {
    union dwc_host_port_ctrlstatus hw_status = regs->host_port_ctrlstatus;

    hw_status.enabled = 0;
    hw_status.connected_changed = 0;
    hw_status.enabled_changed = 0;
    hw_status.overcurrent_changed = 0;
    hw_status.powered = 1;

    regs->host_port_ctrlstatus = hw_status;

    // Let the host port power up.
    usleep(25000);
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
    txn->ops->copyto(txn, (void*)&val, sizeof(root_port_status), 0);
    txn->ops->complete(txn, NO_ERROR, sizeof(root_port_status));
    return NO_ERROR;
}

static enum dwc_intr_status dwc_handle_channel_halted(struct dwc_transfer_request* req, unsigned int channel) {
    volatile struct dwc_host_channel* chanptr = &regs->host_channels[channel];
    union dwc_host_channel_interrupts interrupts = chanptr->interrupts;

    printf("dwc_handle_channel_halted\n");

    uint packets_remaining = chanptr->transfer.packet_count;
    uint packets_transferred = req->packets_remaining - packets_remaining;

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

        printf("Bytes transferred = %lu\n", bytes_transferred);

        req->packets_remaining -= packets_transferred;
        req->bytes_remaining -= bytes_transferred;
        req->data_index += bytes_transferred;

        if (req->packets_remaining == 0 || 
            (dir == USB_DIRECTION_IN &&
             bytes_transferred < packets_transferred * max_packet_size)) {
            if (!interrupts.transfer_completed) {
                printf("TRANSFER FAILED!");
                return XFER_FAILED;
            }

            if (req->more_data && req->bytes_remaining == 0 &&
                type != USB_TRANSFER_TYPE_INTERRUPT) {
                req->complete_split = false;
                req->next_data_pid = chanptr->transfer.packet_id;
                printf("Restart trasnfer with pid = %u\n", req->next_data_pid);
                if (!usb_is_control_request(req) || req->txn_phase == PHASE_DATA) {
                    req->actual_size = req->data_index;
                }
                return XFER_NEEDS_RESTART;
            }

            // TODO(gkalsi): Put this in the callback.
            if (usb_is_control_request(req) && req->txn_phase < PHASE_STATUS) {
                req->complete_split = false;
                return XFER_COMPLETE;
            }

            if (usb_is_control_request(req) && req->txn_phase == PHASE_SETUP) {
                req->data_index = 0;
            }

            return XFER_COMPLETE;
        } else {
            if (chanptr->split_control.split_enable) {
                req->complete_split = !req->complete_split;
                printf("Inverting Complete Split. Complete split is now %d\n", req->complete_split);
            }
            return XFER_NEEDS_TRANS_RESTART;
        }
    } else {
        if (interrupts.ack_response_received &&
            chanptr->split_control.split_enable && !req->complete_split) {
            req->complete_split = true;
            printf("Enabling Complete Split. Complete split is now %d\n", req->complete_split);
            return XFER_NEEDS_TRANS_RESTART;
        } else {
            printf("Xfer Failed:\n");
            printf("  interrupts.ack_response_received: %d\n", interrupts.ack_response_received);
            printf("  chanptr->split_control.split_enable: %d\n", chanptr->split_control.split_enable);
            printf("  req->complete_split: %d\n", req->complete_split);
            return XFER_FAILED;
        }
    }


}

bool dwc_handle_host_channel_interrupt(unsigned int channel) {
    volatile struct dwc_host_channel* chanptr = &regs->host_channels[channel];
    union dwc_host_channel_interrupts interrupts = chanptr->interrupts;

    printf("Channel interrupt = 0x%x\n", chanptr->interrupts.val);
    enum dwc_intr_status intr_status;

    struct dwc_transfer_request* req = stashed_requests[channel];
    // iotxn_t* txn = request->txn;
    // usb_protocol_data_t* data = iotxn_pdata(txn, usb_protocol_data_t);
    // usb_setup_t* setup = (data->ep_address == 0 ? &data->setup : NULL);

    if (interrupts.stall_response_received || interrupts.ahb_error ||
        interrupts.transaction_error || interrupts.babble_error ||
        interrupts.excess_transaction_error || interrupts.frame_list_rollover ||
        (interrupts.nyet_response_received) ||
        (interrupts.data_toggle_error &&
         chanptr->characteristics.endpoint_direction == USB_DIRECTION_OUT))
    {
        printf("Transfer failed with status = 0x%x\n", interrupts.val);
        return true;   
    } else if (interrupts.frame_overrun)
    {
        printf("Transfer failed with frame_overrun\n");
        return true;
    }
    else if (interrupts.nyet_response_received)
    {
        printf("Transfer failed with nyet\n");
        return true;
    }
    else if (interrupts.nak_response_received)
    {
        printf("Transfer failed with nak\n");
        return true;
    } 
    else {
        intr_status = dwc_handle_channel_halted(req, channel);
        printf("Channel Halted with intr_status = %d\n", intr_status);
    }



    if (intr_status == XFER_COMPLETE) {
        union dwc_host_channel_characteristics characteristics;
        characteristics.val = 0;
        chanptr->characteristics = characteristics;
        chanptr->interrupt_mask.val = 0;
        chanptr->interrupts.val = 0xffffffff;

        req->txn->ops->complete(req->txn, NO_ERROR, req->txn->length);
        return true;
    } else if (intr_status == XFER_NEEDS_TRANS_RESTART) {
        printf("Restarting transaction with CSplit\n");
        dwc_channel_start_transaction(req, channel);
        return false;
    } else if (intr_status == XFER_NEEDS_RESTART) {
        union dwc_host_channel_characteristics characteristics;
        characteristics.val = 0;
        chanptr->characteristics = characteristics;
        chanptr->interrupt_mask.val = 0;
        chanptr->interrupts.val = 0xffffffff;

        if (req->cb == dwc_control_data_complete) {
            req->cb = dwc_control_setup_complete;
        }

        printf("TRANSFER NEEDS RESTART!\n");

        return true;
    }

    return true;
}

static void reset_request(struct dwc_transfer_request* req) {
    req->more_data = 0;
    req->complete_split = 0;
    req->data_index = 0;
    req->actual_size = 0;
    req->attempted_size = 0;
    req->bytes_remaining = 0;
    req->packets_remaining = 0;
    req->next_data_pid = 0;
}

static void dwc_control_status_complete(mx_status_t result, void* data) {
    printf("STATUS transaction completed\n");
}

static void dwc_control_data_complete(mx_status_t result, void* data) {
    printf("DATA txn complete, starting STATUS txn\n");
    struct dwc_transfer_request* req = (struct dwc_transfer_request*)data;
    req->txn_phase = PHASE_STATUS;
    req->cb = dwc_control_status_complete;

    dwc_queue_transfer(req);
}

static void dwc_control_setup_complete(mx_status_t result, void* data) {
    printf("SETUP txn complete. ");
    struct dwc_transfer_request* req = (struct dwc_transfer_request*)data;
    iotxn_t *txn = req->txn;

    if (txn->length) {
        printf("Starting DATA txn.\n");
        req->txn_phase = PHASE_DATA;
        req->cb = dwc_control_data_complete;
    } else {
        printf("Starting STATUS txn.\n");
        req->txn_phase = PHASE_STATUS;
        req->cb = dwc_control_status_complete;
    }
    dwc_queue_transfer(req);
}

static inline ulong first_set_bit(ulong word)
{
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

        // Clear the interrupt.
        hw_status.enabled = 0;
        regs->host_port_ctrlstatus = hw_status;

        dwc_complete_root_port_status_txn();

        core_interrupt_ack.port_intr = 1;
    }

    if (interrupts.host_channel_intr) {
        uint32_t chintr = regs->host_channels_interrupt;
        // printf("Channel interrupts = 0x%x\n", chintr);
        do {
            unsigned int chan = first_set_bit(chintr);
            if (dwc_handle_host_channel_interrupt(chan))
                result |= (1 << chan);

            release_channel(chan);
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
            printf("dwc_irq_thread::mx_handle_wait_one(irq_handle) returned "
                   "error code = %d\n", wait_res);

        uint32_t completed_channels = dwc_handle_interrupt();

        mx_interrupt_complete(dwc->irq_handle);

        for (int i = 0; i < NUM_HOST_CHANNELS; i++) {
            if (completed_channels & (1 << i)) {
                do_channel_callback(i);
            }
        }

    }

    printf("dwc_irq_thread done.\n");
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
        dwc_start_root_hub();
        dwc_reset_host_port();
        dwc_add_root_hub(dwc);
    } else {
        dwc->bus_protocol = NULL;
    }
}

static size_t dwc_get_max_device_count(mx_device_t* device) {
    printf("[UNIMPLEMENTED]: dwc_get_max_device_count\n");
    return MAX_DEVICE_COUNT + ROOT_HUB_COUNT + 1;
}

static mx_status_t dwc_enable_ep(mx_device_t* hci_device, uint32_t device_id,
                                  usb_endpoint_descriptor_t* ep_desc, bool enable) {
    printf("[UNIMPLEMENTED]: dwc_enable_ep\n");

    printf("Enabling Endpoint on Device = %u\n", device_id);
    printf("  bLength = %u\n", ep_desc->bLength);
    printf("  bDescriptorType = %u\n", ep_desc->bDescriptorType);
    printf("  bEndpointAddress = %u\n", ep_desc->bEndpointAddress);
    printf("  bmAttributes = %u\n", ep_desc->bmAttributes);
    printf("  wMaxPacketSize = %u\n", ep_desc->wMaxPacketSize);
    printf("  bInterval = %u\n", ep_desc->bInterval);

    // Do nothing for root hubs.
    if (dwc_is_root_hub(device_id)) {
        return NO_ERROR;
    }

    if (enable) {
        // Create a new endpoint context.
        endpoint_context_t* ep_ctx = calloc(1, sizeof(*ep_ctx));
        memcpy(&ep_ctx->descriptor, ep_desc, sizeof(*ep_desc));
        list_add_tail(&device_eps[device_id], &ep_ctx->node);
    } else {
        // TODO(gkalsi);
        // Remove the endpoint from the list of endpoints from this device and
        // free the memory that it has allocated.
    }


    return NO_ERROR;
}

static uint64_t dwc_get_frame(mx_device_t* hci_device) {
    printf("[UNIMPLEMENTED]: dwc_get_frame\n");
    return NO_ERROR;
}

mx_status_t dwc_config_hub(mx_device_t* hci_device, uint32_t device_id, usb_speed_t speed,
                            usb_hub_descriptor_t* descriptor) {
    printf("[UNIMPLEMENTED]: dwc_config_hub\n");
    return NO_ERROR;
}

static void usb_control_complete(iotxn_t* txn, void* cookie) {
    completion_signal((completion_t*)cookie);
}

#define SWAP_16(x) \
    ((((uint16_t)(x) & 0xff) << 8) | ((uint16_t)(x) >> 8))

mx_status_t dwc_hub_device_added(mx_device_t* hci_device, uint32_t hub_address, int port,
                                  usb_speed_t speed) {
    printf("dwc_hub_device_added: hub_address = %u, port = %d, speed = %d\n", hub_address, port, speed);

    usb_dwc_t* dwc = dev_to_usb_dwc(hci_device);

    // TODO(gkalsi): Maybe do all this in a thread?
    iotxn_t *getdesc_txn;

    mx_status_t status = iotxn_alloc(&getdesc_txn, 0, 64, 0);
    if (status != NO_ERROR) {
        printf("iotxn_alloc failed\n");
        return status;
    }

    getdesc_txn->protocol = MX_PROTOCOL_USB;
    usb_protocol_data_t* proto_data = iotxn_pdata(getdesc_txn, usb_protocol_data_t);
    completion_t completion = COMPLETION_INIT;
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

    if (speed == USB_SPEED_LOW) {
        proto_data->setup.wLength = 8;            // Minimum possible packet size.
        getdesc_txn->length = 8;
    } else {
        proto_data->setup.wLength = 64;            // Minimum possible packet size.
        getdesc_txn->length = 64;
    }

    device_context_t* dev_ctx = calloc(1, sizeof(*dev_ctx));
    dev_ctx->parent_port = port;
    dev_ctx->parent_hub = hub_address;
    dev_ctx->speed = speed;

    if (speed == USB_SPEED_LOW) {
        dev_ctx->max_packet_size = 8;   // Until we can determine what the MPS really is.
    } else {
        dev_ctx->max_packet_size = 64;   // Until we can determine what the MPS really is.
    }
    
    struct dwc_transfer_request* req = calloc(1, sizeof(*req));
    req->txn = getdesc_txn;
    req->txn_phase = PHASE_SETUP;
    req->cb = dwc_control_setup_complete;
    req->ep_ctx = NULL;
    req->dev_ctx = dev_ctx;

    dwc_queue_transfer(req);
    completion_wait(&completion, MX_TIME_INFINITE);

    usb_device_descriptor_t short_descriptor;
    getdesc_txn->ops->copyfrom(getdesc_txn, &short_descriptor, getdesc_txn->actual, 0);

    printf("Got 8 byte Device Descriptor. Device MPS is %u\n", short_descriptor.bMaxPacketSize0);
    dev_ctx->max_packet_size = short_descriptor.bMaxPacketSize0;

    completion_reset(&completion);
    // Set the Device Address
    iotxn_t *setaddr_txn;
    status = iotxn_alloc(&setaddr_txn, 0, 64, 0);
    if (status != NO_ERROR) {
        printf("iotxn alloc failed for setaddr_txn\n");
        return status;
    }

    setaddr_txn->protocol = MX_PROTOCOL_USB;
    proto_data = iotxn_pdata(setaddr_txn, usb_protocol_data_t);
    proto_data->ep_address = 0;
    proto_data->frame = 0;
    proto_data->device_id = 0;
    setaddr_txn->complete_cb = usb_control_complete;
    setaddr_txn->cookie = &completion;

    proto_data->setup.bmRequestType = 0x00;
    proto_data->setup.bRequest = 0x05;
    proto_data->setup.wValue = next_device_address;
    proto_data->setup.wIndex = 0;
    proto_data->setup.wLength = 0;
    setaddr_txn->length = 0;

    memset(req, 0, sizeof(*req));

    req->txn = setaddr_txn;
    req->txn_phase = PHASE_SETUP;
    req->cb = dwc_control_setup_complete;
    req->ep_ctx = NULL;
    req->dev_ctx = dev_ctx;

    dwc_queue_transfer(req);
    completion_wait(&completion, MX_TIME_INFINITE);

    printf("Freeing the reqeust\n");

    free(req);

    // Set Address Completed. Store this device context.
    devctxs[next_device_address] = dev_ctx;

    printf("dwc->bus_protocol->add_device(dwc->bus_device, %d, %d, %d)\n", next_device_address, hub_address, speed);
    dwc->bus_protocol->add_device(dwc->bus_device, next_device_address, hub_address, speed);

    next_device_address++;

    return NO_ERROR;
}

mx_status_t dwc_hub_device_removed(mx_device_t* hci_device, uint32_t hub_address, int port) {
    printf("[UNIMPLEMENTED]: dwc_hub_device_removed\n");
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

    printf("dwc_rh_get_descriptor unsupported value: %d index: %d\n", value, index);
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

    printf("dwc_usb_rh_control: 0x%02X req: %d value: %d index: %d length: %d\n",
            request_type, request, value, index, le16toh(data->setup.wLength));

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
                txn->ops->complete(txn, NO_ERROR, 0);
                return NO_ERROR;
            } else if (value == USB_FEATURE_PORT_RESET) {
                txn->ops->complete(txn, NO_ERROR, 0);
                return NO_ERROR;
            }
        } else if (request == USB_REQ_CLEAR_FEATURE) {
            // TODO(gkalsi): This clears all the features.
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

            txn->ops->complete(txn, NO_ERROR, 0);
            return NO_ERROR;
        } else if ((request_type & USB_DIR_MASK) == USB_DIR_IN &&
                   request == USB_REQ_GET_STATUS && value == 0) {
            txn->ops->copyto(txn, (void*)&root_port_status, sizeof(root_port_status), 0);
            txn->ops->complete(txn, NO_ERROR, sizeof(root_port_status));
            return NO_ERROR;
        } else {
            printf("UNKNOWN USB PORT REQUEST");
        }
    } else if (request_type == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE) &&
               request == USB_REQ_SET_CONFIGURATION && txn->length == 0) {
        // nothing to do here
        txn->ops->complete(txn, NO_ERROR, 0);
        return NO_ERROR;
    }

    // TODO(gkalsi): do we need to txn->ops->complete(...) here?
    printf("UNKNOWN USB REQUEST");
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

static void dwc_iotxn_callback(mx_status_t result, void* cookie) {
    iotxn_t* txn = (iotxn_t *)cookie;
    mx_status_t status;
    size_t actual;

    if (result > 0) {
        actual = result;
        status = NO_ERROR;
    } else {
        actual = 0;
        status = result;
    }
    free(txn->context);
    txn->context = NULL;

    txn->ops->complete(txn, status, actual);
}

static mx_status_t dwc_do_iotxn_queue(iotxn_t* txn) {
    usb_protocol_data_t* data = iotxn_pdata(txn, usb_protocol_data_t);

    if (dwc_is_root_hub(data->device_id)) {
        return dwc_rh_iotxn_queue(txn);
    }

    struct dwc_transfer_request* req = calloc(1, sizeof(*req));

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
            printf("Looking for ep_address = %u on device = %u\n", data->ep_address, data->device_id);
            printf("Requested endpoint does not exist on this device!\n");
            return ERR_INTERNAL;
        }
    } else {
        req->cb = dwc_control_setup_complete;
        req->txn_phase = PHASE_SETUP;
    }

    req->txn = txn;
    req->ep_ctx = target_ctx;
    req->dev_ctx = devctxs[data->device_id];

    return dwc_queue_transfer(req);
}

static void dwc_iotxn_queue(mx_device_t* hci_device, iotxn_t* txn) {
    mx_status_t status = dwc_do_iotxn_queue(txn);
    if (status != NO_ERROR && status != ERR_BUFFER_TOO_SMALL) {
        printf("Completing transaction with failure = %d\n", status);
        txn->ops->complete(txn, status, 0);
    }
}

static void dwc_unbind(mx_device_t* dev) {
    printf("[UNIMPLEMENTED]: dwc_unbind\n");
}

static mx_status_t dwc_release(mx_device_t* device) {
    printf("[UNIMPLEMENTED]: dwc_release\n");
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
