// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once
#include <magenta/types.h>
#include <ddk/iotxn.h>
#include <magenta/hw/usb.h>
#include <threads.h>
#include <ddk/completion.h>

struct dwc_regs;
struct dwc_transfer_request;

typedef void (*dwc_transfer_complete_cb)(mx_status_t result, void* data);

typedef enum {
    PHASE_SETUP  = 1,
    PHASE_DATA   = 2,
    PHASE_STATUS = 3,
    PHASE_COUNT  = 4,  // Always last.
} usb_control_txn_phase_t;


// TODO(gkalsi): Change these to have a DWC specific name.
typedef struct endpoint_context {
    list_node_t node;
    usb_endpoint_descriptor_t descriptor;

    // The following fields pertain to handling deferred transactions on 
    // interrupt endpoints.
    thrd_t intr_ep_worker;
    completion_t intr_ep_completion;
    struct dwc_transfer_request* pending_request;
} endpoint_context_t;

typedef struct device_context {
    int parent_port;
    uint32_t parent_hub;
    uint32_t max_packet_size;
    usb_speed_t speed;
} device_context_t;

struct dwc_transfer_request {
    iotxn_t* txn;
    int mps;
    usb_control_txn_phase_t txn_phase;
    dwc_transfer_complete_cb cb;
    endpoint_context_t* ep_ctx;
    list_node_t node;
    bool more_data;
    bool complete_split;
    device_context_t* dev_ctx;

    size_t data_index;
    size_t actual_size;
    size_t attempted_size;
    size_t bytes_remaining;
    size_t packets_remaining;
    uint32_t next_data_pid;
};

void dwc_channel_start_transaction(struct dwc_transfer_request* req, uint32_t channel);
mx_status_t dwc_queue_transfer(struct dwc_transfer_request* req);
mx_status_t dwc_start_scheduler_thread(struct dwc_regs* regs);
void do_channel_callback(unsigned int channel);
void release_channel(unsigned int ch);
void dwc_save_request(struct dwc_transfer_request* req, unsigned int chan);
struct dwc_transfer_request* dwc_get_request(unsigned int chan);