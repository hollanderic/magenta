#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/binding.h>
#include <ddk/protocol/bcm.h>
#include <ddk/protocol/display.h>

#include <magenta/threads.h>
#include <magenta/syscalls.h>
#include <magenta/syscalls/port.h>

#include <magenta/listnode.h>

#include <bcm/bcm28xx.h>
#include <bcm/dma.h>

/*
Notes
    -Need a threadpool for handling things?
    -Need to keep track of channel allocations

*/
static bcm_dma_ctrl_regs_t* dma_regs = NULL;
static list_node_t clients;

static mx_handle_t port = MX_HANDLE_INVALID;

typedef struct {
    mx_handle_t     mx_ch;
    bcm_dma_t       props;

    list_node_t     node;
    // statistics keeping
    size_t      num_transfers;
    size_t      num_bytes;
    mx_time_t   last_active;

} bcm_dma_client_t;


void dma_watch_the_world_burn(mx_status_t status, bcm_dma_client_t* client) {
    printf("shit shit shit %x   %p\n",status,client);
}

static ssize_t bcm_dma_ioctl(mx_device_t* dev, uint32_t op,
                                    const void* in_buf, size_t in_len,
                                    void* out_buf, size_t out_len) {
    mx_status_t status;
    mx_handle_t* reply = out_buf;

    if (op != BCM_DMA_IOCTL_GET_CHANNEL) return ERR_INVALID_ARGS;

    bcm_dma_client_t* client = calloc(1, sizeof(bcm_dma_client_t));
    if (!client) {
        status = ERR_NO_MEMORY;
        goto ioctl_fail;
    }

    mx_handle_t ret_channel;
    status = mx_channel_create(0, &client->mx_ch, &ret_channel);
    if (status != NO_ERROR) goto ioctl_fail;
    *reply = ret_channel;

    status = mx_port_bind(port, (uint64_t)client, client->mx_ch, MX_CHANNEL_READABLE | MX_CHANNEL_PEER_CLOSED);
    if (status != NO_ERROR) goto ioctl_fail;

    list_add_head(&clients, &client->node);

    return NO_ERROR;

ioctl_fail:
    if (client) {
        mx_handle_close(client->mx_ch);
        mx_handle_close(ret_channel);
        free(client);
    }
    return status;
}

mx_status_t bcm_dma_transfer(bcm_dma_transfer_req_t req,
                                                mx_handle_t* handles,
                                                uint32_t num_handles) {

    printf("Got handles:\n");
    for (uint i = 0; i < num_handles; i++)
        printf("   %x\n",handles[i]);

    return NO_ERROR;
}


mx_status_t bcm_dma_debug(bcm_dma_cmd_hdr_t req, mx_handle_t* handles,
                                                 uint32_t num_handles) {
    bcm_dma_client_t* client = NULL;
     mx_time_t now = mx_time_get(MX_CLOCK_MONOTONIC);

    switch(req.cmd) {
        case BCM_DMA_DEBUG_SHOW_CLIENTS:

            list_for_every_entry(&clients, client, bcm_dma_client_t, node) {
                printf("Client %p:\n",client);
                printf("   last active %lu ns ago\n", now - client->last_active);
            }
            break;
        default:
            break;
    }

    return NO_ERROR;
}

#define HANDLE_REQ(_ioctl, _payload, _handler)      \
case _ioctl:                                        \
    if (req_size != sizeof(req._payload)) {         \
        printf("Bad " #_payload                     \
                  " reqonse length (%u != %zu)\n",  \
                  req_size, sizeof(req._payload));  \
        return ERR_INVALID_ARGS;                    \
    }                                               \
    _handler(req._payload,handles,num_handles);                    \
    break;
static int bcm_dma_thread(void *arg) {
    mx_status_t status;
    mx_io_packet_t port_out;

    dma_packet_t req;

    while (true) {
        status = mx_port_wait(port, MX_TIME_INFINITE, &port_out, sizeof(port_out));
        if (status != 0) { printf("I should probably do something about this\n");}

        bcm_dma_client_t* client = (bcm_dma_client_t*)port_out.hdr.key;

        uint32_t req_size;

        if (port_out.signals == MX_CHANNEL_READABLE) {
            mx_handle_t handles[4];
            uint32_t    num_handles;
            status = mx_channel_read(client->mx_ch,0,&req,sizeof(req),&req_size,handles,4,&num_handles);
            if (status != NO_ERROR) dma_watch_the_world_burn(status,client);
            switch(req.hdr.cmd) {
                HANDLE_REQ(BCM_DMA_SEND_SHIT, transfer_req, bcm_dma_transfer);
                HANDLE_REQ(BCM_DMA_DEBUG_SHOW_CLIENTS, hdr, bcm_dma_debug);
            }

        } else if (port_out.signals == MX_CHANNEL_PEER_CLOSED) {
            printf("Closing channel %x\n",client->mx_ch);
            mx_handle_close(client->mx_ch);
            printf("Removing client\n");
            list_delete(&client->node);
            free(client);
        }
        client->last_active = mx_time_get(MX_CLOCK_MONOTONIC);
    }



    return -1;
}

mx_driver_t _driver_bcm_dma = {
    .name = "bcm-dma",
};

static mx_protocol_device_t bcm_dma_proto = {
    .ioctl = bcm_dma_ioctl,
};


mx_status_t bcm_dma_init(mx_device_t* parent) {
    mx_status_t status;

    list_initialize(&clients);

    if (dma_regs == NULL) {
        status = mx_mmap_device_memory(
            get_root_resource(),
            DMA_BASE, 0x1000,
            MX_CACHE_POLICY_UNCACHED_DEVICE, (uintptr_t*)&dma_regs);
        if (status != NO_ERROR)
            goto dma_cleanup;
    }

    status = mx_port_create(0,&port);
    if (status != NO_ERROR) goto dma_cleanup;


    thrd_t port_thrd;
    int thrd_rc = thrd_create_with_name(&port_thrd,
                                        bcm_dma_thread, NULL,
                                        "bcm_dma_thread");
    if (thrd_rc != thrd_success) {
        status = thrd_status_to_mx_status(thrd_rc);
        goto dma_cleanup;
    }
    thrd_detach(port_thrd);


    mx_device_t* dma_device;      //= malloc(sizeof(*dma_device));
    status = device_create(&dma_device, &_driver_bcm_dma, "bcm-dma", &bcm_dma_proto);
    if (status != NO_ERROR) goto dma_cleanup;


    status = device_add(dma_device, parent);
    if (status != NO_ERROR) goto dma_cleanup;


    return status;

dma_cleanup:
    if (dma_device) free(dma_device);
    if (port != MX_HANDLE_INVALID) mx_handle_close(port);
    return status;
}



