// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
/*
NOTES
        -Ioctl to get vmo representing sample buffer
            to get info about current config
            to set current config
            to stop data
            to set volume
            to mute





*/
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/binding.h>
#include <ddk/protocol/bcm.h>

#include <magenta/syscalls.h>
#include <magenta/syscalls/port.h>
#include <magenta/threads.h>
#include <magenta/device/i2c.h>
#include <magenta/device/audio2.h>
#include <magenta/compiler.h>

#include <bcm/dma.h>
#include <bcm/gpio.h>
#include <bcm/bcm28xx.h>

#include "pcm.h"
#include "codec/pcm5122.h"

#define BCM_FIFO_DEADLINE_MS 100

//ideally we would get this from system, just putting it here now for reference
#define BCM_PCM_PAGE_SIZE   4096

#define DMA_CHAN 11
#define PCM_TRACE 0

typedef union {
    audio2_cmd_hdr_t                        hdr;
    audio2_stream_cmd_set_format_req_t      set_fmt_req;
} stream_packet_t;

typedef union {
    audio2_cmd_hdr_t                        hdr;
    audio2_rb_cmd_set_buffer_req_t          set_buffer_req;
    audio2_rb_cmd_start_req_t               start_req;
    audio2_rb_cmd_stop_req_t                stop_req;
} buffer_packet_t;


typedef struct {

    mx_device_t             device;
    mx_device_t*            parent;
    mx_driver_t*            driver;
    bcm_pcm_regs_t*         control_regs;
    bcm_gpio_ctrl_t*        gpio_regs;
    volatile uint32_t*      clock_regs;

    bcm_dma_t               dma;

    mx_handle_t             stream_ch;
    mx_handle_t             buffer_ch;
    mx_handle_t             pcm_port;

    mx_handle_t             buffer_vmo;
    size_t                  buffer_size;        // size of buffer in bytes
    uint32_t                buffer_notifications;

    mtx_t buffer_mutex;
    mtx_t mutex;

    bool running;
    bool dead;

    uint32_t* sample_rates;
    int sample_rate_count;

    uint32_t sample_rate;
    int num_channels;
    int audio_frame_size; // size of an audio frame

} bcm_pcm_t;

static mx_status_t pcm_dma_init(bcm_pcm_t* ctx);


#define dev_to_bcm_pcm(dev) containerof(dev, bcm_pcm_t, device)


static void pcm_watch_the_world_burn(mx_status_t status,bcm_pcm_t* ctx){
    printf("Some really bad shit happened, need to tear it all down\n");
    while(1);;
}

static void pcm_deinit(bcm_pcm_t* ctx) {

    // Turn off TX/RX, Clear FIFOs, Clear Errors
    ctx->control_regs->cs           = BCM_PCM_CS_ENABLE | BCM_PCM_CS_TXCLR | BCM_PCM_CS_RXCLR ;

    ctx->control_regs->mode         = BCM_PCM_MODE_INITIAL_STATE;
    ctx->control_regs->txc          = BCM_PCM_TXC_INITIAL_STATE;
    ctx->control_regs->rxc          = BCM_PCM_RXC_INITIAL_STATE;
    ctx->control_regs->dreq_lvl     = BCM_PCM_DREQ_LVL_INITIAL_STATE;
    ctx->control_regs->cs           = BCM_PCM_CS_INITIAL_STATE;
    ctx->running = false;

}

static int pcm_notify_thread(void* arg) {

    bcm_pcm_t* ctx = arg;

    uint32_t offset=0;

    uint32_t notify_period = (1000000 * ctx->buffer_size) / (ctx->sample_rate * 4 * ctx->buffer_notifications);
    printf("BCMPCM: Notification interval = %uuS\n",notify_period);
    while (ctx->running) {
        mx_nanosleep(MX_USEC(notify_period));
        mx_paddr_t pos = bcm_dma_get_position(&ctx->dma);
        //printf("DMA at phys: %lx\n",pos);
        bcm_dma_paddr_to_offset(&ctx->dma,pos,&offset);
        //printf("DMA at offset %u\n",offset);
        audio2_rb_position_notify_t resp;
        resp.hdr.cmd = AUDIO2_RB_POSITION_NOTIFY;
        resp.ring_buffer_pos = offset;
        mx_channel_write( ctx->buffer_ch, 0, &resp, sizeof(resp), NULL, 0);
    }

    return 0;
}


static mx_status_t pcm_start(bcm_pcm_t* ctx, audio2_rb_cmd_start_req_t req, void* dummy, uint32_t none) {

    audio2_rb_cmd_start_resp_t resp;
    //Stuff below should actually be in the start handler
    ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_DMAEN | (0x03 << 15);
    bcm_dma_start(&ctx->dma);
    // turn on i2s transmitter
    ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_DMAEN | BCM_PCM_CS_TXON;
    resp.start_ticks = mx_ticks_get();
    ctx->running = true;

    thrd_t notify_thrd;
    int thrd_rc = thrd_create_with_name(&notify_thrd,
                                        pcm_notify_thread, ctx,
                                        "pcm_notify_thread");
    if (thrd_rc != thrd_success) {
        return thrd_status_to_mx_status(thrd_rc);
    }
    thrd_detach(notify_thrd);


    resp.result = NO_ERROR;
    resp.hdr.transaction_id = req.hdr.transaction_id;
    resp.hdr.cmd = req.hdr.cmd;

    return mx_channel_write( ctx->buffer_ch, 0, &resp, sizeof(resp), NULL, 0);
    //i2s is running at this point
}



static mx_status_t pcm_set_stream_fmt(bcm_pcm_t* ctx, audio2_stream_cmd_set_format_req_t req) {
    printf("Audio2 set stream format request\n");
    mx_status_t status;
    // This is where we should set up the codec for the format requested
    //      we also need to create the rb channel, and bind the port to it.


    //Check current state, see what we need to shutdown/teardown
        // Is DMA currently running?
        // Is the PCM currently running?
    if (ctx->running) { //currently running, need to bounce back to known state
        pcm_deinit(ctx);
    }

    if (ctx->dma.state != BCM_DMA_STATE_SHUTDOWN) {
        bcm_dma_deinit(&ctx->dma);
    }

    //if (!pcm5122_is_valid_mode(req)) return ERR_NOT_SUPPORTED;

    // This will need to change once we have new modes
    ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_TXCLR;
    ctx->control_regs->mode = BCM_PCM_MODE_I2S_16BIT_64BCLK;
    ctx->control_regs->txc  = BCM_PCM_TXC_I2S_16BIT_64BCLK;
    ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_TXCLR;
    ctx->sample_rate = 44100;

    mx_nanosleep(MX_MSEC(10));

    pcm_dma_init(ctx);


    // Might make sense to split the codec init vs codec start
    status = pcm5122_init();
    if (status != NO_ERROR) return status;

    mx_handle_t ret_handle;
    status = mx_channel_create(0, &ctx->buffer_ch, &ret_handle);
    if (status != NO_ERROR) return ERR_INTERNAL;

    status = mx_port_bind( ctx->pcm_port, (uint64_t)ctx->buffer_ch, ctx->buffer_ch, MX_CHANNEL_READABLE | MX_CHANNEL_PEER_CLOSED);

    if (status != NO_ERROR) {
        pcm_watch_the_world_burn(status,ctx);
        return status;
    }

    audio2_stream_cmd_set_format_resp_t resp;
    resp.hdr.transaction_id = req.hdr.transaction_id;
    resp.hdr.cmd = AUDIO2_STREAM_CMD_SET_FORMAT;
    resp.result = NO_ERROR;

    status = mx_channel_write( ctx->stream_ch, 0, &resp, sizeof(resp), &ret_handle, 1 );
    return status;

    //Set up the pcm periperhal for the mode requested

    //Set up the codec for the mode requested

    //Set up the DMA to support the mode requested

}


static void set_pcm_clock(bcm_pcm_t* pcm_ctx) {
    volatile uint32_t* pcmclk = &pcm_ctx->clock_regs[0x26];
    volatile uint32_t* pcmdiv = &pcm_ctx->clock_regs[0x27];

    *pcmclk = 0x5a000021;
    *pcmdiv = 0x5a006cd4;
    *pcmclk = 0x5a000211;
}

static mx_status_t pcm_audio_sink_release(mx_device_t* device) {
    bcm_pcm_t* ctx = dev_to_bcm_pcm(device);
    free(ctx);
    return NO_ERROR;
}

static void pcm_audio_sink_unbind(mx_device_t* device) {

    bcm_pcm_t* ctx = dev_to_bcm_pcm(device);
    ctx->dead = true;
    //update_signals(ctx);
    //completion_signal(&ctx->free_write_completion);
    device_remove(&ctx->device);
}


static mx_status_t pcm_set_buffer(bcm_pcm_t* ctx, audio2_rb_cmd_set_buffer_req_t req, mx_handle_t vmo) {

    ctx->buffer_vmo = vmo;
    ctx->buffer_size = req.ring_buffer_bytes;
    printf("recieved %lu byte vmo\n",ctx->buffer_size);
    ctx->buffer_notifications = req.notifications_per_ring;
    printf("doing %d notifications per ring\n",ctx->buffer_notifications);


    uint32_t transfer_info =  BCM_DMA_DREQ_ID_PCM_TX << 16 |
                              BCM_DMA_TI_DEST_DREQ         |
                              BCM_DMA_TI_SRC_INC           |
                              BCM_DMA_TI_WAIT_RESP         | (15 << 21);

    mx_paddr_t dest_addr     = 0x7e000000 | ( 0x00ffffff & (uint32_t)(mx_paddr_t)(&((bcm_pcm_regs_t*)I2S_BASE)->fifo));

    mx_status_t status = bcm_dma_link_vmo_to_peripheral(&ctx->dma, ctx->buffer_vmo, transfer_info, dest_addr);


    audio2_rb_cmd_set_buffer_resp_t resp;
    resp.hdr.cmd = req.hdr.cmd;
    resp.hdr.transaction_id = req.hdr.transaction_id;
    resp.result = status;

    status = mx_channel_write( ctx->buffer_ch, 0, &resp, sizeof(resp), NULL, 0);
    return status;
}

#define HANDLE_REQ(_ioctl, _payload, _handler)      \
case _ioctl:                                        \
    if (req_size != sizeof(req._payload)) {         \
        printf("Bad " #_payload                     \
                  " reqonse length (%u != %zu)\n",  \
                  req_size, sizeof(req._payload));  \
        return ERR_INVALID_ARGS;                    \
    }                                               \
    _handler(ctx, req._payload);
static int pcm_port_thread(void *arg) {

    bcm_pcm_t* ctx = arg;
    mx_status_t status;

    mx_io_packet_t port_out;

    stream_packet_t req;
    buffer_packet_t buff_req;

    while (1) {
        printf("waiting again...\n");
        status = mx_port_wait(ctx->pcm_port, MX_TIME_INFINITE, &port_out, sizeof(port_out));
        if (status != NO_ERROR) pcm_watch_the_world_burn(status,ctx);

        mx_handle_t channel = (mx_handle_t)port_out.hdr.key;
        printf("Got port key:%lx\n",port_out.hdr.key);
        printf("Got port type:%x\n",port_out.hdr.type);
        printf("Got port extra:%x\n",port_out.hdr.extra);
        printf("Got port signals:%x\n",port_out.signals);
        uint32_t req_size;

        if (channel == ctx->stream_ch) {

            status = mx_channel_read(channel,0,&req,sizeof(req),&req_size,NULL,0,NULL);
            if (status != NO_ERROR) pcm_watch_the_world_burn(status,ctx);
            printf("Read %u of %lu bytes from stream channel\n",req_size,sizeof(req));

            switch(req.hdr.cmd) {
                HANDLE_REQ(AUDIO2_STREAM_CMD_SET_FORMAT, set_fmt_req, pcm_set_stream_fmt);
                break;
            default:
                printf("Unrecognized stream command 0x%04x\n", req.hdr.cmd);
                return ERR_NOT_SUPPORTED;
            }
        } else if(channel == ctx->buffer_ch) {

            if (port_out.signals == MX_CHANNEL_READABLE) {
                mx_handle_t handles[4];
                uint32_t num_handles;
                status = mx_channel_read(channel,0,&buff_req,sizeof(buff_req),&req_size,handles,4,&num_handles);
                if (status != NO_ERROR) pcm_watch_the_world_burn(status,ctx);
                printf("Read %u of %lu bytes from buffer channel\n",req_size,sizeof(req));
                printf("    Got command 0x%04x\n",req.hdr.cmd);
                printf("    Received %u handles\n",num_handles);
                switch(buff_req.hdr.cmd) {
                    case AUDIO2_RB_CMD_SET_BUFFER:
                        pcm_set_buffer(ctx,buff_req.set_buffer_req,handles[0]);
                        break;

                    case AUDIO2_RB_CMD_START:
                        pcm_start(ctx,buff_req.start_req,NULL,0);
                        break;
                    case AUDIO2_RB_CMD_STOP:
                    case AUDIO2_STREAM_CMD_SET_FORMAT:
                    case AUDIO2_RB_POSITION_NOTIFY:
                    default:
                        printf("unrecognized buffer command\n");
                }
            }

        }
/*
        printf("port.key  %lx\n",port_out.hdr.key);
        printf("port.type  %x\n",port_out.hdr.type);
        printf("port.signals %x\n",port_out.signals);
*/
    }

    return -1;
}


static ssize_t pcm_audio2_sink_ioctl(mx_device_t* device, uint32_t op,
                          const void* in_buf, size_t in_len,
                          void* out_buf, size_t out_len) {

    bcm_pcm_t* ctx = dev_to_bcm_pcm(device);
    mx_handle_t* reply = out_buf;

    if (op != AUDIO2_IOCTL_GET_CHANNEL) return ERR_INVALID_ARGS;

    if (ctx->stream_ch != MX_HANDLE_INVALID) return ERR_BAD_STATE;

    mx_handle_t ret_handle;
    mx_status_t status = mx_channel_create(0, &ctx->stream_ch, &ret_handle);
    if (status != NO_ERROR) return ERR_INTERNAL;
    *reply = ret_handle;

    status = mx_port_create(0,&ctx->pcm_port);
    if (status != NO_ERROR) {
        mx_handle_close(ctx->stream_ch);
        return status;
    }

    status = mx_port_bind( ctx->pcm_port, (uint64_t)ctx->stream_ch, ctx->stream_ch, MX_CHANNEL_READABLE | MX_CHANNEL_PEER_CLOSED);

    if (status != NO_ERROR) {
        mx_handle_close(ctx->stream_ch);
        mx_handle_close(ctx->pcm_port);
        return status;
    }

    thrd_t port_thrd;
    int thrd_rc = thrd_create_with_name(&port_thrd,
                                        pcm_port_thread, ctx,
                                        "pcm_port_thread");
    if (thrd_rc != thrd_success) {
        mx_handle_close(ctx->stream_ch);
        mx_handle_close(ctx->pcm_port);
        return thrd_status_to_mx_status(thrd_rc);
    }
    thrd_detach(port_thrd);

    return NO_ERROR;
}




static mx_status_t pcm_dma_init(bcm_pcm_t* ctx) {

    mx_status_t status = bcm_dma_init(&ctx->dma, DMA_CHAN);
    if (status != NO_ERROR) return status;

    return NO_ERROR;
}

static mx_protocol_device_t pcm_audio_ctx_device_proto = {
    .unbind = pcm_audio_sink_unbind,
    .release = pcm_audio_sink_release,
    //.write = pcm_audio_sink_write,
    .ioctl = pcm_audio2_sink_ioctl,
};

static int pcm_bootstrap_thread(void *arg) {

    assert(arg);

    printf("\n\nEntering the pcm bootstrap\n");

    bcm_pcm_t* pcm_ctx = (bcm_pcm_t*)arg;

    // Carve out some address space for the clock control registers
    mx_status_t status = mx_mmap_device_memory(
        get_root_resource(),
        BCM_CM_BASE, 0x1000,
        MX_CACHE_POLICY_UNCACHED_DEVICE, (uintptr_t*)&pcm_ctx->clock_regs);

    if (status != NO_ERROR)
        goto pcm_err;


    // Carve out some address space for the device -- it's memory mapped.
    status = mx_mmap_device_memory(
        get_root_resource(),
        GPIO_BASE, 0x1000,
        MX_CACHE_POLICY_UNCACHED_DEVICE, (uintptr_t*)&pcm_ctx->gpio_regs);

    if (status != NO_ERROR)
        goto pcm_err;

    /* ALT Function 0 is PCM for these pins */
    set_gpio_function(pcm_ctx->gpio_regs, BCM_PCM_CLK_ALT0_PIN, FSEL_ALT0);
    set_gpio_function(pcm_ctx->gpio_regs, BCM_PCM_FS_ALT0_PIN, FSEL_ALT0);
    set_gpio_function(pcm_ctx->gpio_regs, BCM_PCM_DIN_ALT0_PIN, FSEL_ALT0);
    set_gpio_function(pcm_ctx->gpio_regs, BCM_PCM_DOUT_ALT0_PIN, FSEL_ALT0);

    status = mx_mmap_device_memory(
        get_root_resource(),
        I2S_BASE, 0x1000,
        MX_CACHE_POLICY_UNCACHED_DEVICE, (uintptr_t*)&pcm_ctx->control_regs);

    if (status != NO_ERROR)
        goto pcm_err;


#if 0
//  This is temporary until we have working hooks to receive vmo from client
#define PCM_VMO_SIZE 5000
    status = mx_vmo_create(PCM_VMO_SIZE, 0, &pcm_ctx->buffer_vmo);
    if (status != NO_ERROR) goto pcm_err;


    status = mx_vmo_op_range(pcm_ctx->buffer_vmo, MX_VMO_OP_COMMIT, 0, PCM_VMO_SIZE, NULL, 0);
    if (status != NO_ERROR) {
        printf("Error committing vmo\n");
        goto pcm_err;
    }

// Put some junk in there just so we know it is running
    uint16_t count = 0;
    for (uint32_t i = 0 ; i < (PCM_VMO_SIZE/2); i++) {

        mx_vmo_write(pcm_ctx->buffer_vmo, &count, i*sizeof(count), sizeof(count), NULL);
        count = (count + 1) % 30000;
    }
    mx_vmo_op_range(pcm_ctx->buffer_vmo, MX_VMO_OP_CACHE_CLEAN, 0, 5000, NULL, 0);
//***************************************************************
#endif
    set_pcm_clock(pcm_ctx);
#if 0
    pcm_ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_TXCLR;
    pcm_ctx->control_regs->mode = BCM_PCM_MODE_I2S_16BIT_64BCLK;
    pcm_ctx->control_regs->txc  = BCM_PCM_TXC_I2S_16BIT_64BCLK;
    pcm_ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_TXCLR;

    mx_nanosleep(MX_MSEC(100)); //Use a big hammer, need to tighten this up


    pcm_dma_init(pcm_ctx);
    pcm_ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_DMAEN | (0x03 << 15);
    bcm_dma_start(&pcm_ctx->dma);

    // turn on i2s transmitter
    pcm_ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_DMAEN | BCM_PCM_CS_TXON;
    //i2s is running at this point
#endif
    device_init(&pcm_ctx->device, pcm_ctx->driver, "pcm0", &pcm_audio_ctx_device_proto);

    pcm_ctx->device.protocol_id = MX_PROTOCOL_AUDIO2_OUTPUT;
    pcm_ctx->device.protocol_ops = NULL;

    status = device_add(&pcm_ctx->device, pcm_ctx->parent);

    return 0;

pcm_err:
    if (pcm_ctx)
        free(pcm_ctx);

    return -1;
}

static mx_status_t bcm_pcm_bind(mx_driver_t* driver, mx_device_t* parent, void** cookie) {

    bcm_pcm_t* pcm_ctx = calloc(1, sizeof(*pcm_ctx));
    if (!pcm_ctx)
        return ERR_NO_MEMORY;

    pcm_ctx->driver = driver;
    pcm_ctx->parent = parent;

    thrd_t bootstrap_thrd;
    int thrd_rc = thrd_create_with_name(&bootstrap_thrd,
                                        pcm_bootstrap_thread, pcm_ctx,
                                        "pcm_bootstrap_thread");
    if (thrd_rc != thrd_success) {
        free(pcm_ctx);
        return thrd_status_to_mx_status(thrd_rc);
    }

    thrd_detach(bootstrap_thrd);
    return NO_ERROR;
}

mx_driver_t _driver_bcm_pcm = {
    .ops = {
        .bind = bcm_pcm_bind,
    },
};

MAGENTA_DRIVER_BEGIN(_driver_bcm_pcm, "bcm-pcm", "magenta", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PROTOCOL, MX_PROTOCOL_SOC),
    BI_ABORT_IF(NE, BIND_SOC_VID, SOC_VID_BROADCOMM),
    BI_MATCH_IF(EQ, BIND_SOC_DID, SOC_DID_BROADCOMM_PCM),
MAGENTA_DRIVER_END(_driver_bcm_pcm)