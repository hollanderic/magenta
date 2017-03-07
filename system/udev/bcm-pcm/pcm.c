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
#include <ddk/io-buffer.h>
#include <ddk/protocol/bcm.h>

#include <magenta/syscalls.h>
#include <magenta/threads.h>
#include <magenta/device/i2c.h>
#include <magenta/device/audio2.h>
#include <magenta/compiler.h>

#include <bcm/dma.h>
#include <bcm/gpio.h>
#include <bcm/bcm28xx.h>

#include "pcm.h"

#define BCM_FIFO_DEADLINE_MS 100

//ideally we would get this from system, just putting it here now for reference
#define BCM_PCM_PAGE_SIZE   4096

#define DMA_CHAN 11
#define PCM_TRACE 0

typedef struct {

    mx_device_t             device;
    mx_device_t*            parent;
    mx_driver_t*            driver;
    bcm_pcm_regs_t*         control_regs;
    bcm_gpio_ctrl_t*        gpio_regs;
    bcm_dma_ctrl_regs_t*    dma_regs;
    volatile uint32_t*      clock_regs;

    mx_handle_t             stream_ch;
    mx_handle_t             stream_port;

    mx_handle_t             buffer_vmo;
    size_t                  buffer_size;        // size of buffer in bytes

    io_buffer_t             ctl_blocks;
    mtx_t buffer_mutex;
    mtx_t mutex;

    bool started;
    bool dead;

    uint32_t* sample_rates;
    int sample_rate_count;

    uint32_t sample_rate;
    int num_channels;
    int audio_frame_size; // size of an audio frame

} bcm_pcm_t;

//static uint32_t next_cb = 0;

#define dev_to_bcm_pcm(dev) containerof(dev, bcm_pcm_t, device)



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

static ssize_t pcm_audio2_sink_ioctl(mx_device_t* device, uint32_t op,
                          const void* in_buf, size_t in_len,
                          void* out_buf, size_t out_len) {

    bcm_pcm_t* ctx = dev_to_bcm_pcm(device);
    mx_handle_t* reply = out_buf;

    if (op != AUDIO2_IOCTL_GET_CHANNEL) return ERR_INVALID_ARGS;

    if (ctx->stream_ch != NULL) return ERR_BAD_STATE;

    printf("BCMPCM: ioctl get channel\n");
    mx_handle_t ret_handle;
    mx_status_t status = mx_channel_create(0, &ctx->stream_ch, &ret_handle);
    if (status != NO_ERROR) return ERR_INTERNAL;
    *reply = ret_handle;



    return sizeof(mx_handle_t);
}


static inline void pcm_dma_set_active(bcm_pcm_t* ctx) {
    ctx->dma_regs->channels[DMA_CHAN].cs |= (BCM_DMA_CS_ACTIVE | BCM_DMA_CS_WAIT);
}


static mx_status_t pcm_dma_init(bcm_pcm_t* ctx) {

    mx_status_t status = mx_vmo_get_size(ctx->buffer_vmo, &ctx->buffer_size);
    if (status != NO_ERROR) {
        return status;
    }

    uint32_t num_pages = (ctx->buffer_size / BCM_PCM_PAGE_SIZE) +
                                            (((ctx->buffer_size % BCM_PCM_PAGE_SIZE) > 0)? 1 : 0);

    mx_paddr_t* buf_pages = calloc(num_pages,sizeof(mx_paddr_t));

    status = mx_vmo_op_range(ctx->buffer_vmo, MX_VMO_OP_LOOKUP, 0, ctx->buffer_size, buf_pages, sizeof(mx_paddr_t)*num_pages);
    if (status != NO_ERROR) goto dma_err;

    /*  Allocate memory for dma control blocks
            Needs to be 256bit aligned, but we get it for free since this is page aligned.
            TODO - need to add extra control blocks for buffer position notifications
    */
    status = io_buffer_init(&ctx->ctl_blocks, num_pages*sizeof(bcm_dma_cb_t), IO_BUFFER_RW);
    if (status != NO_ERROR) {
        printf("\nBCM_PCM: Error Allocating control blocks: %d\n\n",status);
        goto dma_err;
    }

    ssize_t total_bytes = ctx->buffer_size;
    bcm_dma_cb_t* cb = (bcm_dma_cb_t*) io_buffer_virt(&ctx->ctl_blocks);

    for (uint32_t i = 0; i < num_pages; i++) {

        cb[i].transfer_info = BCM_DMA_DREQ_ID_PCM_TX << 16 |
                              BCM_DMA_TI_DEST_DREQ         |
                              BCM_DMA_TI_SRC_INC           |
                              BCM_DMA_TI_WAIT_RESP         | (15 << 21);

        cb[i].source_addr   = (uint32_t)( buf_pages[i] | BCM_SDRAM_BUS_ADDR_BASE);
        cb[i].dest_addr     = 0x7e000000 | ( 0x00ffffff & (uint32_t)(mx_paddr_t)(&((bcm_pcm_regs_t*)I2S_BASE)->fifo));

        uint32_t tfer_len = (total_bytes > BCM_PCM_PAGE_SIZE) ? BCM_PCM_PAGE_SIZE : total_bytes;
        cb[i].transfer_len  = tfer_len;
        total_bytes -= tfer_len;

        uint32_t next_cb_offset = (total_bytes > 0) ? ( sizeof( bcm_dma_cb_t) * (i + 1) ) : 0;
        cb[i].next_ctl_blk_addr = (uint32_t)( (io_buffer_phys(&ctx->ctl_blocks) + next_cb_offset) |BCM_SDRAM_BUS_ADDR_BASE);
    }

    io_buffer_cache_op(&ctx->ctl_blocks, MX_VMO_OP_CACHE_CLEAN, 0, num_pages*sizeof(bcm_dma_cb_t));

    bcm_dma_chan_t* dmactl = &ctx->dma_regs->channels[DMA_CHAN];    //Arbitrarily picking dma channel

    dmactl->cs = BCM_DMA_CS_RESET;  //reset the channel

    dmactl->ctl_blk_addr = (uint32_t)(io_buffer_phys(&ctx->ctl_blocks) | BCM_SDRAM_BUS_ADDR_BASE);

#if PCM_TRACE
    for (uint32_t i = 0; i < num_pages; i++) {
        printf("Control Block %u    0x%08x\n",i, (uint32_t)(io_buffer_phys(&ctx->ctl_blocks) + sizeof( bcm_dma_cb_t) * i));
        printf("   Length : %u\n",cb[i].transfer_len);
        printf("   Start  : 0x%08x\n",cb[i].source_addr);
        printf("   Next Cb: 0x%08x\n",cb[i].next_ctl_blk_addr);
    }
#endif

dma_err:
    if (buf_pages) free(buf_pages);
    return status;
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

    // Carve out some address space for the dma control registers
    status = mx_mmap_device_memory(
        get_root_resource(),
        DMA_BASE, 0x1000,
        MX_CACHE_POLICY_UNCACHED_DEVICE, (uintptr_t*)&pcm_ctx->dma_regs);

    if (status != NO_ERROR)
        goto pcm_err;

    printf("DMA regs at %p\n",pcm_ctx->dma_regs);
    printf("int status at %p\n",&pcm_ctx->dma_regs->int_status);

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

    set_pcm_clock(pcm_ctx);

    pcm_ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_TXCLR;
    pcm_ctx->control_regs->mode = BCM_PCM_MODE_I2S_16BIT_64BCLK;
    pcm_ctx->control_regs->txc  = BCM_PCM_TXC_I2S_16BIT_64BCLK;
    pcm_ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_TXCLR;

    mx_nanosleep(MX_MSEC(100)); //Use a big hammer, need to tighten this up


    pcm_dma_init(pcm_ctx);
    pcm_ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_DMAEN | (0x03 << 15);
    pcm_dma_set_active(pcm_ctx);

    // turn on i2s transmitter
    pcm_ctx->control_regs->cs   = BCM_PCM_CS_ENABLE | BCM_PCM_CS_DMAEN | BCM_PCM_CS_TXON;
    //i2s is running at this point

    device_init(&pcm_ctx->device, pcm_ctx->driver, "pcm0", &pcm_audio_ctx_device_proto);

    pcm_ctx->device.protocol_id = MX_PROTOCOL_AUDIO;
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