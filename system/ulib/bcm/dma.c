// Copyright 2017 The Fuchsia Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ddk/io-buffer.h>
#include <ddk/protocol/bcm.h>

#include <magenta/syscalls.h>
#include <magenta/compiler.h>
#include <magenta/threads.h>

#include <bcm/dma.h>
#include <bcm/bcm28xx.h>

#define BCM_DMA_PAGE_SIZE   4096
#define BCM_DMA_NUM_CONTROL_BLOCKS   (64)

static bcm_dma_ctrl_regs_t* dma_regs = NULL;

mx_status_t bcm_dma_get_ctl_blk(bcm_dma_t* dma, bcm_dma_cb_t* cb, mx_paddr_t* pa) {

    return NO_ERROR;
}

bool bcm_dma_is_running(bcm_dma_t* dma) {
    return true;
}

mx_status_t bcm_dma_init(bcm_dma_t* dma, uint32_t ch) {
    mx_status_t status;

    if (dma_regs == NULL) {
        status = mx_mmap_device_memory(
            get_root_resource(),
            DMA_BASE, 0x1000,
            MX_CACHE_POLICY_UNCACHED_DEVICE, (uintptr_t*)&dma_regs);

        if (status != NO_ERROR)
            return status;
    }

    status = io_buffer_init(&dma->ctl_blks, BCM_DMA_NUM_CONTROL_BLOCKS * sizeof(bcm_dma_cb_t), IO_BUFFER_RW);
    if (status != NO_ERROR) {
        printf("\nBCM_DMA: Error Allocating control blocks: %d\n",status);
        return status;
    }
    dma->ctl_blk_mask = 0;
    dma->ch_num = ch;

    dma_regs->channels[dma->ch_num].cs = BCM_DMA_CS_RESET;  //reset the channel

    dma->state |= BCM_DMA_STATE_INITIALIZED;

    return NO_ERROR;
}

/*
    Takes a vmo and links together control blocks, one for each page in vmo
        This assumes a transfer to single non-incrementing address paddr
*/
mx_status_t bcm_dma_link_vmo_to_peripheral(bcm_dma_t* dma, mx_handle_t vmo, uint32_t t_info, mx_paddr_t dest) {

    if ( (dma->state & BCM_DMA_STATE_INITIALIZED) == 0) return ERR_BAD_STATE;

    size_t buffsize;
    mx_status_t status = mx_vmo_get_size(vmo, &buffsize);
    if (status != NO_ERROR) {
        return status;
    }

    uint32_t num_pages = (buffsize / BCM_DMA_PAGE_SIZE) +
                                            (((buffsize % BCM_DMA_PAGE_SIZE) > 0)? 1 : 0);

    mx_paddr_t* buf_pages = calloc(num_pages,sizeof(mx_paddr_t));

    status = mx_vmo_op_range(vmo, MX_VMO_OP_LOOKUP, 0, buffsize, buf_pages, sizeof(mx_paddr_t)*num_pages);
    if (status != NO_ERROR) goto dma_link_err;

    ssize_t total_bytes = buffsize;

    bcm_dma_cb_t* cb = (bcm_dma_cb_t*) io_buffer_virt(&dma->ctl_blks);

    for (uint32_t i = 0; i < num_pages; i++) {

        cb[i].transfer_info = t_info;

        cb[i].source_addr   = (uint32_t)( buf_pages[i] | BCM_SDRAM_BUS_ADDR_BASE);
        cb[i].dest_addr     = (uint32_t)dest;

        uint32_t tfer_len = (total_bytes > BCM_DMA_PAGE_SIZE) ? BCM_DMA_PAGE_SIZE : total_bytes;
        cb[i].transfer_len  = tfer_len;
        total_bytes -= tfer_len;

        uint32_t next_cb_offset = (total_bytes > 0) ? ( sizeof( bcm_dma_cb_t) * (i + 1) ) : 0;
        cb[i].next_ctl_blk_addr = (uint32_t)( (io_buffer_phys(&dma->ctl_blks) + next_cb_offset) |BCM_SDRAM_BUS_ADDR_BASE);
        dma->ctl_blk_mask |= (1 << i);
    }

    io_buffer_cache_op(&dma->ctl_blks, MX_VMO_OP_CACHE_CLEAN, 0, num_pages*sizeof(bcm_dma_cb_t));

    dma->state |= BCM_DMA_STATE_READY;

dma_link_err:
    if (buf_pages) free(buf_pages);
    return status;
}


mx_status_t bcm_dma_start(bcm_dma_t* dma) {

    if ( (dma_regs == NULL) || (dma->state == 0)) return ERR_BAD_STATE;

    dma_regs->channels[dma->ch_num].ctl_blk_addr =  (uint32_t)(io_buffer_phys(&dma->ctl_blks) | BCM_SDRAM_BUS_ADDR_BASE);
    dma_regs->channels[dma->ch_num].cs |= (BCM_DMA_CS_ACTIVE | BCM_DMA_CS_WAIT);

    return NO_ERROR;
}

mx_status_t bcm_dma_release(bcm_dma_t* dma) {

    // check if running, if so, shut it down

    // let go of the iobuffer we use for control blocks
    if (io_buffer_is_valid(&dma->ctl_blks)) {
        io_buffer_release(&dma->ctl_blks);
    }

    return NO_ERROR;
}

mx_status_t bcm_dma_deinit(bcm_dma_t* dma) {

    dma_regs->channels[dma->ch_num].cs = BCM_DMA_CS_RESET;
    dma_regs->channels[dma->ch_num].ctl_blk_addr = (uint32_t)(0);
    bcm_dma_release(dma);

    dma->state = BCM_DMA_STATE_SHUTDOWN;

    return NO_ERROR;
}