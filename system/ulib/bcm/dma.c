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

#if TRACE
#define xprintf(fmt...) printf(fmt)
#else
#define xprintf(fmt...) \
    do {                \
    } while (0)
#endif

#define BCM_DMA_PAGE_SIZE   4096
#define BCM_DMA_NUM_CONTROL_BLOCKS   (64)

static bcm_dma_ctrl_regs_t* dma_regs = NULL;

bool bcm_dma_is_running(bcm_dma_t* dma) {
    return true;
}

static void dma_cb_test(void* arg) {
    //xprintf("Got DMA callback\n");
}

static int dma_irq_thread(void* arg) {
    bcm_dma_t* dma = arg;
    mx_handle_t irq_handle = dma->irq_handle;
    xprintf("BCMDMA: dma interrupt thread started\n");

    mx_status_t stat;
    bool done = false;

    while(!done) {
        stat = mx_interrupt_wait(irq_handle);
        if (stat != NO_ERROR) {
            xprintf("dma interupt wait failed = %d\n",stat);
        }
        mtx_lock(&dma->irq_thrd_lock);
        done = dma->irq_thrd_stop;
        mtx_unlock(&dma->irq_thrd_lock);
        // should put this in different thread and use completion signal here?
        if (dma->callback) {
            (dma->callback)(dma);
        }
        dma_regs->channels[dma->ch_num].cs |= BCM_DMA_CS_INT;
        mx_interrupt_complete(irq_handle);
    }
    mtx_lock(&dma->irq_thrd_lock);
    dma->irq_thrd_stop=false;
    mtx_unlock(&dma->irq_thrd_lock);
    xprintf("BCMDMA: dma interrupt thread quitting\n");
    return 0;
}

mx_status_t bcm_dma_init(bcm_dma_t* dma, uint32_t ch) {

    xprintf("BCMDMA: Initializing dma channel %u\n",ch);
    mx_status_t status;
    mx_handle_t irq_handle = MX_HANDLE_INVALID;

    mtx_lock(&dma->dma_lock);

    if (dma->state != BCM_DMA_STATE_SHUTDOWN) {
        mtx_unlock(&dma->dma_lock);
        return ERR_BAD_STATE;
    }

    if (ch > 14) {   // Don't use ch 15 as it has different properties
        mtx_unlock(&dma->dma_lock);
        return ERR_INVALID_ARGS;
    }

    if (dma_regs == NULL) {
        status = mx_mmap_device_memory(
            get_root_resource(),
            DMA_BASE, 0x1000,
            MX_CACHE_POLICY_UNCACHED_DEVICE, (uintptr_t*)&dma_regs);

        if (status != NO_ERROR) {
            goto dma_init_err;
        }
    }
    xprintf("BCMDMA: Initializing control block buffers\n");
    // pre-init the control block buffer
    status = io_buffer_init(&dma->ctl_blks, BCM_DMA_NUM_CONTROL_BLOCKS * sizeof(bcm_dma_cb_t), IO_BUFFER_RW);
    if (status != NO_ERROR) {
        xprintf("\nBCM_DMA: Error Allocating control blocks: %d\n",status);
        mtx_unlock(&dma->dma_lock);
        return status;
    }

    dma->mem_idx = NULL;
    dma->ctl_blk_mask = 0;
    dma->ch_num = ch;
    dma->callback = dma_cb_test;

    xprintf("BCMDMA: Initializing interrupt handler\n");
    irq_handle = mx_interrupt_create(get_root_resource(),
                                     INTERRUPT_DMA0 + ch,
                                     MX_FLAG_REMAP_IRQ);
    if (irq_handle < 0) {
        xprintf("bcm-dma: failed to create interrupt handle, handle = %d\n",
                irq_handle);
        status = irq_handle;  //clean up
        goto dma_init_err;
    }

    dma->irq_handle = irq_handle;

    dma_regs->channels[dma->ch_num].cs = BCM_DMA_CS_RESET;  //reset the channel

    // Create a thread to handle IRQs.
    xprintf("BCMDMA: Creating interrupt thread\n");
    char thrd_name[20];
    snxprintf(thrd_name,sizeof(thrd_name),"dma%02u_irq_thrd",ch);
    int thrd_rc = thrd_create_with_name(&dma->irq_thrd, dma_irq_thread, dma,
                                        thrd_name);
    if (thrd_rc != thrd_success) {
        xprintf("BCMDMA: failed to create irq thread\n");
        status = thrd_status_to_mx_status(thrd_rc);
        goto dma_init_err;
    }

    dma->state |= BCM_DMA_STATE_INITIALIZED;

    mtx_unlock(&dma->dma_lock);
    return NO_ERROR;

dma_init_err:
    if (io_buffer_is_valid(&dma->ctl_blks)) {
        io_buffer_release(&dma->ctl_blks);
    }
    if (irq_handle > 0) {
        mx_handle_close(irq_handle);
    }

    mtx_unlock(&dma->dma_lock);
    return status;
}


mx_paddr_t bcm_dma_get_position(bcm_dma_t* dma) {
    uint32_t address = (dma_regs->channels[ dma->ch_num ].source_addr) & 0x0fffffff;
    return (mx_paddr_t)address;
}

mx_status_t bcm_dma_paddr_to_offset(bcm_dma_t* dma, mx_paddr_t paddr, uint32_t* offset) {

    // This call only works if an index was created for the memory object
    if (!dma->mem_idx) {
        return ERR_BAD_STATE;
    }

    *offset = 10;
    for (uint32_t i = 0; i < dma->mem_idx_len; i++) {
        if ( (paddr >= dma->mem_idx[i].paddr) && ( paddr < (dma->mem_idx[i].paddr + dma->mem_idx[i].len)) ) {
            *offset =  dma->mem_idx[i].offset + (paddr - dma->mem_idx[i].paddr);
            return NO_ERROR;
        }
    }
    return ERR_OUT_OF_RANGE;

}

/* Builds index of vmo pages.  This is used to translate physical addresses reported by the dma status
    into offsets into the memory object used for the transaction.
*/
static mx_status_t bcm_dma_build_mem_index(bcm_dma_t* dma, mx_paddr_t* page_list, uint32_t len) {

    dma->mem_idx = calloc(len,sizeof(bcm_dma_vmo_index_t));  //Allocate worse case sized array
    if (!dma->mem_idx) return ERR_NO_MEMORY;

    dma->mem_idx_len = 0;
    uint32_t j = 0;

    for( uint32_t i = 0; i < len; i++) {

        for (j = 0; ((page_list[i]  > dma->mem_idx[j].paddr) && (dma->mem_idx[j].paddr != 0)); j++);

        if (( j != 0) && ( (i * BCM_DMA_PAGE_SIZE) == (dma->mem_idx[j - 1].offset + dma->mem_idx[j - 1].len)  ) &&
                         ( (page_list[i] == (dma->mem_idx[j - 1].paddr + dma->mem_idx[j - 1].len) ))) {

            dma->mem_idx[j-1].len += BCM_DMA_PAGE_SIZE;

        } else {

            for( uint32_t k = dma->mem_idx_len ; k > j; k--) {
                memcpy( &dma->mem_idx[k] , &dma->mem_idx[k-1], sizeof(bcm_dma_vmo_index_t));
            }
            dma->mem_idx[j].paddr = page_list[i];
            dma->mem_idx[j].offset = i * BCM_DMA_PAGE_SIZE;
            dma->mem_idx[j].len = BCM_DMA_PAGE_SIZE;
            dma->mem_idx_len++;
        }
    }
    return NO_ERROR;
}


mx_status_t bcm_dma_init_vmo_to_fifo_trans(bcm_dma_t* dma, mx_handle_t vmo, uint32_t t_info,
                                                           mx_paddr_t dest, uint32_t flags   ) {
    xprintf("Linking vmo to fifo...\n");
    mtx_lock(&dma->dma_lock);

    if ( (dma->state & BCM_DMA_STATE_INITIALIZED) == 0) {
        mtx_unlock(&dma->dma_lock);
        return ERR_BAD_STATE;
    }

    size_t buffsize;
    mx_status_t status = mx_vmo_get_size(vmo, &buffsize);
    if (status != NO_ERROR) {
        mtx_unlock(&dma->dma_lock);
        return status;
    }

    uint32_t num_pages = (buffsize / BCM_DMA_PAGE_SIZE) +
                                            (((buffsize % BCM_DMA_PAGE_SIZE) > 0)? 1 : 0);

    mx_paddr_t* buf_pages = calloc(num_pages,sizeof(mx_paddr_t));
    if (!buf_pages) {
        mtx_unlock(&dma->dma_lock);
        return ERR_NO_MEMORY;
    }

    status = mx_vmo_op_range(vmo, MX_VMO_OP_LOOKUP, 0, buffsize, buf_pages, sizeof(mx_paddr_t)*num_pages);
    if (status != NO_ERROR) goto dma_link_err;

    if (flags & BCM_DMA_FLAGS_USE_MEM_INDEX) {
        status = bcm_dma_build_mem_index(dma,buf_pages,num_pages);
        if (status != NO_ERROR) goto dma_link_err;
    }

    ssize_t total_bytes = buffsize;

    // Create the control blocks for this vmo.  control block iobuffer was inited when
    //   dma object was inited.  Currently creates one control block for each page of
    //   memory in the memory object.

    dma->ctl_blk_mask = 0;
    bcm_dma_cb_t* cb = (bcm_dma_cb_t*) io_buffer_virt(&dma->ctl_blks);
    // bus address of the control block buffer
    uint32_t cb_bus_addr = (uint32_t)io_buffer_phys(&dma->ctl_blks) | BCM_SDRAM_BUS_ADDR_BASE;

    for (uint32_t i = 0; i < num_pages; i++) {

        cb[i].transfer_info = t_info;

        cb[i].source_addr   = (uint32_t)( buf_pages[i] | BCM_SDRAM_BUS_ADDR_BASE);
        cb[i].dest_addr     = (uint32_t)dest;

        uint32_t tfer_len = (total_bytes > BCM_DMA_PAGE_SIZE) ? BCM_DMA_PAGE_SIZE : total_bytes;
        cb[i].transfer_len  = tfer_len;
        total_bytes -= tfer_len;

        if (total_bytes > 0) {
            cb[i].next_ctl_blk_addr = cb_bus_addr + (sizeof(bcm_dma_cb_t) * (i+1));
        } else {
            cb[i].next_ctl_blk_addr = (flags & BCM_DMA_FLAGS_CIRCULAR) ? cb_bus_addr : 0;
            if (dma->callback)  cb[i].transfer_info |= BCM_DMA_TI_INTEN;
        }

        if (total_bytes > 0) {

        }
        uint32_t next_cb_offset = (total_bytes > 0) ? ( sizeof( bcm_dma_cb_t) * (i + 1) ) : 0;
        if ( (next_cb_offset == 0) && !(flags & BCM_DMA_FLAGS_CIRCULAR)) {
            cb[i].next_ctl_blk_addr = 0;
        } else {
            cb[i].next_ctl_blk_addr = (uint32_t)( (io_buffer_phys(&dma->ctl_blks) + next_cb_offset) |BCM_SDRAM_BUS_ADDR_BASE);
        }
        dma->ctl_blk_mask |= (1 << i);
    }

    io_buffer_cache_op(&dma->ctl_blks, MX_VMO_OP_CACHE_CLEAN, 0, num_pages*sizeof(bcm_dma_cb_t));

    dma->state |= BCM_DMA_STATE_READY;
    if (buf_pages) free(buf_pages);

    mtx_unlock(&dma->dma_lock);
    return NO_ERROR;

dma_link_err:
    if (buf_pages) free(buf_pages);
    if (dma->mem_idx) {
        free(dma->mem_idx);
        dma->mem_idx_len = 0;
    }
    dma->ctl_blk_mask = 0;
dma_link_ret:
    mtx_unlock(&dma->dma_lock);
    return status;
}

mx_status_t bcm_dma_start(bcm_dma_t* dma) {

    xprintf("BCMDMA: starting dma channel %u\n",dma->ch_num);
    mtx_lock(&dma->dma_lock);
    if ( (dma_regs == NULL) || !(dma->state & BCM_DMA_STATE_READY)){
        mtx_unlock(&dma->dma_lock);
        return ERR_BAD_STATE;
    }

    dma_regs->channels[dma->ch_num].ctl_blk_addr =  (uint32_t)(io_buffer_phys(&dma->ctl_blks) | BCM_SDRAM_BUS_ADDR_BASE);
    dma_regs->channels[dma->ch_num].cs |= (BCM_DMA_CS_ACTIVE | BCM_DMA_CS_WAIT);

    dma->state |= BCM_DMA_STATE_RUNNING;
    mtx_unlock(&dma->dma_lock);
    return NO_ERROR;
}

mx_status_t bcm_dma_stop(bcm_dma_t* dma) {
    xprintf("BCMDMA: Stopping dma channel %u\n",dma->ch_num);
    mtx_lock(&dma->dma_lock);

    if ( (dma_regs == NULL) || (dma->state == BCM_DMA_STATE_SHUTDOWN)){
        mtx_unlock(&dma->dma_lock);
        return ERR_BAD_STATE;
    }

    dma_regs->channels[dma->ch_num].cs &= ~BCM_DMA_CS_ACTIVE;
    dma->state &= ~BCM_DMA_STATE_RUNNING;

    mtx_unlock(&dma->dma_lock);
    return NO_ERROR;
}

mx_status_t bcm_dma_deinit(bcm_dma_t* dma) {
    xprintf("BCMDMA: Deiniting dma channel %u\n",dma->ch_num);

    //Halt any activity (if there is any)
    //bcm_dma_stop(dma);

    mtx_lock(&dma->dma_lock);

    //shut down the irq thread
    xprintf("BCMDMA: Shutting down irq thread\n");
    mtx_lock(&dma->irq_thrd_lock);
    dma->irq_thrd_stop = true;
    mtx_unlock(&dma->irq_thrd_lock);
    thrd_join(dma->irq_thrd, NULL);
    xprintf("BCMDMA: irq thread shut down\n");

    dma_regs->channels[dma->ch_num].cs &= ~BCM_DMA_CS_ACTIVE;

    //Release the irq handle
    mx_handle_close(dma->irq_handle);
    dma->irq_handle = MX_HANDLE_INVALID;

    //Reset the hardware
    dma_regs->channels[dma->ch_num].cs = BCM_DMA_CS_RESET;
    dma_regs->channels[dma->ch_num].ctl_blk_addr = (uint32_t)(0);

    //Release whatever memory we are sitting on
    if(dma->mem_idx) {
        free(dma->mem_idx);
        dma->mem_idx=NULL;
    }
    dma->mem_idx_len = 0;

    if (io_buffer_is_valid(&dma->ctl_blks)) {
        io_buffer_release(&dma->ctl_blks);
    }

    dma->state = BCM_DMA_STATE_SHUTDOWN;

    mtx_unlock(&dma->dma_lock);

    return NO_ERROR;
}