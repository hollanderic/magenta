// Copyright 2017 The Fuchsia Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/io-buffer.h>
#include <magenta/device/ioctl.h>

#pragma once

#define BCM_DMA_IOCTL_GET_CHANNEL   IOCTL(IOCTL_KIND_GET_HANDLE, 0xFE, 0x00)


#define BCM_DMA_DREQ_ID_NONE        (0)
#define BCM_DMA_DREQ_ID_DSI         (1)
#define BCM_DMA_DREQ_ID_PCM_TX      (2)
#define BCM_DMA_DREQ_ID_PCM_RX      (3)


#define BCM_DMA_CS_ACTIVE           (uint32_t)( 1 << 0 )
#define BCM_DMA_CS_RESET            (uint32_t)( 1 << 31)
#define BCM_DMA_CS_WAIT             (uint32_t)( 1 << 28)


#define BCM_DMA_TI_SRC_INC          (uint32_t)( 1 << 8 )
#define BCM_DMA_TI_DEST_DREQ        (uint32_t)( 1 << 6 )
#define BCM_DMA_TI_WAIT_RESP        (uint32_t)( 1 << 3 )


__BEGIN_CDECLS


typedef volatile struct {
    uint32_t transfer_info;
    uint32_t source_addr;
    uint32_t dest_addr;
    uint32_t transfer_len;
    uint32_t stride;
    uint32_t next_ctl_blk_addr;
    uint32_t reserved1;
    uint32_t reserved2;
} bcm_dma_cb_t;


typedef volatile struct {
    uint32_t    cs;
    uint32_t    ctl_blk_addr;
    uint32_t    transfer_info;
    uint32_t    source_addr;
    uint32_t    dest_addr;
    uint32_t    transfer_len;
    uint32_t    stride;
    uint32_t    next_ctl_blk_addr;
    uint32_t    debug;
    uint32_t    reserved[55];   // 256 bytes (64 words) per channel control block.
} bcm_dma_chan_t;               //  Pad so we can lay them out as array (see below).

typedef volatile struct {

    bcm_dma_chan_t  channels[15];       //note: the 16th DMA channel is not in this page
    uint8_t         reserved[0xe0];
    uint32_t        int_status;
    uint8_t         reserved2[12];
    uint32_t        enable;

} bcm_dma_ctrl_regs_t;

typedef enum {
    unused = 0,
    configured,
    running,
    error,
} bcm_dma_states_t;

#define BCM_DMA_STATE_SHUTDOWN      (uint32_t)(0)
#define BCM_DMA_STATE_INITIALIZED   (uint32_t)( 1 << 0)
#define BCM_DMA_STATE_READY         (uint32_t)( 1 << 1)
#define BCM_DMA_STATE_RUNNING       (uint32_t)( 1 << 2)


typedef struct {
    mx_paddr_t  paddr;
    uint32_t    offset;
    uint32_t    len;
} bcm_dma_vmo_index_t;


typedef struct {

    uint32_t                ch_num;
    io_buffer_t             ctl_blks;
    uint64_t                ctl_blk_mask;
    uint32_t                state;
    bcm_dma_vmo_index_t*    vmo_idx;
    uint32_t                vmo_idx_len;

} bcm_dma_t;



typedef enum bcm_dma_cmd {
    // Commands sent on the dma channel
    BCM_DMA_SEND_SHIT = 0x1000,

    // Debug and info commands
    BCM_DMA_DEBUG_SHOW_CLIENTS = 0xF000,
} bcm_dma_cmd_t;

typedef struct bcm_dma_cmd_hdr {
    uint32_t        transaction_id;
    bcm_dma_cmd_t   cmd;
} bcm_dma_cmd_hdr_t;


typedef struct bcm_dma_transfer_req {
    bcm_dma_cmd_hdr_t      hdr;
    uint64_t                source_offset;
    uint64_t                dest_offset;
    size_t                  num_bytes;
} bcm_dma_transfer_req_t;

typedef struct bcm_dma_trasfer_resp {
    bcm_dma_cmd_hdr_t hdr;
    mx_status_t      result;

} bcm_dma_transfer_resp_t;


typedef union {
    bcm_dma_cmd_hdr_t                       hdr;
    bcm_dma_transfer_req_t                  transfer_req;
} dma_packet_t;


mx_status_t bcm_dma_get_ctl_blk(bcm_dma_t* dma, bcm_dma_cb_t* cb, mx_paddr_t* pa);
bool bcm_dma_isrunning(bcm_dma_t* dma);
mx_status_t bcm_dma_start(bcm_dma_t* dma);
mx_status_t bcm_dma_stop(bcm_dma_t* dma);
//mx_status_t bcm_dma_init(bcm_dma_t* dma, uint32_t ch);
mx_status_t bcm_dma_release(bcm_dma_t* dma);
mx_status_t bcm_dma_link_vmo_to_peripheral(bcm_dma_t* dma, mx_handle_t vmo, uint32_t t_info, mx_paddr_t dest);
mx_status_t bcm_dma_deinit(bcm_dma_t* dma);
mx_status_t bcm_dma_paddr_to_offset(bcm_dma_t* dma, mx_paddr_t paddr, uint32_t* offset);
mx_paddr_t bcm_dma_get_position(bcm_dma_t* dma);

__END_CDECLS