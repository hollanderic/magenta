// Copyright 2017 The Fuchsia Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

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

