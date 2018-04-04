// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/device.h>
#include <ddk/protocol/gpio.h>
#include <ddk/protocol/test.h>

#include <ddktl/device.h>
#include <ddktl/protocol/ethernet.h>
#include <ddktl/protocol/test.h>
#include <zircon/compiler.h>
#include <zircon/types.h>
#include <zx/socket.h>
#include <zx/vmo.h>
#include <fbl/mutex.h>
#include <fbl/unique_ptr.h>
#include <threads.h>

typedef volatile struct dw_mac_regs {
    uint32_t conf;            /*0 0x00 */
    uint32_t framefilt;       /*1 0x04 */
    uint32_t hashtablehigh;   /*2 0x08 */
    uint32_t hashtablelow;    /*3 0x0c */
    uint32_t miiaddr;         /*4 0x10 */
    uint32_t miidata;         /*5 0x14 */
    uint32_t flowcontrol;     /*6 0x18 */
    uint32_t vlantag;         /*7 0x1c */
    uint32_t version;         /*8 0x20 */
    uint32_t  reserved_1[5];  /*9 - 13 */
    uint32_t intreg;          /*14 0x38 */
    uint32_t intmask;         /*15 0x3c */
    uint32_t macaddr0hi;      /*16 0x40 */
    uint32_t macaddr0lo;      /*17 0x44 */
    uint32_t reserved_2[36];  /*18 - 53 */
    uint32_t rgmiistatus;     /*54 0xd8 */

} dw_mac_regs_t;

// Offset of DMA regs into dwmac register block
#define DW_DMA_BASE_OFFSET    (0x1000)

typedef volatile struct dw_dma_regs {
    uint32_t busmode;              /*0  0x00 */
    uint32_t txpolldemand;         /*1  0x04 */
    uint32_t rxpolldemand;         /*2  0x08 */
    uint32_t rxdesclistaddr;       /*3  0x0c */
    uint32_t txdesclistaddr;       /*4  0x10 */
    uint32_t status;               /*5  0x14 */
    uint32_t opmode;               /*6  0x18 */
    uint32_t intenable;            /*7  0x1c */
    uint32_t missedframes;         /*8  0x20 */
    uint32_t rxwdt;                /*9  0x24 */
    uint32_t axibusmode;           /*10 0x28 */
    uint32_t axistatus;            /*11 0x2c */
    uint32_t reserved[6];
    uint32_t currhosttxdesc;       /*18 0x48 */
    uint32_t currhostrxdesc;       /*19 0x4c */
    uint32_t currhosttxbuffaddr;   /*20 0x50 */
    uint32_t currhostrxbuffaddr;   /*21 0x54 */
    uint32_t hwfeature;            /*22 0x58 */
} dw_dma_regs_t;

//DMA transaction descriptors
struct dw_dmadescr {
    uint32_t txrx_status;
    uint32_t dmamac_cntl;
    uint32_t dmamac_addr;
    uint32_t dmamac_next;
} __ALIGNED(64);


namespace eth {

class AmlDWMacDevice : public ddk::Device<AmlDWMacDevice, ddk::Unbindable>,
                       public ddk::EthmacProtocol<AmlDWMacDevice> {
  public:
    AmlDWMacDevice(zx_device_t* device);

    static zx_status_t Create(zx_device_t* device);

    void DdkRelease();
    void DdkUnbind();

    zx_status_t EthmacQuery(uint32_t options, ethmac_info_t* info);
    void        EthmacStop();
    zx_status_t EthmacStart(fbl::unique_ptr<ddk::EthmacIfcProxy> proxy);
    zx_status_t EthmacQueueTx(uint32_t options, ethmac_netbuf_t* netbuf);
    zx_status_t EthmacSetParam(uint32_t param, int32_t value, void* data);
    zx_status_t MDIOWrite(uint32_t reg, uint32_t val);
    zx_status_t MDIORead(uint32_t reg, uint32_t* val);
    zx_handle_t EthmacGetBti();
    int Thread();

  private:
    zx_status_t InitBuffers();
    zx_status_t InitDevice();
    void UpdateLinkStatus();
    void DumpRegisters();
    void ProcRxBuffer(uint32_t int_status);
    zx_status_t GetMAC(uint8_t* addr);

    //Number each of tx/rx transaction descriptors
    static constexpr uint32_t kNumDesc_    = 16;
    //Size of each transaction buffer
    static constexpr uint32_t kTxnBufSize_ = 2048;

    dw_dmadescr* tx_descriptors_;
    dw_dmadescr* rx_descriptors_;
    fbl::VmoMapper dma_desc_mapper_;
    fbl::VmoMapper dma_buff_mapper_;

    uint8_t* tx_buffer_;
    uint32_t curr_tx_buf_ = 0;
    uint8_t* rx_buffer_;
    uint32_t curr_rx_buf_ = 0;

    // designware mac options
    uint32_t options_ = 0;
    uint32_t tester=0;

    // ethermac fields
    uint32_t features_ = 0;
    uint32_t mtu_ = 0;
    uint8_t mac_[6] = {};
    uint16_t mii_addr_ = 0;

    zx_handle_t bti_;
    zx_handle_t dma_irq_;

    platform_device_protocol_t pdev_;

    zx_vaddr_t periph_regs_;
    size_t periph_regs_size_;
    zx::vmo periph_regs_vmo_;

    dw_mac_regs_t* dwmac_regs_;
    dw_dma_regs_t* dwdma_regs_;
    size_t dwmac_regs_size_;
    zx::vmo dwmac_regs_vmo_;

    zx_vaddr_t hhi_regs_;
    size_t hhi_regs_size_;
    zx::vmo hhi_regs_vmo_;

    gpio_protocol_t gpio_;

    fbl::Mutex lock_;
    fbl::unique_ptr<ddk::EthmacIfcProxy> ethmac_proxy_ __TA_GUARDED(lock_);

    // Only accessed from Thread, so not locked.
    bool online_ = false;
    zx::socket data_;

    //statistics
    uint32_t rx_packets_;
    uint32_t tx_packets_;
    uint32_t bus_errors_;

    thrd_t thread_;
};

}  // namespace eth
