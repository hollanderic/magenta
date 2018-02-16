// Copyright 2017 The Fuchsia Authors. All rights reserved.
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

//ETH_REG0 definitions
//        (ending in _POS indicates bit position)
#define ETH_REG0_RGMII_SEL      (1 << 0)
#define ETH_REG0_DATA_ENDIAN    (1 << 1)
#define ETH_REG0_DESC_ENDIAN    (1 << 2)
#define ETH_REG0_RX_CLK_INV     (1 << 3)
#define ETH_REG0_TX_CLK_SRC     (1 << 4)
#define ETH_REG0_TX_CLK_PH_POS       (5)
#define ETH_REG0_TX_CLK_RATIO_POS    (7)
#define ETH_REG0_REF_CLK_ENA    (1 << 10)
#define ETH_REG0_RMII_INV       (1 << 11)
#define ETH_REG0_CLK_ENA        (1 << 12)
#define ETH_REG0_ADJ_ENA        (1 << 13)
#define ETH_REG0_ADJ_SETUP      (1 << 14)
#define ETH_REG0_ADJ_DELAY_POS  (15)
#define ETH_REG0_ADJ_SKEW_POS   (20)
#define ETH_REG0_CALI_START     (1 << 25)
#define ETH_REG0_CALI_RISE      (1 << 26)
#define ETH_REG0_CALI_SEL_POS   (27)
#define ETH_REG0_RX_REUSE       (1 << 30)
#define ETH_REG0_URGENT         (1 << 31)


// miiaddr register defintions
#define MII_BUSY        (1 << 0)
#define MII_WRITE        (1 << 1)
#define MII_CLKRANGE_60_100M    (0)
#define MII_CLKRANGE_100_150M    (0x4)
#define MII_CLKRANGE_20_35M    (0x8)
#define MII_CLKRANGE_35_60M    (0xC)
#define MII_CLKRANGE_150_250M    (0x10)
#define MII_CLKRANGE_250_300M    (0x14)

#define MIIADDRSHIFT        (11)
#define MIIREGSHIFT        (6)
#define MII_REGMSK        (0x1F << 6)
#define MII_ADDRMSK        (0x1F << 11)


#define MII_BMCR        0x00    /* Basic mode control register */
#define MII_BMSR        0x01    /* Basic mode status register  */
#define MII_PHYSID1        0x02    /* PHYS ID 1               */
#define MII_PHYSID2        0x03    /* PHYS ID 2               */
#define MII_ADVERTISE        0x04    /* Advertisement control reg   */
#define MII_LPA            0x05    /* Link partner ability reg    */
#define MII_EXPANSION        0x06    /* Expansion register           */
#define MII_CTRL1000        0x09    /* 1000BASE-T control           */
#define MII_STAT1000        0x0a    /* 1000BASE-T status           */
#define MII_ESTATUS        0x0f    /* Extended Status */
#define MII_DCOUNTER        0x12    /* Disconnect counter           */
#define MII_FCSCOUNTER        0x13    /* False carrier counter       */
#define MII_NWAYTEST        0x14    /* N-way auto-neg test reg     */
#define MII_RERRCOUNTER     0x15    /* Receive error counter       */
#define MII_SREVISION        0x16    /* Silicon revision           */
#define MII_RESV1        0x17    /* Reserved...               */
#define MII_LBRERROR        0x18    /* Lpback, rx, bypass error    */
#define MII_PHYADDR        0x19    /* PHY address               */
#define MII_RESV2        0x1a    /* Reserved...               */
#define MII_TPISTATUS        0x1b    /* TPI status for 10mbps       */
#define MII_NCONFIG        0x1c    /* Network interface config    */


#define DESC_TXCTRL_TXCHAIN 0

typedef volatile struct dw_mac_regs {
    uint32_t conf;        /* 0x00 */
    uint32_t framefilt;        /* 0x04 */
    uint32_t hashtablehigh;    /* 0x08 */
    uint32_t hashtablelow;    /* 0x0c */
    uint32_t miiaddr;        /* 0x10 */
    uint32_t miidata;        /* 0x14 */
    uint32_t flowcontrol;    /* 0x18 */
    uint32_t vlantag;        /* 0x1c */
    uint32_t version;        /* 0x20 */
    uint8_t  reserved_1[20];
    uint32_t intreg;        /* 0x38 */
    uint32_t intmask;        /* 0x3c */
    uint32_t macaddr0hi;        /* 0x40 */
    uint32_t macaddr0lo;        /* 0x44 */
} dw_mac_regs_t;



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
    void EthmacStop();
    zx_status_t EthmacStart(fbl::unique_ptr<ddk::EthmacIfcProxy> proxy);
    zx_status_t EthmacQueueTx(uint32_t options, ethmac_netbuf_t* netbuf);
    zx_status_t EthmacSetParam(uint32_t param, int32_t value, void* data);
    zx_status_t MDIOWrite(uint32_t reg, uint32_t val);
    zx_status_t MDIORead(uint32_t reg, uint32_t* val);
    //int Thread();

  private:
    //zx_status_t UpdateLinkStatus(zx_signals_t observed);
    //zx_status_t Recv(uint8_t* buffer, uint32_t capacity);
    zx_status_t InitBuffers();


    //Number each of tx/rx transaction descriptor
    static constexpr uint32_t kNumDesc_    = 16;
    //Size of each transaction buffer
    static constexpr uint32_t kTxnBufSize_ = 2048;

    dw_dmadescr* tx_descriptors_;
    dw_dmadescr* rx_descriptors_;
    fbl::VmoMapper dma_desc_mapper_;
    fbl::VmoMapper dma_buff_mapper_;

    uint8_t* tx_buffer_;
    uint8_t* rx_buffer_;

    // designware mac options
    uint32_t options_ = 0;

    // ethermac fields
    uint32_t features_ = 0;
    uint32_t mtu_ = 0;
    uint8_t mac_[6] = {};
    uint16_t mii_addr_;

    platform_device_protocol_t pdev_;

    zx_vaddr_t periph_regs_;
    size_t periph_regs_size_;
    zx::vmo periph_regs_vmo_;

    dw_mac_regs_t* dwmac_regs_;
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

    thrd_t thread_;
};

}  // namespace eth
