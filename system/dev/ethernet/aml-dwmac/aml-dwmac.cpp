// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/debug.h>
#include <ddk/protocol/platform-device.h>
#include <fbl/auto_lock.h>
#include <fbl/type_support.h>
#include <fbl/ref_counted.h>
#include <fbl/ref_ptr.h>
#include <fbl/vmar_manager.h>
#include <fbl/vmo_mapper.h>
#include <hw/reg.h>
#include <soc/aml-s912/s912-hw.h>
#include <zircon/compiler.h>

#include <stdio.h>
#include <string.h>

#include "aml-dwmac.h"
#include "dw-gmac-dma.h"

namespace eth {

enum {
    PHY_RESET,
    PHY_INTR,
};

template <typename T, typename U>
static inline T* offset_ptr(U* ptr, size_t offset) {
    return reinterpret_cast<T*>(reinterpret_cast<uintptr_t>(ptr) + offset);
}


int AmlDWMacDevice::Thread() {
    zxlogf(INFO,"AmLogic ethmac started\n");

    zx_status_t status;
    while (true) {

        uint64_t slots;
        status = zx_interrupt_wait(dma_irq_, &slots);
        if (status != ZX_OK) {
            zxlogf(ERROR,"aml-dwmac: Interrupt error\n");
            break;
        }
        uint32_t stat = dwdma_regs_->status;
        dwdma_regs_->status = stat;
        if (stat & DMA_STATUS_GLI) {
            UpdateLinkStatus();
        }
        if (stat & DMA_STATUS_RI) {
            rx_packets_++;
            ProcRxBuffer(stat);
        }
        if (stat & DMA_STATUS_AIS) {
            bus_errors_++;
            zxlogf(ERROR,"aml-dwmac: abnormal interrupt\n");
        }
        if (stat & DMA_STATUS_TI) {
            tx_packets_++;
        }
    }
    return status;
}


void AmlDWMacDevice::UpdateLinkStatus() {
    bool temp = dwmac_regs_->rgmiistatus & GMAC_RGMII_STATUS_LNKSTS;
    if (temp != online_) {
        online_ = temp;
        fbl::AutoLock lock(&lock_);
        if (ethmac_proxy_ != nullptr) {
            ethmac_proxy_->Status(online_ ? ETH_STATUS_ONLINE : 0u);
        }
    }
    zxlogf(INFO,"aml-dwmac: Link is now %s\n",online_ ? "up": "down");
}

zx_status_t AmlDWMacDevice::InitPdev() {

    zx_status_t status = device_get_protocol(parent_,
                                             ZX_PROTOCOL_PLATFORM_DEV,
                                             &pdev_);
    if (status != ZX_OK) {
        return status;
    }

    status = device_get_protocol(parent_, ZX_PROTOCOL_GPIO, &gpio_);
    if (status != ZX_OK) {
        return status;
    }

    gpio_config(&gpio_, PHY_RESET, GPIO_DIR_OUT);

    // Map amlogic peripheral control registers
    status = pdev_map_mmio_buffer(&pdev_, 0, ZX_CACHE_POLICY_UNCACHED_DEVICE,
                                  &periph_regs_iobuff_);
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not map periph mmio: %d\n",status);
        return status;
    }

    // Map mac control registers and dma control registers
    status = pdev_map_mmio_buffer(&pdev_, 1, ZX_CACHE_POLICY_UNCACHED_DEVICE,
                                  &dwmac_regs_iobuff_);
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not map dwmac mmio: %d\n",status);
        return status;
    }
    dwmac_regs_ = static_cast<dw_mac_regs_t*>(io_buffer_virt(&dwmac_regs_iobuff_));
    dwdma_regs_ = offset_ptr<dw_dma_regs_t>(dwmac_regs_, DW_DMA_BASE_OFFSET);

    // Map HHI regs (clocks and power domains)
    status = pdev_map_mmio_buffer(&pdev_, 2, ZX_CACHE_POLICY_UNCACHED_DEVICE,
                                  &hhi_regs_iobuff_);
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not map hiu mmio: %d\n",status);
        return status;
    }

    // Map dma interrupt
    status = pdev_map_interrupt(&pdev_, 0, &dma_irq_);
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not map dma interrupt\n");
        return status;
    }

    // Get our bti
    status = pdev_get_bti(&pdev_, 0, &bti_);
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not obtain bti: %d\n",status);
        return status;
    }

    return status;

}

zx_status_t AmlDWMacDevice::Create(zx_device_t* device){

    auto mac_device = fbl::make_unique<AmlDWMacDevice>(device);

    zx_status_t status = mac_device->InitPdev();
    if (status != ZX_OK) {
        return status;
    }

// Initialize AMLogic peripheral registers associated with dwmac
    void* pregs = io_buffer_virt(&mac_device->periph_regs_iobuff_);
    //Sorry about the magic...rtfm
    writel(0x1621, offset_ptr<uint32_t>(pregs, PER_ETH_REG0));
    writel(0x20000, offset_ptr<uint32_t>(pregs, PER_ETH_REG1));

    writel(REG2_ETH_REG2_REVERSED | REG2_INTERNAL_PHY_ID,
           offset_ptr<uint32_t>(pregs, PER_ETH_REG2));

    writel(REG3_CLK_IN_EN      | REG3_ETH_REG3_19_RESVERD  |
           REG3_CFG_PHY_ADDR   | REG3_CFG_MODE             |
           REG3_CFG_EN_HIGH    | REG3_ETH_REG3_2_RESERVED,
           offset_ptr<uint32_t>(pregs, PER_ETH_REG3));

// Enable clocks and power domain for dwmac
    void* hregs = io_buffer_virt(&mac_device->hhi_regs_iobuff_);
    set_bitsl( 1 << 3, offset_ptr<uint32_t>(hregs, HHI_GCLK_MPEG1));
    clr_bitsl( (1 << 3) | (1<<2) , offset_ptr<uint32_t>(hregs, HHI_MEM_PD_REG0));

//reset the phy
    gpio_write(&mac_device->gpio_, PHY_RESET, 0);
    zx_nanosleep(zx_deadline_after(ZX_MSEC(100)));
    gpio_write(&mac_device->gpio_, PHY_RESET, 1);
    zx_nanosleep(zx_deadline_after(ZX_MSEC(100)));

    //Enable GigE advertisement
    mac_device->MDIOWrite(MII_CTRL1000, 1<<9);

    //Restart advertisements
    uint32_t val;
    mac_device->MDIORead(MII_BMCR,&val);
    val |= BMCR_ANENABLE | BMCR_ANRESTART;
    val &= ~BMCR_ISOLATE;
    mac_device->MDIOWrite(MII_BMCR,val);

    // Save the mac address, the reset below will clear out this register
    uint32_t tempmachi = mac_device->dwmac_regs_->macaddr0hi;
    uint32_t tempmaclo = mac_device->dwmac_regs_->macaddr0lo;

    //Reset the dma peripheral
    //TODO - put a timeout on this
    mac_device->dwdma_regs_->busmode |= DMAMAC_SRST;
    while(mac_device->dwdma_regs_->busmode & DMAMAC_SRST) {}

    mac_device->dwmac_regs_->macaddr0hi = tempmachi;
    mac_device->dwmac_regs_->macaddr0lo = tempmaclo;

    status = mac_device->InitBuffers();
    if (status != ZX_OK) return status;

    mac_device->InitDevice();

    //mac_device->DumpRegisters();
    auto thunk = [](void* arg) -> int { return reinterpret_cast<AmlDWMacDevice*>(arg)->Thread(); };

    int ret = thrd_create_with_name(&mac_device->thread_, thunk,
                                    reinterpret_cast<void*>(mac_device.get()),
                                    "amlmac-thread");
    ZX_DEBUG_ASSERT(ret == thrd_success);

    status = mac_device->DdkAdd("AmLogic dwMac");
    if (status != ZX_OK) {
        zxlogf(ERROR, "aml-dwmac: Could not create eth device: %d\n", status);
    } else {
        zxlogf(INFO,"aml-dwmac: Added AmLogic dwMac device\n");
    }

    __UNUSED auto ptr = mac_device.release();
    return ZX_OK;
}

zx_status_t AmlDWMacDevice::InitBuffers() {


    fbl::RefPtr<fbl::VmarManager> vmar_mgr;

    constexpr size_t kDescSize = ROUNDUP(2 * kNumDesc * sizeof(dw_dmadescr),PAGE_SIZE);

    constexpr size_t kBufSize = 2 * kNumDesc * kTxnBufSize;

    txn_buffer_ = PinnedBuffer::Create(kBufSize, zx::bti(bti_), ZX_CACHE_POLICY_CACHED);
    desc_buffer_ = PinnedBuffer::Create(kDescSize, zx::bti(bti_), ZX_CACHE_POLICY_UNCACHED);


    tx_buffer_ = static_cast<uint8_t*>(txn_buffer_->GetBaseAddress());
    zx_cache_flush(tx_buffer_, kBufSize,  ZX_CACHE_FLUSH_DATA | ZX_CACHE_FLUSH_INVALIDATE);
    //rx buffer right after tx
    rx_buffer_ = &tx_buffer_[kBufSize / 2];


    tx_descriptors_ = static_cast<dw_dmadescr*>(desc_buffer_->GetBaseAddress());
    //rx descriptors right after tx
    rx_descriptors_ = &tx_descriptors_[kNumDesc];

    zx_paddr_t addy;

    // Initialize descriptors. Doing tx and rx all at once
    for (uint i = 0; i < kNumDesc; i++) {

        desc_buffer_->LookupPhys( ((i + 1) % kNumDesc) * sizeof(dw_dmadescr), &addy);
        tx_descriptors_[i].dmamac_next = static_cast<uint32_t>(addy);

        txn_buffer_->LookupPhys(i * kTxnBufSize, &addy);
        tx_descriptors_[i].dmamac_addr = static_cast<uint32_t>(addy);
        tx_descriptors_[i].txrx_status = 0;
        tx_descriptors_[i].dmamac_cntl = DESC_TXCTRL_TXCHAIN;

        desc_buffer_->LookupPhys((((i + 1) % kNumDesc) + kNumDesc) * sizeof(dw_dmadescr), &addy);
        rx_descriptors_[i].dmamac_next = static_cast<uint32_t>(addy);


        txn_buffer_->LookupPhys((i + kNumDesc) * kTxnBufSize, &addy);
        rx_descriptors_[i].dmamac_addr = static_cast<uint32_t>(addy);
        rx_descriptors_[i].dmamac_cntl =
                        (MAC_MAX_FRAME_SZ & DESC_RXCTRL_SIZE1MASK) | \
                         DESC_RXCTRL_RXCHAIN;

        rx_descriptors_[i].txrx_status = DESC_RXSTS_OWNBYDMA;
    }

    zx_cache_flush(tx_descriptors_, kDescSize, ZX_CACHE_FLUSH_DATA);

    desc_buffer_->LookupPhys(0, &addy);
    dwdma_regs_->txdesclistaddr = static_cast<uint32_t>(addy);
    desc_buffer_->LookupPhys(kNumDesc * sizeof(dw_dmadescr), &addy);
    dwdma_regs_->rxdesclistaddr = static_cast<uint32_t>(addy);
    return ZX_OK;
}

zx_handle_t AmlDWMacDevice::EthmacGetBti() {
    return bti_;
}

zx_status_t AmlDWMacDevice::MDIOWrite(uint32_t reg, uint32_t val){

    writel(val, &dwmac_regs_->miidata);

    uint32_t miiaddr = (mii_addr_ << MIIADDRSHIFT) |
                       (reg << MIIREGSHIFT) |
                       MII_WRITE;

    writel( miiaddr | MII_CLKRANGE_150_250M | MII_BUSY,
            &dwmac_regs_->miiaddr);

    zx_time_t deadline = zx_deadline_after(ZX_MSEC(3));
    while (zx_clock_get(ZX_CLOCK_MONOTONIC) < deadline) {
        if (!(readl(&dwmac_regs_->miiaddr) & MII_BUSY )) {
            return ZX_OK;
        }
        zx_nanosleep(zx_deadline_after(ZX_USEC(10)));
    }
    return ZX_ERR_TIMED_OUT;
}

zx_status_t AmlDWMacDevice::MDIORead(uint32_t reg, uint32_t* val){

    uint32_t miiaddr = (mii_addr_ << MIIADDRSHIFT) |
                       (reg << MIIREGSHIFT);

    writel(miiaddr | MII_CLKRANGE_150_250M | MII_BUSY, &dwmac_regs_->miiaddr);

    zx_time_t deadline = zx_deadline_after(ZX_MSEC(3));
    while (zx_clock_get(ZX_CLOCK_MONOTONIC) < deadline) {
        if (!(readl(&dwmac_regs_->miiaddr) & MII_BUSY )) {
            *val = readl(&dwmac_regs_->miidata);
            return ZX_OK;
        }
        zx_nanosleep(zx_deadline_after(ZX_USEC(10)));
    }
    return ZX_ERR_TIMED_OUT;
}


AmlDWMacDevice::AmlDWMacDevice(zx_device_t *device)
    : ddk::Device<AmlDWMacDevice, ddk::Unbindable>(device) {

}

AmlDWMacDevice::~AmlDWMacDevice(){
    io_buffer_release(&periph_regs_iobuff_);
    io_buffer_release(&hhi_regs_iobuff_);
    io_buffer_release(&dwmac_regs_iobuff_);
}

void AmlDWMacDevice::DdkRelease() {
    zxlogf(INFO,"AmLogic Ethmac release...\n");
    delete this;
}

void AmlDWMacDevice::DdkUnbind() {
    zxlogf(INFO,"AmLogic Ethmac DdkUnbind\n");
    //TODO - signal Thread to shutdown
    //fbl::AutoLock lock(&lock_);
}

zx_status_t AmlDWMacDevice::GetMAC(uint8_t* addr) {
    uint32_t hi = dwmac_regs_->macaddr0hi;
    uint32_t lo = dwmac_regs_->macaddr0lo;

    /* Extract the MAC address from the high and low words */
    addr[0] = (lo & 0xff);
    addr[1] = (lo >> 8) & 0xff;
    addr[2] = (lo >> 16) & 0xff;
    addr[3] = ((uint8_t)(lo >> 24) & 0xff);
    addr[4] = (hi & 0xff);
    addr[5] = (hi >> 8) & 0xff;
    return ZX_OK;
}

zx_status_t AmlDWMacDevice::EthmacQuery(uint32_t options, ethmac_info_t* info) {
    memset(info, 0, sizeof(*info));
    info->features = ETHMAC_FEATURE_DMA;
    info->mtu = 1500;
    GetMAC(info->mac);
    return ZX_OK;
}

void AmlDWMacDevice::EthmacStop() {
    zxlogf(INFO,"Stopping AmLogic Ethermac\n");
    fbl::AutoLock lock(&lock_);
    ethmac_proxy_.reset();
}

zx_status_t AmlDWMacDevice::EthmacStart(fbl::unique_ptr<ddk::EthmacIfcProxy> proxy) {
    fbl::AutoLock lock(&lock_);
    if (ethmac_proxy_ != nullptr) {
        zxlogf(ERROR,"aml_dwmac:  Already bound!!!");
        return ZX_ERR_ALREADY_BOUND;
    } else {
        ethmac_proxy_.swap(proxy);
        UpdateLinkStatus();
    }
    return ZX_OK;
}

zx_status_t AmlDWMacDevice::InitDevice() {

    dwdma_regs_->intenable = 0;
    dwdma_regs_->busmode = FIXEDBURST | PRIORXTX_41 | DMA_PBL;

    dwdma_regs_->opmode = DMA_OPMODE_TSF ;
    dwdma_regs_->opmode |= DMA_OPMODE_SR | DMA_OPMODE_ST; //start tx and rx

    //TODO - configure filters
    dwmac_regs_->framefilt |= (1 << 31); //pass all for now

    //Enable Interrupts
    dwdma_regs_->intenable = DMA_INT_NIE | DMA_INT_TIE |
                             DMA_INT_AIE | DMA_INT_FBE |
                             DMA_INT_RIE;

    uint32_t temp = dwmac_regs_->conf;
    temp |= GMAC_CORE_INIT | GMAC_CONF_TE | GMAC_CONF_RE;
    temp &= ~GMAC_CONF_PS;
    dwmac_regs_->conf = temp;

    return ZX_OK;
}


void AmlDWMacDevice::ProcRxBuffer(uint32_t int_status) {

    while (!(rx_descriptors_[curr_rx_buf_].txrx_status & DESC_RXSTS_OWNBYDMA)) {

        rx_descriptors_[curr_rx_buf_].dmamac_cntl =
                                (MAC_MAX_FRAME_SZ & DESC_RXCTRL_SIZE1MASK) | \
                                 DESC_RXCTRL_RXCHAIN;
        fbl::AutoLock lock(&lock_);
        if (ethmac_proxy_ != nullptr) {

            uint32_t pkt_stat = rx_descriptors_[curr_rx_buf_].txrx_status;

            size_t fr_len = (pkt_stat & DESC_RXSTS_FRMLENMSK) >> DESC_RXSTS_FRMLENSHFT;

            uint8_t* temptr = &rx_buffer_[curr_rx_buf_ * kTxnBufSize];

            ethmac_proxy_->Recv(temptr,fr_len - 4,0);
            memset(temptr, 0xbb, kTxnBufSize);
            zx_cache_flush(temptr, kTxnBufSize, ZX_CACHE_FLUSH_DATA | ZX_CACHE_FLUSH_INVALIDATE);
        }
        rx_descriptors_[curr_rx_buf_].txrx_status = DESC_RXSTS_OWNBYDMA;
        curr_rx_buf_ = (curr_rx_buf_ + 1)%kNumDesc;
    }
}

zx_status_t AmlDWMacDevice::EthmacQueueTx(uint32_t options, ethmac_netbuf_t* netbuf) {

    fbl::AutoLock lock(&lock_);
    if (!online_) {
        return ZX_ERR_UNAVAILABLE;
    }
    uint8_t* temptr = &tx_buffer_[curr_tx_buf_ * kTxnBufSize];

    memcpy(temptr, netbuf->data, netbuf->len);

    zx_cache_flush(temptr, netbuf->len, ZX_CACHE_FLUSH_DATA);

    //Descriptors are pre-iniitialized with the paddr of their corresponding
    // buffers
    tx_descriptors_[curr_tx_buf_].txrx_status = DESC_TXSTS_OWNBYDMA;
    tx_descriptors_[curr_tx_buf_].dmamac_cntl =
                                DESC_TXCTRL_TXINT |
                                DESC_TXCTRL_TXLAST |
                                DESC_TXCTRL_TXFIRST |
                                DESC_TXCTRL_TXCHAIN |
                                (netbuf->len & DESC_TXCTRL_SIZE1MASK);

    //TODO - prevent this from overwriting buffers in queue, but not xmitted yet.
    curr_tx_buf_ = (curr_tx_buf_ + 1) % kNumDesc;

    dwdma_regs_->txpolldemand = ~0;

    return ZX_OK;
}

zx_status_t AmlDWMacDevice::EthmacSetParam(uint32_t param, int32_t value, void* data) {

    return ZX_ERR_NOT_SUPPORTED;
}

} // namespace eth

extern "C" zx_status_t aml_eth_bind(void* ctx, zx_device_t* device, void** cookie) 
{
    return eth::AmlDWMacDevice::Create(device);
}