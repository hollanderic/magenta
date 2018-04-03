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
#include <pretty/hexdump.h>
#include <zircon/compiler.h>

#include <soc/aml-s912/s912-hw.h>


#include <stdio.h>
#include <string.h>

#include "aml-dwmac.h"
#include "dw-gmac-dma.h"

namespace eth {

enum {
    PHY_RESET,
    PHY_INTR,
};

int amlmac_device_thread(void* arg) {
    AmlDWMacDevice* device = reinterpret_cast<AmlDWMacDevice*>(arg);
    return device->Thread();
}
int AmlDWMacDevice::Thread() {
    zxlogf(INFO,"Device Thread Started -- (%p)%x\n",this,this->dma_irq_);

    while (true) {
        zx_status_t status;

        uint64_t slots;
        status = zx_interrupt_wait(dma_irq_, &slots);
        if (status != ZX_OK) {
            zxlogf(ERROR,"aml-dwmac: Interrupt error\n");
        }

        uint32_t stat = dwdma_regs_->status;
        if (stat & DMA_STATUS_GLI) {
            dwdma_regs_->status |= DMA_STATUS_GLI;
            online_ = dwmac_regs_->rgmiistatus & GMAC_RGMII_STATUS_LNKSTS;
            zxlogf(INFO,"amlmac: Link is now %s\n",online_ ? "up": "down");
        }
        if (stat & DMA_STATUS_RI) {
            zxlogf(INFO,"receive interrupt %08x\n",dwdma_regs_->currhostrxdesc);
            dwdma_regs_->status |= DMA_STATUS_RI;
            ProcRxBuffer();
        }
        if (stat & DMA_STATUS_AIS) {
            zxlogf(INFO,"abnormal interrupt\n");
        }

        dwdma_regs_->status |= ( DMA_INT_NIE | DMA_INT_TIE |
                             DMA_INT_AIE | DMA_INT_FBE |
                             DMA_INT_TUE | DMA_INT_TSE
                             );
    }

    return 0;
}




void AmlDWMacDevice::UpdateLinkStatus() {
    uint32_t rgstat = dwmac_regs_->rgmiistatus;
    online_ = rgstat & GMAC_RGMII_STATUS_LNKSTS;
}

zx_status_t AmlDWMacDevice::Create(zx_device_t* device){

    auto mac_device = fbl::unique_ptr<eth::AmlDWMacDevice>(
        new eth::AmlDWMacDevice(device));

    zx_status_t status = device_get_protocol(mac_device->parent_,
                                            ZX_PROTOCOL_PLATFORM_DEV,
                                            &mac_device->pdev_);
    if (status != ZX_OK) {
        return status;
    }

    status = mac_device->DdkAdd("AmLogic dwMac");
    if (status != ZX_OK) {
        zxlogf(ERROR, "aml-dwmac: Could not create eth device: %d\n", status);
    } else {
        zxlogf(INFO,"aml-dwmac: Added AmLogic dwMac device\n");
    }

    status = device_get_protocol(mac_device->parent_, ZX_PROTOCOL_GPIO, &mac_device->gpio_);
    if (status != ZX_OK) {
        return status;
    }

    gpio_config(&mac_device->gpio_, PHY_RESET, GPIO_DIR_OUT);

    status = pdev_map_mmio(&mac_device->pdev_, 0,
                            ZX_CACHE_POLICY_UNCACHED_DEVICE,
                            (void**)&mac_device->periph_regs_,
                            &mac_device->periph_regs_size_,
                            mac_device->periph_regs_vmo_.reset_and_get_address());
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not map periph mmio: %d\n",status);
        return status;
    }
    //void* reg_ptr;
    status = pdev_map_mmio(&mac_device->pdev_, 1,
                            ZX_CACHE_POLICY_UNCACHED_DEVICE,
                            (void**)&mac_device->dwmac_regs_,
                            &mac_device->dwmac_regs_size_,
                            mac_device->dwmac_regs_vmo_.reset_and_get_address());
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not map dwmac mmio: %d\n",status);
        return status;
    }
    //c_device->dwmac_regs_
    //dw_dma_regs_t* dmaptr = static_cast<dw_dma_regs_t*>(
    //                          (void*)mac_device->dwmac_regs_ + DW_DMA_BASE_OFFSET);
    mac_device->dwdma_regs_ = reinterpret_cast<dw_dma_regs_t*>(
                                  (zx_vaddr_t)mac_device->dwmac_regs_ + DW_DMA_BASE_OFFSET);

    status = pdev_map_mmio(&mac_device->pdev_, 2,
                            ZX_CACHE_POLICY_UNCACHED_DEVICE,
                            (void**)&mac_device->hhi_regs_,
                            &mac_device->hhi_regs_size_,
                            mac_device->hhi_regs_vmo_.reset_and_get_address());
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not map hiu mmio: %d\n",status);
        return status;
    }


    status = pdev_map_interrupt(&mac_device->pdev_, 0, &mac_device->dma_irq_);
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not map dma interrupt\n");
        return status;
    } else {
        printf("IRQ Handle = %x\n",mac_device->dma_irq_);
    }
    uint32_t tempmachi = mac_device->dwmac_regs_->macaddr0hi;
    uint32_t tempmaclo = mac_device->dwmac_regs_->macaddr0lo;


    printf("mac addr hi -> %08x\n",mac_device->dwmac_regs_->macaddr0hi);
    printf("mac addr lo -> %08x\n",mac_device->dwmac_regs_->macaddr0lo);

    status = pdev_get_bti(&mac_device->pdev_, 0, &mac_device->bti_);
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not obtain bti: %d\n",status);
        return status;
    }


#define BIT(n) (1 << n)
#define ETH_REG2_REVERSED BIT(28)
#define INTERNAL_PHY_ID 0x110181
#define PHY_ENABLE  BIT(31)
#define USE_PHY_IP  BIT(30)
#define CLK_IN_EN   BIT(29)
#define USE_PHY_MDI BIT(26)
#define LED_POLARITY  BIT(23)
#define ETH_REG3_19_RESVERD (0x9 << 16)
#define CFG_PHY_ADDR (0x8 << 8)
#define CFG_MODE (0x7 << 4)
#define CFG_EN_HIGH BIT(3)
#define ETH_REG3_2_RESERVED 0x7
    writel(0x1621, mac_device->periph_regs_ + PER_ETH_REG0);
    writel(0x20000, mac_device->periph_regs_ + PER_ETH_REG1);

    writel(ETH_REG2_REVERSED | INTERNAL_PHY_ID,
           mac_device->periph_regs_ + PER_ETH_REG2);
    writel(CLK_IN_EN | ETH_REG3_19_RESVERD  |
            CFG_PHY_ADDR | CFG_MODE | CFG_EN_HIGH |
            ETH_REG3_2_RESERVED, mac_device->periph_regs_ + PER_ETH_REG3);


    set_bitsl( 1 << 3, mac_device->hhi_regs_ + HHI_GCLK_MPEG1);
    clr_bitsl( (1 << 3) | (1<<2) , mac_device->hhi_regs_ +  HHI_MEM_PD_REG0);

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

    //Reset the dma peripheral
    //TODO - put a timeout on this
    mac_device->dwdma_regs_->busmode |= DMAMAC_SRST;
    while(mac_device->dwdma_regs_->busmode & DMAMAC_SRST) {}

    mac_device->dwmac_regs_->macaddr0hi = tempmachi;
    mac_device->dwmac_regs_->macaddr0lo = tempmaclo;

    printf("initing the buffers\n");
    status = mac_device->InitBuffers();
    if (status != ZX_OK) return status;

    mac_device->InitDevice();

    //mac_device->DumpRegisters();

    int ret = thrd_create_with_name(&mac_device->thread_, amlmac_device_thread,
                                    reinterpret_cast<void*>(mac_device.get()),
                                    "amlmac-thread");
    ZX_DEBUG_ASSERT(ret == thrd_success);

    __UNUSED auto ptr = mac_device.release();
    return ZX_OK;
}

zx_status_t AmlDWMacDevice::InitBuffers() {

    fbl::RefPtr<fbl::VmarManager> vmar_mgr;

    constexpr size_t kDescSize = ROUNDUP(2 * kNumDesc_ * sizeof(dw_dmadescr),PAGE_SIZE);
    //constexpr size_t kDescPages = ROUNDUP(kDescSize, PAGE_SIZE);

    constexpr size_t kBufSize = 2 * kNumDesc_ * kTxnBufSize_;

    //create vmar large enough for rx,tx buffers, and rx,tx dma descriptors
    vmar_mgr = fbl::VmarManager::Create(0x100000, nullptr);

    /* Descriptors are mapped uncached.  This is due to a quirk in the dwmac
        dma engine where status is saved in the descriptor.  This makes it
        problematic to check the state of a dma transaction via the descriptor
        while the dma may still be using the descriptor.
    */
    zx::vmo desc_vmo;
    zx_status_t status = dma_desc_mapper_.CreateAndMap(kDescSize,
                                ZX_VM_FLAG_PERM_READ | ZX_VM_FLAG_PERM_WRITE,
                                vmar_mgr,
                                &desc_vmo,
                                ZX_RIGHT_READ | ZX_RIGHT_MAP | ZX_RIGHT_WRITE);
                                //ZX_CACHE_POLICY_UNCACHED);
    if (status != ZX_OK) {
        zxlogf(ERROR,"descriptor buffer create failed %d\n",status);
        return status;
    }

    zx::vmo buff_vmo;
    status = dma_buff_mapper_.CreateAndMap(kBufSize,
                                ZX_VM_FLAG_PERM_READ | ZX_VM_FLAG_PERM_WRITE,
                                vmar_mgr,
                                &buff_vmo,
                                ZX_RIGHT_READ | ZX_RIGHT_MAP | ZX_RIGHT_WRITE);
    if (status != ZX_OK) {
        zxlogf(ERROR,"data buffer create failed %d\n",status);
        return status;
    }

    // Commit the buffer VMOs to allow access to the physical mapping info
    status = desc_vmo.op_range(ZX_VMO_OP_COMMIT, 0, kDescSize, nullptr, 0);
    if (status != ZX_OK) return status;

    status = buff_vmo.op_range(ZX_VMO_OP_COMMIT, 0, kBufSize, nullptr, 0);
    if (status != ZX_OK) return status;

    tx_buffer_ = static_cast<uint8_t*>(dma_buff_mapper_.start());

    //rx buffer right after tx
    rx_buffer_ = &tx_buffer_[kBufSize / 2];

    tx_descriptors_ = static_cast<dw_dmadescr*>(dma_desc_mapper_.start());

    //rx descriptors right after tx
    rx_descriptors_ = &tx_descriptors_[kNumDesc_];

    zx_paddr_t desc_phys[ROUNDUP(kDescSize, PAGE_SIZE) / PAGE_SIZE];
    status = desc_vmo.op_range(ZX_VMO_OP_LOOKUP, 0, kDescSize,
                                desc_phys, sizeof(desc_phys));
    if (status != ZX_OK) return status;


    zx_paddr_t buff_phys[ROUNDUP(kBufSize, PAGE_SIZE) / PAGE_SIZE];
    status = buff_vmo.op_range(ZX_VMO_OP_LOOKUP, 0, kBufSize,
                                buff_phys, sizeof(buff_phys));
    if (status != ZX_OK) return status;
    printf("lookups done\n");

    auto paddr = [](uint idx, size_t stride, zx_paddr_t* paddrs){
        return static_cast<uint32_t>(paddrs[ idx * stride / PAGE_SIZE] +
                                     ((idx * stride) % PAGE_SIZE));
    };
    printf("Initing the descriptors\n");
    // Initialize descriptors. Doing tx and rx all at once
    for (uint i = 0; i < kNumDesc_; i++) {

        tx_descriptors_[i].dmamac_next = paddr((i+1)%kNumDesc_,
                                               sizeof(dw_dmadescr),
                                               desc_phys);
        tx_descriptors_[i].dmamac_addr = paddr(i, kTxnBufSize_, buff_phys);
        tx_descriptors_[i].txrx_status = 0;
        tx_descriptors_[i].dmamac_cntl = DESC_TXCTRL_TXCHAIN;

        rx_descriptors_[i].dmamac_next = paddr((i+1)%kNumDesc_ + kNumDesc_,
                                               sizeof(dw_dmadescr),
                                               desc_phys);

        rx_descriptors_[i].dmamac_addr = paddr(i + kNumDesc_, kTxnBufSize_, buff_phys);
        rx_descriptors_[i].dmamac_cntl =
                        (MAC_MAX_FRAME_SZ & DESC_RXCTRL_SIZE1MASK) | \
                         DESC_RXCTRL_RXCHAIN;

        rx_descriptors_[i].txrx_status = DESC_RXSTS_OWNBYDMA;
    }

    //zx_cache_flush(tx_descriptors_, kDescSize, ZX_CACHE_FLUSH_DATA);

    dwdma_regs_->txdesclistaddr = paddr(0, sizeof(dw_dmadescr),desc_phys);
    dwdma_regs_->rxdesclistaddr = paddr(kNumDesc_, sizeof(dw_dmadescr),desc_phys);
    printf("buffers are done\n");
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

void AmlDWMacDevice::DdkRelease() {
    printf("DdkRelease\n");
    // Only the thread can call DdkRemove(), which means the thread is exiting on its own. No need
    // to join the thread.
    delete this;
}

void AmlDWMacDevice::DdkUnbind() {
    printf("DdkUnbind\n");
    //fbl::AutoLock lock(&lock_);
    //zx_status_t status = data_.signal(0, TAP_SHUTDOWN);
    //ZX_DEBUG_ASSERT(status == ZX_OK);
    // When the thread exits after the channel is closed, it will call DdkRemove.
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
    uint8_t mac[6] = {};
    GetMAC(mac);
    memcpy(info->mac, mac, 6);
    return ZX_OK;
}

void AmlDWMacDevice::EthmacStop() {
    //ethertap_trace("EthmacStop\n");
    //fbl::AutoLock lock(&lock_);
    ethmac_proxy_.reset();
}

zx_status_t AmlDWMacDevice::EthmacStart(fbl::unique_ptr<ddk::EthmacIfcProxy> proxy) {
    fbl::AutoLock lock(&lock_);
    if (ethmac_proxy_ != nullptr) {
        zxlogf(ERROR,"Already bound!!!");
        return ZX_ERR_ALREADY_BOUND;
    } else {
        ethmac_proxy_.swap(proxy);
        ethmac_proxy_->Status(online_ ? ETH_STATUS_ONLINE : 0u);
        printf("EthmacStart - we are %s\n",online_? "online":"offline");

    }
    return ZX_OK;
}

zx_status_t AmlDWMacDevice::InitDevice() {

    dwdma_regs_->intenable = 0;
    dwdma_regs_->busmode = FIXEDBURST | PRIORXTX_41 | DMA_PBL;

    dwdma_regs_->opmode = STOREFORWARD;
    dwdma_regs_->opmode |= TXSTART | RXSTART;

    dwmac_regs_->framefilt |= (1 << 31); //pass all for now

    //Enable Interrupts
    dwdma_regs_->intenable = DMA_INT_NIE | DMA_INT_TIE |
                             DMA_INT_AIE | DMA_INT_FBE |
                             DMA_INT_TUE | DMA_INT_TSE |
                             DMA_INT_RIE;

    uint32_t temp = dwmac_regs_->conf;
    printf("Starting GMAC conf = %08x\n",temp);
    temp |= GMAC_CORE_INIT | GMAC_CONF_TE | GMAC_CONF_RE;
    temp &= ~GMAC_CONF_PS;

    dwmac_regs_->conf = temp;
    printf("ending GMAC conf = %08x\n",dwmac_regs_->conf);

    return ZX_OK;
}


void AmlDWMacDevice::ProcRxBuffer() {

    while (!(rx_descriptors_[curr_rx_buf_].txrx_status & DESC_RXSTS_OWNBYDMA)) {
        printf("rx status %u - %08x\n",curr_rx_buf_,rx_descriptors_[curr_rx_buf_].txrx_status);
        printf("rx cntl %08x\n",rx_descriptors_[curr_rx_buf_].dmamac_cntl);
        rx_descriptors_[curr_rx_buf_].dmamac_cntl =
                                (MAC_MAX_FRAME_SZ & DESC_RXCTRL_SIZE1MASK) | \
                                 DESC_RXCTRL_RXCHAIN;
        rx_descriptors_[curr_rx_buf_].txrx_status = DESC_RXSTS_OWNBYDMA;

        curr_rx_buf_ = (curr_rx_buf_ + 1)%kNumDesc_;
    }



}

zx_status_t AmlDWMacDevice::EthmacQueueTx(uint32_t options, ethmac_netbuf_t* netbuf) {
#if 0
    zxlogf(ERROR,"txbuff index = %2u   %08x  %08x %08x\n",curr_tx_buf_,
                                                    tx_descriptors_[curr_tx_buf_].dmamac_addr,
                                                    dwdma_regs_->currhosttxdesc,
                                                    dwdma_regs_->currhosttxbuffaddr);
    zxlogf(ERROR, "tx_len=%4d  rgmii stat= %08x\n", netbuf->len,dwmac_regs_->rgmiistatus);
#endif
    fbl::AutoLock lock(&lock_);

    if (!online_) {
        return ZX_ERR_UNAVAILABLE;
    }

    uint8_t* temptr = &tx_buffer_[curr_tx_buf_ * kTxnBufSize_];

    memcpy(temptr, netbuf->data, netbuf->len);

    zx_cache_flush(temptr, netbuf->len, ZX_CACHE_FLUSH_DATA);

    //Descriptors are pre-iniitialized with the paddr of their corresponding
    // buffers
    tx_descriptors_[curr_tx_buf_].dmamac_cntl =
                                DESC_TXCTRL_TXINT |
                                DESC_TXCTRL_TXLAST |
                                DESC_TXCTRL_TXFIRST |
                                DESC_TXCTRL_TXCHAIN |
                                DESC_TXCTRL_TXCHECKINSCTRL |
                                (netbuf->len & DESC_TXCTRL_SIZE1MASK);

    tx_descriptors_[curr_tx_buf_].txrx_status = DESC_TXSTS_OWNBYDMA;
    //zx_cache_flush(&tx_descriptors_[curr_tx_buf_], kTxnBufSize_, ZX_CACHE_FLUSH_DATA);

    //TODO - prevent this from overwriting buffers in queue, but not xmitted yet.
    curr_tx_buf_ = (curr_tx_buf_ + 1) % kNumDesc_;

    dwdma_regs_->txpolldemand = ~0;

    return ZX_OK;
}

zx_status_t AmlDWMacDevice::EthmacSetParam(uint32_t param, int32_t value, void* data) {

    return ZX_ERR_NOT_SUPPORTED;
}

} // namespace eth

extern "C" zx_status_t aml_eth_bind(void* ctx, zx_device_t* device, void** cookie) {
    return eth::AmlDWMacDevice::Create(device);
}