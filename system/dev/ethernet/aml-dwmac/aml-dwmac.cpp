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
        printf("Got an interrupt\n");
        if (status != ZX_OK) {
            zxlogf(ERROR,"aml-dwmac: Interrupt error\n");
        }
    }

    return 0;
}

//static
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


    printf("mac addr hi -> %08x\n",mac_device->dwmac_regs_->macaddr0hi);
    printf("mac addr lo -> %08x\n",mac_device->dwmac_regs_->macaddr0lo);

    status = pdev_get_bti(&mac_device->pdev_, 0, &mac_device->bti_);
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not obtain bti: %d\n",status);
        return status;
    }

    status = mac_device->InitBuffers();
    if (status != ZX_OK) return status;

    // Set up AmLogic system registers associated with the dwmac
    writel( ETH_REG0_RGMII_SEL |
            (1 << ETH_REG0_TX_CLK_PH_POS) |
            (4 << ETH_REG0_TX_CLK_RATIO_POS) |
            ETH_REG0_REF_CLK_ENA |
            ETH_REG0_CLK_ENA,
            mac_device->periph_regs_ + PER_ETH_REG0);

    set_bitsl( 1 << 3, mac_device->hhi_regs_ + HHI_GCLK_MPEG1);
    clr_bitsl( (1 << 3) | (1<<2) , mac_device->hhi_regs_ +  HHI_MEM_PD_REG0);

    gpio_write(&mac_device->gpio_, PHY_RESET, 0);
    zx_nanosleep(zx_deadline_after(ZX_MSEC(10)));
    gpio_write(&mac_device->gpio_, PHY_RESET, 1);
    zx_nanosleep(zx_deadline_after(ZX_MSEC(10)));

    mac_device->InitDevice();
    mac_device->DumpRegisters();

    printf("Pointer to mac device  = %p\n",&mac_device);

    int ret = thrd_create_with_name(&mac_device->thread_, amlmac_device_thread,
                                    reinterpret_cast<void*>(mac_device.get()),
                                    "amlmac-thread");
    ZX_DEBUG_ASSERT(ret == thrd_success);

    __UNUSED auto ptr = mac_device.release();
    return ZX_OK;
}

zx_status_t AmlDWMacDevice::InitBuffers() {

    fbl::RefPtr<fbl::VmarManager> vmar_mgr;

    constexpr size_t kDescSize = 2 * kNumDesc_ * sizeof(dw_dmadescr);
    //constexpr size_t kDescPages = ROUNDUP(kDescSize, PAGE_SIZE);

    constexpr size_t kBufSize = 2 * kNumDesc_ * kTxnBufSize_;

    //create vmar large enough for rx,tx buffers, and rx,tx dma descriptors
    vmar_mgr = fbl::VmarManager::Create(0x100000, nullptr);

    zx::vmo desc_vmo;
    zx_status_t status = dma_desc_mapper_.CreateAndMap(kDescSize,
                                ZX_VM_FLAG_PERM_READ | ZX_VM_FLAG_PERM_WRITE,
                                vmar_mgr,
                                &desc_vmo,
                                ZX_RIGHT_READ | ZX_RIGHT_MAP | ZX_RIGHT_WRITE);
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
    memset(tx_descriptors_, 0, kDescSize);
    zx_cache_flush(tx_descriptors_, kDescSize, ZX_CACHE_FLUSH_DATA);

    zx_paddr_t desc_phys[ROUNDUP(kDescSize, PAGE_SIZE) / PAGE_SIZE];
    status = desc_vmo.op_range(ZX_VMO_OP_LOOKUP, 0, kDescSize,
                                desc_phys, sizeof(desc_phys));
    if (status != ZX_OK) return status;


    zx_paddr_t buff_phys[ROUNDUP(kBufSize, PAGE_SIZE) / PAGE_SIZE];
    status = buff_vmo.op_range(ZX_VMO_OP_LOOKUP, 0, kBufSize,
                                buff_phys, sizeof(buff_phys));
    if (status != ZX_OK) return status;


    auto paddr = [](uint idx, size_t stride, zx_paddr_t* paddrs){
        return static_cast<uint32_t>(paddrs[ idx * stride / PAGE_SIZE] +
                                     ((idx * stride) % PAGE_SIZE));
    };

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

    zx_cache_flush(tx_descriptors_, kDescSize, ZX_CACHE_FLUSH_DATA);

    dwdma_regs_->txdesclistaddr = paddr(0, sizeof(dw_dmadescr),desc_phys);
    dwdma_regs_->rxdesclistaddr = paddr(kNumDesc_, sizeof(dw_dmadescr),desc_phys);

    return ZX_OK;
}


zx_handle_t AmlDWMacDevice::EthmacGetBti() {
    printf("chucking back bti=%x\n",bti_);
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

    printf("Constructed AmlDWMacDevice\n");
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
    printf("EthmacStart\n");
    fbl::AutoLock lock(&lock_);
    if (ethmac_proxy_ != nullptr) {
        return ZX_ERR_ALREADY_BOUND;
    } else {
        ethmac_proxy_.swap(proxy);
        ethmac_proxy_->Status(online_ ? ETH_STATUS_ONLINE : 0u);
    }
    return ZX_OK;
}

zx_status_t AmlDWMacDevice::InitDevice() {


    dwmac_regs_->conf |= (1 << 3);
    dwdma_regs_->intenable = DMA_INT_NIE | DMA_INT_TIE |
                             DMA_INT_AIE | DMA_INT_FBE |
                             DMA_INT_TUE | DMA_INT_TSE;
    dwdma_regs_->opmode |= (1 << 13);

    return ZX_OK;
}

zx_status_t AmlDWMacDevice::EthmacQueueTx(uint32_t options, ethmac_netbuf_t* netbuf) {
#if 1
    printf("txbuff index = %u   %08x\n",curr_tx_buf_,tx_descriptors_[curr_tx_buf_].dmamac_addr);

    uint8_t* temptr = &tx_buffer_[curr_tx_buf_ * kTxnBufSize_];

    memcpy(temptr, netbuf->data, netbuf->len);

    zx_cache_flush(temptr, netbuf->len, ZX_CACHE_FLUSH_DATA);
    //Descriptors are pre-iniitialized with the paddr of their corresponding
    // buffers
    tx_descriptors_[curr_tx_buf_].dmamac_cntl =
                                DESC_TXCTRL_TXINT |
                                DESC_TXCTRL_TXLAST |
                                DESC_TXCTRL_TXFIRST |
                                (netbuf->len & DESC_TXCTRL_SIZE1MASK);

    tx_descriptors_[curr_tx_buf_].txrx_status = DESC_TXSTS_OWNBYDMA;
    zx_cache_flush(&tx_descriptors_[curr_tx_buf_], kTxnBufSize_, ZX_CACHE_FLUSH_DATA);

    curr_tx_buf_ = (curr_tx_buf_ + 1) % kNumDesc_;

    //Start the transmission
    dwdma_regs_->txpolldemand = ~0;
    dwdma_regs_->opmode |= (1 << 13);

#endif
#if 0
    uint8_t temp_buf[ETHERTAP_MAX_MTU + sizeof(ethertap_socket_header_t)];
    auto header = reinterpret_cast<ethertap_socket_header*>(temp_buf);
    uint8_t* data = temp_buf + sizeof(ethertap_socket_header_t);
    size_t length = netbuf->len;
    ZX_DEBUG_ASSERT(length <= mtu_);
    memcpy(data, netbuf->data, length);
    header->type = ETHERTAP_MSG_PACKET;

    if (unlikely(options_ & ETHERTAP_OPT_TRACE_PACKETS)) {
        fbl::AutoLock lock(&lock_);
        ethertap_trace("sending %zu bytes\n", length);
        hexdump8_ex(data, length, 0);
    }
    zx_status_t status = data_.write(0u, temp_buf, length + sizeof(ethertap_socket_header_t),
                                     nullptr);
    if (status != ZX_OK) {
        zxlogf(ERROR, "ethertap: EthmacQueueTx error writing: %d\n", status);
    }
    // returning ZX_ERR_SHOULD_WAIT indicates that we will call complete_tx(), which we will not
    return status == ZX_ERR_SHOULD_WAIT ? ZX_ERR_UNAVAILABLE : status;
#endif
    return ZX_OK;
}

zx_status_t AmlDWMacDevice::EthmacSetParam(uint32_t param, int32_t value, void* data) {
#if 0
    if (!(options_ & ETHERTAP_OPT_REPORT_PARAM)) {
        return ZX_ERR_NOT_SUPPORTED;
    }

    struct {
        ethertap_socket_header_t header;
        ethertap_setparam_report_t report;
    } send_buf = {};

    send_buf.header.type = ETHERTAP_MSG_PARAM_REPORT;
    send_buf.report.param = param;
    send_buf.report.value = value;
    send_buf.report.data_length = 0;
    zx_status_t status = data_.write(0, &send_buf, sizeof(send_buf), nullptr);
    if (status != ZX_OK) {
        ethertap_trace("error writing SetParam info to socket: %d\n", status);
    }
    // A failure of data_.write is not a simulated failure of hardware under test, so log it but
    // don't report failure on the SetParam attempt.
#endif
    printf("Tried to set param\n");
    return ZX_ERR_NOT_SUPPORTED;
}



} // namespace eth

extern "C" zx_status_t aml_eth_bind(void* ctx, zx_device_t* device, void** cookie) {
    return eth::AmlDWMacDevice::Create(device);
}