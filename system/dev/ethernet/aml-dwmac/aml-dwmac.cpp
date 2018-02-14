// Copyright 2017 The Fuchsia Authors. All rights reserved.
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
namespace eth {

enum {
    MAC_RESET,
    MAC_INTR,
};

//static
zx_status_t AmlDWMacDevice::Create(zx_device_t* device){

    auto mac_device = fbl::unique_ptr<eth::AmlDWMacDevice>(
        new eth::AmlDWMacDevice(device));

    zx_status_t status = mac_device->InitBuffers();
    if (status != ZX_OK) return status;


    status = device_get_protocol(mac_device->parent_,
                                            ZX_PROTOCOL_PLATFORM_DEV,
                                            &mac_device->pdev_);
    if (status != ZX_OK) {
        return status;
    }

    status = mac_device->DdkAdd("AmLogic dwMac");
    if (status != ZX_OK) {
        zxlogf(ERROR, "aml-dwmac: Could not create eth device: %d\n", status);
    } else {
        //mac_device.release(); //release the reference so object persists
        zxlogf(INFO,"aml-dwmac: Added AmLogic dwMac device\n");
    }

    status = device_get_protocol(mac_device->parent_, ZX_PROTOCOL_GPIO, &mac_device->gpio_);
    if (status != ZX_OK) {
        return status;
    }

    gpio_config(&mac_device->gpio_, MAC_RESET, GPIO_DIR_OUT);

    status = pdev_map_mmio(&mac_device->pdev_, 0,
                            ZX_CACHE_POLICY_UNCACHED_DEVICE,
                            (void**)&mac_device->periph_regs_,
                            &mac_device->periph_regs_size_,
                            mac_device->periph_regs_vmo_.reset_and_get_address());
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not map periph mmio: %d\n",status);
        return status;
    }
    status = pdev_map_mmio(&mac_device->pdev_, 1,
                            ZX_CACHE_POLICY_UNCACHED_DEVICE,
                            (void**)&mac_device->dwmac_regs_,
                            &mac_device->dwmac_regs_size_,
                            mac_device->dwmac_regs_vmo_.reset_and_get_address());
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not map dwmac mmio: %d\n",status);
        return status;
    }


    status = pdev_map_mmio(&mac_device->pdev_, 2,
                            ZX_CACHE_POLICY_UNCACHED_DEVICE,
                            (void**)&mac_device->hhi_regs_,
                            &mac_device->hhi_regs_size_,
                            mac_device->hhi_regs_vmo_.reset_and_get_address());
    if (status != ZX_OK) {
        zxlogf(ERROR,"aml-dwmac: could not map hiu mmio: %d\n",status);
        return status;
    }

    writel( ETH_REG0_RGMII_SEL |
            (1 << ETH_REG0_TX_CLK_PH_POS) |
            (4 << ETH_REG0_TX_CLK_RATIO_POS) |
            ETH_REG0_REF_CLK_ENA |
            ETH_REG0_CLK_ENA,
            mac_device->periph_regs_ + PER_ETH_REG0);

    set_bitsl( 1 << 3, mac_device->hhi_regs_ + HHI_GCLK_MPEG1);
    clr_bitsl( (1 << 3) | (1<<2) , mac_device->hhi_regs_ +  HHI_MEM_PD_REG0);

    gpio_write(&mac_device->gpio_, MAC_RESET, 0);
    zx_nanosleep(zx_deadline_after(ZX_MSEC(10)));
    gpio_write(&mac_device->gpio_, MAC_RESET, 1);
    zx_nanosleep(zx_deadline_after(ZX_MSEC(10)));

#if 1
#define PREG(offs)  (*(uint32_t*)(mac_device->periph_regs_ + offs))
    printf("PER_ETH_REG0 %08x\n",PREG(PER_ETH_REG0));
    printf("PER_ETH_REG1 %08x\n",PREG(PER_ETH_REG1));
    printf("PER_ETH_REG2 %08x\n",PREG(PER_ETH_REG2));
    printf("PER_ETH_REG3 %08x\n",PREG(PER_ETH_REG3));
    printf("PER_ETH_REG4 %08x\n",PREG(PER_ETH_REG4));
    uint32_t val;
    for (uint32_t i=0; i<31; i++) {
        if (mac_device->MDIORead(i,&val)==ZX_OK) {
            printf("MII%02u = %04x\n",i,val);
        } else {
            printf("MDIO READ TIMEOUT%u\n",i);
        }
    }

#undef PREG
#endif
    __UNUSED auto ptr = mac_device.release();
    return ZX_OK;
}

zx_status_t AmlDWMacDevice::InitBuffers() {
    fbl::RefPtr<fbl::VmarManager> vmar_mgr;
    //create vmar large enough for rx,tx buffers, and rx,tx dma descriptors
    size_t desc_size = ROUNDUP(2 * sizeof(dw_dmadescr) * num_descriptors_, PAGE_SIZE);
    size_t buff_size = ROUNDUP(2 * num_descriptors_ * txn_buffer_size_, PAGE_SIZE);

    vmar_mgr = fbl::VmarManager::Create(desc_size +  buff_size, nullptr);

    zx::vmo desc_vmo;
    zx_status_t status = dma_desc_mapper_.CreateAndMap(desc_size,
                                ZX_VM_FLAG_PERM_READ | ZX_VM_FLAG_PERM_WRITE,
                                vmar_mgr,
                                &desc_vmo,
                                ZX_RIGHT_READ | ZX_RIGHT_MAP | ZX_RIGHT_WRITE);
    if (status != ZX_OK) return status;

    zx::vmo buff_vmo;
    status = dma_buff_mapper_.CreateAndMap(buff_size,
                                ZX_VM_FLAG_PERM_READ | ZX_VM_FLAG_PERM_WRITE,
                                vmar_mgr,
                                &buff_vmo,
                                ZX_RIGHT_READ | ZX_RIGHT_MAP | ZX_RIGHT_WRITE);
    if (status != ZX_OK) return status;

    tx_buffer_ = static_cast<uint8_t*>(dma_buff_mapper_.start());
    //rx buffer right after tx
    rx_buffer_ = &tx_buffer_[txn_buffer_size_ * num_descriptors_];

    tx_descriptors_ = static_cast<dw_dmadescr*>(dma_desc_mapper_.start());
    //rx descriptors right after tx
    rx_descriptors_ = &tx_descriptors_[num_descriptors_];

    TxDescInit(&desc_vmo, &buff_vmo);



    return status;
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

zx_status_t AmlDWMacDevice::TxDescInit(zx::vmo* desc, zx::vmo* buff) {
    for (uint32_t i = 0; i< num_descriptors_; i++) {
        //tx_descriptors_[i].dmamac_addr = &tx_buffer_[i * txn_buffer_size_];
    }
    return ZX_OK;
}
#if 0
static void tx_descs_init(struct eth_device *dev)
{
    struct dw_eth_dev *priv = dev->priv;
    struct eth_dma_regs *dma_p = priv->dma_regs_p;
    struct dmamacdescr *desc_table_p = &priv->tx_mac_descrtable[0];
    char *txbuffs = &priv->txbuffs[0];
    struct dmamacdescr *desc_p;
    u32 idx;

    for (idx = 0; idx < CONFIG_TX_DESCR_NUM; idx++) {
        desc_p = &desc_table_p[idx];
        desc_p->dmamac_addr = (u32)(phys_addr_t)&txbuffs[idx * CONFIG_ETH_BUFSIZE];
        desc_p->dmamac_next = (u32)(phys_addr_t)&desc_table_p[idx + 1];


        desc_p->dmamac_cntl = DESC_TXCTRL_TXCHAIN;
        desc_p->txrx_status = 0;
    }

    /* Correcting the last pointer of the chain */
    desc_p->dmamac_next = (u32)(phys_addr_t)&desc_table_p[0];

    /* Flush all Tx buffer descriptors at once */
    flush_dcache_range((phys_addr_t)priv->tx_mac_descrtable,
               (phys_addr_t)priv->tx_mac_descrtable +
               sizeof(priv->tx_mac_descrtable));

    writel((ulong)&desc_table_p[0], &dma_p->txdesclistaddr);
    priv->tx_currdescnum = 0;
}
#endif

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

zx_status_t AmlDWMacDevice::EthmacQuery(uint32_t options, ethmac_info_t* info) {
    memset(info, 0, sizeof(*info));
    info->features = 69;
    info->mtu = 100;
    uint8_t mac[6] = {0xde, 0xad, 0xbe, 0xef, 0xfa, 0xce};
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


zx_status_t AmlDWMacDevice::EthmacQueueTx(uint32_t options, ethmac_netbuf_t* netbuf) {


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
    return ZX_ERR_NOT_SUPPORTED;
}



} // namespace eth

extern "C" zx_status_t aml_eth_bind(void* ctx, zx_device_t* device, void** cookie) {
    return eth::AmlDWMacDevice::Create(device);
}