// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/debug.h>
#include <ddk/protocol/platform-device.h>
#include <fbl/auto_call.h>
#include <fbl/auto_lock.h>
#include <fbl/ref_counted.h>
#include <fbl/ref_ptr.h>
#include <fbl/type_support.h>
#include <hw/arch_ops.h>
#include <hw/reg.h>
#include <zircon/compiler.h>

#include <stdio.h>
#include <string.h>

#include "ft3x27.h"

namespace ft{
Ft3x27Device::Ft3x27Device(zx_device_t* device)
    : ddk::Device<Ft3x27Device, ddk::Unbindable>(device) {
}
int Ft3x27Device::TestThread() {
    while (true) {
        zx_nanosleep(zx_deadline_after(ZX_MSEC(100)));
        zxlogf(INFO,"%02x\n",Read(0x02));
        //zxlogf(INFO,"Gesture = %02x\n",i2c_read(i2c,0x01));
        //zxlogf(INFO,"CurPoints = %2u\n",i2c_read(i2c,0x02));
        //zxlogf(INFO,"....\n");
    }
    return 0;

}

int Ft3x27Device::Thread() {
    zx_status_t status;
    zxlogf(INFO,"Entering ft3x27 thread\n");
    while(true) {
        status = irq_.wait(nullptr);
        if (!running_.load()) {
            return ZX_OK;
        }
        if (status != ZX_OK) {
            zxlogf(ERROR,"ft3x27: Interrupt error %d\n",status);
            return status;
        }
        zxlogf(INFO,"Touch interrupt\n");
    }
    zxlogf(INFO,"Exiting ft3x27 thread\n");

}

zx_status_t Ft3x27Device::InitPdev() {
    zx_status_t status = device_get_protocol(parent_,
                                             ZX_PROTOCOL_PLATFORM_DEV,
                                             &pdev_);
    if (status != ZX_OK) {
        return status;
    }

   status = device_get_protocol(parent_, ZX_PROTOCOL_I2C, &i2c_);
    if ( status != ZX_OK) {
        zxlogf(ERROR,"ft3x27: failed to acquire i2c\n");
        return status;
    }

    status = device_get_protocol(parent_, ZX_PROTOCOL_GPIO, &gpio_);
    if (status != ZX_OK) {
        return status;
    }

    gpio_config(&gpio_, FT_INT_PIN, GPIO_DIR_IN);
    //gpio_write(&gpio_, PHY_RESET, 0);

    status = gpio_get_interrupt(&gpio_, FT_INT_PIN,
                       ZX_INTERRUPT_MODE_EDGE_LOW,
                       irq_.reset_and_get_address());
    if (status != ZX_OK) {
        return status;
    }

    return ZX_OK;
}

zx_status_t Ft3x27Device::Create(zx_device_t* device) {

    zxlogf(INFO,"ft3x27: driver started...\n");

    auto ft_dev = fbl::make_unique<Ft3x27Device>(device);

    zx_status_t status = ft_dev->InitPdev();
    if (status != ZX_OK) {
        zxlogf(ERROR,"ft3x27: Driver bind failed %d\n",status);
        return status;
    }

    for(uint8_t i=0; i<32; i++) {
        zxlogf(INFO,"REG 0x%02x = %02x\n",i,ft_dev->Read(i));
    }

    auto thunk = [](void* arg) -> int {
        return reinterpret_cast<Ft3x27Device*>(arg)->Thread();
    };

//    auto thunk2 = [](void* arg) -> int {
//        return reinterpret_cast<Ft3x27Device*>(arg)->TestThread();
//    };
    ft_dev->running_.store(true);
    int ret = thrd_create_with_name(&ft_dev->thread_, thunk,
                                    reinterpret_cast<void*>(ft_dev.get()),
                                    "ft3x27-thread");
    ZX_DEBUG_ASSERT(ret == thrd_success);
//    ret = thrd_create_with_name(&ft_dev->test_thread_, thunk2,
//                                    reinterpret_cast<void*>(ft_dev.get()),
//                                    "ft3x27-thread");

   // device intentionally leaked as it is now held by DevMgr
    __UNUSED auto ptr = ft_dev.release();

    return ZX_OK;
}

zx_status_t Ft3x27Device::HidBusQuery(uint32_t options, hid_info_t* info){
    return ZX_OK;
}

void Ft3x27Device::DdkRelease() {
    delete this;
}

void Ft3x27Device::DdkUnbind() {
    DdkRemove();
}


zx_status_t Ft3x27Device::HidBusGetDescriptor(uint8_t desc_type, void** data, size_t* len) {
    return ZX_OK;
}
zx_status_t Ft3x27Device::HidBusGetReport(uint8_t rpt_type, uint8_t rpt_id, void* data,
                                          size_t len, size_t* out_len){
    return ZX_OK;
}

zx_status_t Ft3x27Device::HidBusSetReport(uint8_t rpt_type, uint8_t rpt_id, void* data,
                                          size_t len){
    return ZX_OK;
}
zx_status_t Ft3x27Device::HidBusGetIdle(uint8_t rpt_id, uint8_t* duration) {
    return ZX_OK;
}
zx_status_t Ft3x27Device::HidBusSetIdle(uint8_t rpt_id, uint8_t duration) {
    return ZX_OK;
}


zx_status_t Ft3x27Device::HidBusGetProtocol(uint8_t* protocol) {
    return ZX_OK;
}
zx_status_t Ft3x27Device::HidBusSetProtocol(uint8_t protocol) {
    return ZX_OK;
}

void Ft3x27Device::HidBusStop() {
    fbl::AutoLock lock(&lock_);
    proxy_.clear();
}

zx_status_t Ft3x27Device::HidBusStart(ddk::HidBusIfcProxy proxy) {
    fbl::AutoLock lock(&lock_);
    if (proxy_.is_valid()) {
        zxlogf(ERROR,"ft3x27: Already bound!\n");
        return ZX_ERR_ALREADY_BOUND;
    } else {
        proxy_ = proxy;
        zxlogf(INFO,"ft3x27: started\n");
    }
    return ZX_OK;
}


uint8_t Ft3x27Device::Read(uint8_t addr) {
    uint8_t rbuf;
    i2c_transact_sync(&i2c_,kI2cIndex,&addr,1,&rbuf,1);
    return rbuf;
}

} //namespace ft


extern "C" zx_status_t ft3x27_bind(void* ctx, zx_device_t* device, void** cookie) {
    return ft::Ft3x27Device::Create(device);
}