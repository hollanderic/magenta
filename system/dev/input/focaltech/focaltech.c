// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <ddk/binding.h>
#include <ddk/debug.h>
#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/protocol/i2c.h>
#include <ddk/protocol/platform-device.h>
#include <ddk/protocol/platform-defs.h>


static uint8_t i2c_read(i2c_protocol_t* i2c, uint8_t addr) {
    uint8_t rbuf[4];
    uint8_t tbuf[2];
    tbuf[0] = addr;
    zx_status_t res = i2c_transact_sync(i2c,0,tbuf,1,rbuf,1);
    if (res) {
        zxlogf(ERROR,"i2c error %d\n",res);
        return 0;
    } else {
        return rbuf[0];
    }
}

static uint32_t getpos(i2c_protocol_t* i2c, uint8_t idx) {
    uint8_t rbuf[6];
    uint8_t tbuf[2];
    tbuf[0] = idx*6+3;
    zx_status_t res = i2c_transact_sync(i2c,0,tbuf,1,rbuf,6);

    if (res) {
        zxlogf(ERROR,"i2c error %d\n",res);
        return 0;
    } else {
        uint32_t x = ((rbuf[0] & 0x0f) << 8) + rbuf[1];
        uint32_t y = ((rbuf[2] & 0x0f) << 8) + rbuf[3];
        zxlogf(INFO,"(%4u,%4u) %3u %3u %1u %02x\n",x,y,rbuf[4], rbuf[5]>>4,
                                          rbuf[0]>>6,rbuf[2]>>4);
        return x;
    }
}

static int test_thread(void* ctx) {
    i2c_protocol_t* i2c = (i2c_protocol_t*)ctx;

    while (true) {
        zx_nanosleep(zx_deadline_after(ZX_MSEC(100)));
        getpos(i2c,0);
        //zxlogf(INFO,"Gesture = %02x\n",i2c_read(i2c,0x01));
        //zxlogf(INFO,"CurPoints = %2u\n",i2c_read(i2c,0x02));
        //zxlogf(INFO,"....\n");
    }
    return 0;
}

static zx_status_t focaltech_bind(void* ctx, zx_device_t* parent) {

    zxlogf(INFO, "focaltech_bind\n");

    platform_device_protocol_t pdev;
    zx_status_t res = device_get_protocol(parent, ZX_PROTOCOL_PLATFORM_DEV, &pdev);
    if (res != ZX_OK) {
        return res;
    }

    i2c_protocol_t* i2c = calloc(1, sizeof(i2c_protocol_t));
    res = device_get_protocol(parent, ZX_PROTOCOL_I2C, i2c);
    if ( res != ZX_OK) {
        zxlogf(ERROR,"focaltech-touch: failed to acquire i2c\n");
        return res;
    }
/*
    zxlogf(INFO,"ID_G_CHIPER_LOW(%02x) = %02x\n",0x9f,i2c_read(i2c,0x9f));
    zxlogf(INFO,"ID_G_TYPE      (%02x) = %02x\n",0xa0,i2c_read(i2c,0xa0));
    zxlogf(INFO,"ID_G_LIB_VHI   (%02x) = %02x\n",0xa1,i2c_read(i2c,0xa1));
    zxlogf(INFO,"ID_G_LIB_VLO   (%02x) = %02x\n",0xa2,i2c_read(i2c,0xa2));

    zxlogf(INFO,"ID_G_CHIPER_HI (%02x) = %02x\n",0xa3,i2c_read(i2c,0xa3));

    zxlogf(INFO,"ID_G_TYPE      (%02x) = %02x\n",0xa0,i2c_read(i2c,0xa0));
*/

    uint8_t buf[1024];
    uint8_t tbuf[2];
    tbuf[0] = 0x01;
    tbuf[1] = 0x00;
    i2c_transact_sync(i2c,0,tbuf,2,buf,0);
    tbuf[0] = 0x00;
    i2c_transact_sync(i2c,0,tbuf,1,buf,32);
    for(int i=0; i<32; i++) {
        zxlogf(INFO,"REG 0x%02x = %02x\n",i,buf[i]);
    }
    for(int i=0; i<17; i++) {
        zxlogf(INFO,"REG 0x%02x = %02x\n",0x9f+i,i2c_read(i2c,0x9f+i));
    }
/*
    uint8_t tbuf[5];
    uint8_t rbuf[5];
    tbuf[0] = FTS_REG_CHIP_ID;
    res = i2c_transact_sync(i2c, 0, tbuf, 1, rbuf, 1);
    if (res) {
        zxlogf(ERROR,"Read1 failed\n");
    }
    tbuf[0] = FTS_REG_CHIP_ID2;


    res = i2c_transact_sync(i2c, 0, tbuf, 1, &rbuf[1], 1);
    if (res) {
        zxlogf(ERROR,"Read2 failed\n");
    }

    zxlogf(INFO,"Chipid = %02x%02x\n",rbuf[0],rbuf[1]);


    tbuf[0] = 0x01;
    tbuf[1] = 0x00;
    res = i2c_transact_sync(i2c, 0, tbuf, 2, rbuf, 4);
    if (res) {
        zxlogf(ERROR,"Read3 failed\n");
    }

    zxlogf(INFO,"HID DESC = %02x %02x  %02x  %02x\n",rbuf[0],rbuf[1],rbuf[2],rbuf[3]);


    thrd_t testthread;
    thrd_create_with_name(&testthread, test_thread,
                                    i2c,
                                    "touch-thread");
*/

    zxlogf(INFO,"Exiting focaltech bind\n");
    return ZX_OK;
}
extern zx_status_t ft3x27_bind(void* ctx, zx_device_t* parent);


static zx_driver_ops_t focaltech_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .bind = ft3x27_bind,
};

// clang-format off
ZIRCON_DRIVER_BEGIN(focaltech_touch, focaltech_driver_ops, "focaltech-touch", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_GOOGLE),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_PID, PDEV_PID_ASTRO),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_ASTRO_FOCALTOUCH),
ZIRCON_DRIVER_END(focaltech_touch)
// clang-format on
