// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ddk/device.h>
#include <ddk/driver.h>
#include <ddk/binding.h>
#include <ddk/protocol/google.h>
#include <ddk/protocol/display.h>

#include <magenta/syscalls.h>


// clang-format on

static uint8_t* msm_framebuffer = (uint8_t*)NULL;
static uint32_t msm_framebuffer_size = 0;

static mx_device_t disp_device;
static mx_display_info_t disp_info;


static mx_status_t msm_parse_framebuffer(uint8_t* args) {
    //mx_status_t ret = NO_ERROR;
    const char delim[] = ",";

    char * config;
    char * token;

    config = (char*)args;

    if (!msm_framebuffer) {

        // buffer needs to be aligned on 16 byte boundary, pad the alloc to make sure we have room to adjust

        token = strsep(&config,delim);
        printf("MSMFBUFF:  First parsed string = %s\n",token);
        mx_paddr_t pa = strtol(token,NULL,16);
        printf("MSMFBUFF:  Parsed fb phys pointer = %lx\n",pa);
        token = strsep(&config,delim);
        printf("MSMFBUFF:  second parsed string = %s\n",token);
        uint32_t width = strtol(token,NULL,10);

        token = strsep(&config,delim);
        printf("MSMFBUFF:  third parsed string = %s\n",token);
        uint32_t height = strtol(token,NULL,10);


        token = strsep(&config,delim);
        printf("MSMFBUFF:  fourth parsed string = %s\n",token);
        uint32_t bpp = strtol(token,NULL,10);


        printf("MSMFBUFF: dimensions:%ux%u  stride:%u\n",width,height,bpp);

        //mx_paddr_t pa;

        msm_framebuffer_size = width * height * bpp;


        uintptr_t page_base;

        // map framebuffer into userspace
#if 1
        mx_mmap_device_memory(
            get_root_resource(),
            pa, msm_framebuffer_size,
            MX_CACHE_POLICY_CACHED, &page_base);
        msm_framebuffer = (uint8_t*)page_base;
        memset(msm_framebuffer, 0x60, msm_framebuffer_size);

        printf("MSMFBUFF: fbuffer mapped at %p\n",msm_framebuffer );

    disp_info.format = MX_PIXEL_FORMAT_ARGB_8888;
    disp_info.width = width;
    disp_info.height = height;
    disp_info.stride = width;


#endif
    }
    //memcpy(fb_desc, &bcm_vc_framebuffer, sizeof(bcm_fb_desc_t));
    return 0;//sizeof(bcm_fb_desc_t);
}

void msm_flush_framebuffer(mx_device_t* dev) {
    mx_cache_flush(msm_framebuffer, msm_framebuffer_size,
                   MX_CACHE_FLUSH_DATA);
}

static mx_status_t msm_set_mode(mx_device_t* dev, mx_display_info_t* info) {

    return NO_ERROR;
}

static mx_status_t msm_get_mode(mx_device_t* dev, mx_display_info_t* info) {
    assert(info);
    memcpy(info, &disp_info, sizeof(mx_display_info_t));
    return NO_ERROR;
}

static mx_status_t msm_get_framebuffer(mx_device_t* dev, void** framebuffer) {
    assert(framebuffer);
    (*framebuffer) = msm_framebuffer;
    return NO_ERROR;
}

static mx_display_protocol_t msm_display_proto = {
    .set_mode = msm_set_mode,
    .get_mode = msm_get_mode,
    .get_framebuffer = msm_get_framebuffer,
    .flush = msm_flush_framebuffer
};

static mx_protocol_device_t msm_device_proto = {};


mx_status_t fb_bind(mx_driver_t* driver, mx_device_t* parent) {

    mx_status_t status;
    char* v = getenv("magenta.fbuffer");
    if (v) {
        printf("MSMFBUFFER: Got env string = %s\n",v);
    } else {
        printf("MSMFBUFFER: No environment strings\n");
    }
    msm_parse_framebuffer((uint8_t*)v);


#if 0
    uintptr_t page_base;

    // Carve out some address space for the device -- it's memory mapped.
    mx_status_t status = mx_mmap_device_memory(
        get_root_resource(),
        MAILBOX_PAGE_ADDRESS, MAILBOX_REGS_LENGTH,
        MX_CACHE_POLICY_UNCACHED_DEVICE, &page_base);

    if (status != NO_ERROR)
        return status;

    // The device is actually mapped at some offset into the page.
    mailbox_regs = (uint32_t*)(page_base + PAGE_REG_DELTA);

    mx_device_t* dev;
    status = device_create(&dev, driver, "bcm-vc-rpc", &mailbox_device_proto);
    if (status != NO_ERROR)
        return status;

    dev->props = calloc(2, sizeof(mx_device_prop_t));
    dev->props[0] = (mx_device_prop_t){BIND_SOC_VID, 0, SOC_VID_BROADCOMM};
    dev->props[1] = (mx_device_prop_t){BIND_SOC_DID, 0, SOC_DID_BROADCOMM_MAILBOX};
    dev->prop_count = 2;

    status = device_add(dev, parent);
    if (status != NO_ERROR) {
        free(dev);
        return status;
    }

    bcm_fb_desc_t framebuff_descriptor;

    // For now these are set to work with the rpi 5" lcd didsplay
    // TODO: add a mechanisms to specify and change settings outside the driver

    framebuff_descriptor.phys_width = 800;
    framebuff_descriptor.phys_height = 480;
    framebuff_descriptor.virt_width = 800;
    framebuff_descriptor.virt_height = 480;
    framebuff_descriptor.pitch = 0;
    framebuff_descriptor.depth = 32;
    framebuff_descriptor.virt_x_offs = 0;
    framebuff_descriptor.virt_y_offs = 0;
    framebuff_descriptor.fb_p = 0;
    framebuff_descriptor.fb_size = 0;

    bcm_vc_get_framebuffer(&framebuff_descriptor);
#endif
    device_init(&disp_device, driver, "msm-fb", &msm_device_proto);

    disp_device.protocol_id = MX_PROTOCOL_DISPLAY;
    disp_device.protocol_ops = &msm_display_proto;
    mx_set_framebuffer(get_root_resource(), msm_framebuffer,
                       msm_framebuffer_size, disp_info.format,
                       disp_info.width, disp_info.height, disp_info.stride);



    status = device_add(&disp_device, parent);
    return NO_ERROR;
    if (status != NO_ERROR) {
        return status;
    }

    return NO_ERROR;
}

mx_driver_t _driver_msm_fbuffer = {
    .name = "msm-fb",
    .ops = {
        .bind = fb_bind,
    },
};

MAGENTA_DRIVER_BEGIN(_driver_msm_fbuffer, "msm-fb", "magenta", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PROTOCOL, MX_PROTOCOL_SOC),
    BI_ABORT_IF(NE, BIND_SOC_VID, SOC_VID_GOOGLE),
    BI_MATCH_IF(EQ, BIND_SOC_DID, SOC_DID_TRAPPER),
MAGENTA_DRIVER_END(_driver_msm_fbuffer)
