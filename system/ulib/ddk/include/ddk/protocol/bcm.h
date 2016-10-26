// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/driver.h>
#include <magenta/device/ioctl.h>
#include <magenta/device/ioctl-wrapper.h>
#include <magenta/compiler.h>
#include <stddef.h>

__BEGIN_CDECLS;

// Broadcomm vendor id
#define SOC_VID_BROADCOMM 0x00BC


// Broadcomm specific PIDs
#define SOC_DID_BROADCOMM_VIDEOCORE_BUS 0x0000  // Videocore device (used as root bus)
#define SOC_DID_BROADCOMM_MAILBOX       0x0001  // Videocore mailbox, used for comms between cpu/gpu


#define BCM_MAILBOX_REQUEST            0x00000000
#define BCM_MAILBOX_REQUEST_SUCCESS    0x80000000
#define BCM_MAILBOX_REQUEST_ERROR      0x80000001

#define BCM_MAILBOX_TAG_GET_POWER_STATE 0x00020001
#define BCM_MAILBOX_TAG_SET_POWER_STATE 0x00028001

typedef struct {
    uint32_t tag;
    uint32_t size;
    uint32_t len;
} bcm_tag_header_t;

struct bcm_mailbox_header {
    uint32_t    buff_size;
    uint32_t    code;
};

typedef struct bcm_mailbox_header bcm_mailbox_header_t;


typedef struct {
    bcm_tag_header_t tag_header;
    union {
        struct {
            //uint32_t dev_id;
        } request;
        struct {
            uint32_t dev_id;
            uint32_t state;
        } response;
    } body;
} bcm_get_powerstate_tag_t;



typedef struct {
    uint32_t size;
    uint32_t code;
    union {
        bcm_get_powerstate_tag_t powerstate_tag;
    } tag;
} bcm_mailbox_message_t;


#define MAILBOX_INIT_TAG(tag_t,id) { \
    (tag_t)->tag_header.tag = BCM_MAILBOX_TAG_##id; \
    (tag_t)->tag_header.size = sizeof((tag_t)->body); \
    (tag_t)->tag_header.len = sizeof((tag_t)->body.request); \
    }




typedef struct {
    uint32_t phys_width;    //request
    uint32_t phys_height;   //request
    uint32_t virt_width;    //request
    uint32_t virt_height;   //request
    uint32_t pitch;         //response
    uint32_t depth;         //request
    uint32_t virt_x_offs;   //request
    uint32_t virt_y_offs;   //request
    uint32_t fb_p;          //response
    uint32_t fb_size;       //response
} bcm_fb_desc_t;


#define IOCTL_BCM_POWER_ON_USB \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_BCM, 0)

#define IOCTL_BCM_GET_FRAMEBUFFER \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_BCM, 1)

#define IOCTL_BCM_FILL_FRAMEBUFFER \
    IOCTL(IOCTL_KIND_DEFAULT, IOCTL_FAMILY_BCM, 2)


// ssize_t ioctl_bcm_power_on_usb(int fd);
IOCTL_WRAPPER(ioctl_bcm_power_on_usb, IOCTL_BCM_POWER_ON_USB);

// ssize_t ioctl_bcm_get_framebuffer(int fd, bcm_fb_desc_t*, bcm_fb_desc_t*);
IOCTL_WRAPPER_INOUT(ioctl_bcm_get_framebuffer, IOCTL_BCM_GET_FRAMEBUFFER, bcm_fb_desc_t, bcm_fb_desc_t);

IOCTL_WRAPPER_IN(ioctl_bcm_fill_framebuffer, IOCTL_BCM_FILL_FRAMEBUFFER, uint8_t);

__END_CDECLS
