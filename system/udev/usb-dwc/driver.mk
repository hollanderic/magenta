# Copyright 2016 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

# TODO(gkalsi, MG-295): Remove this compile time flag.
# ifeq ($(PLATFORM),bcm28xx)

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE_STATIC_LIBS += \
	udev/usb-dwc-xinu

DRIVER_SRCS += \
    $(LOCAL_DIR)/usb-dwc.c \
    $(LOCAL_DIR)/dwc-request-scheduler.c

# endif