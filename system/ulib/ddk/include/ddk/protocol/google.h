// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/driver.h>
#include <magenta/device/ioctl.h>
#include <magenta/device/ioctl-wrapper.h>
#include <magenta/compiler.h>
#include <stddef.h>

__BEGIN_CDECLS;

// Google vendor id
#define SOC_VID_GOOGLE 0x0000


// Google specific PIDs
#define SOC_DID_TRAPPER                  0x0000


__END_CDECLS
