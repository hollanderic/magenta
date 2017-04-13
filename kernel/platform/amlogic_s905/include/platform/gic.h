// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2014-2015 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#pragma once

#include <platform/s905.h>

#define GICBASE(n)  (CPUPRIV_BASE_VIRT)
#define GICC_OFFSET (0x02000)
#define GICD_OFFSET (0x01000)

