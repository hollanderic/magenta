// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/protocol/platform-bus.h>
#include <ddk/io-buffer.h>

#include "a113-bus.h"


#define I2CB_BASE_PHYS 0xffd1e000
#define I2CB_BASE_PAGE ((~0xFFF) & I2CB_BASE_PHYS)
#define I2CB_BASE_PAGE_OFFSET (I2CB_BASE_PHYS - I2CB_BASE_PAGE)

typedef enum {
    AML_I2C_B,
} aml_i2c_port_t;

typedef volatile struct {
    uint32_t    control;
    uint32_t    slave_address;
    uint32_t    token_list_0;
    uint32_t    token_list_1;
    uint32_t    token_wdata_0;
    uint32_t    token_wdata_1;
    uint32_t    token_rdata_0;
    uint32_t    token_rdata_1;
} aml_i2c_regs_t;

typedef struct {
    a113_bus_t     *host_bus;
    io_buffer_t    regs_iobuff;
    aml_i2c_regs_t *virt_regs;

    uint32_t        bitrate;
    uint32_t        slave_addr;

} aml_i2c_dev_t;


zx_status_t aml_i2c_init(aml_i2c_dev_t **device, a113_bus_t *host_bus,
                                                 aml_i2c_port_t portnum);
zx_status_t aml_i2c_dumpstate(aml_i2c_dev_t *dev);
