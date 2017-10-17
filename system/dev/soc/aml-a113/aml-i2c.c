// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdint.h>
#include <threads.h>

#include <bits/limits.h>
#include <ddk/debug.h>
#include <hw/reg.h>

#include <zircon/assert.h>
#include <zircon/types.h>

#include "aml-i2c.h"
#include "a113-bus.h"

zx_status_t aml_i2c_dumpstate(aml_i2c_dev_t *dev) {
    printf("control reg      : %08x\n",dev->virt_regs->control);
    printf("slave addr  reg  : %08x\n",dev->virt_regs->slave_addr);
    printf("token list0 reg  : %08x\n",dev->virt_regs->token_list_0);
    printf("token list1 reg  : %08x\n",dev->virt_regs->token_list_1);
    printf("token wdata0     : %08x\n",dev->virt_regs->token_wdata_0);
    printf("token wdata1     : %08x\n",dev->virt_regs->token_wdata_1);
    printf("token rdata0     : %08x\n",dev->virt_regs->token_rdata_0);
    printf("token rdata1     : %08x\n",dev->virt_regs->token_rdata_1);

    return ZX_OK;
}

zx_status_t aml_i2c_init(aml_i2c_dev_t **device, a113_bus_t *host_bus,
                                                 aml_i2c_port_t portnum) {

    *device = calloc(1, sizeof(aml_i2c_dev_t));
    if (!(*device)) {
        return ZX_ERR_NO_MEMORY;
    }

    (*device)->host_bus = host_bus;  // TODO - might not need this

    zx_handle_t resource = get_root_resource();
    zx_status_t status;

    status = io_buffer_init_physical(&(*device)->regs_iobuff, I2CB_BASE_PAGE, PAGE_SIZE,
                                     resource, ZX_CACHE_POLICY_UNCACHED_DEVICE);

    if (status != ZX_OK) {
        dprintf(ERROR, "aml_i2c_init: io_buffer_init_physical failed %d\n", status);
        goto init_fail;
    }

    (*device)->virt_regs = (aml_i2c_regs_t*)(io_buffer_virt(&(*device)->regs_iobuff));

    aml_i2c_regs_t *reg = (*device)->virt_regs;
    reg->control = reg->control | (1<<22);
    reg->control = reg->control & ~(1<<23);

    return ZX_OK;

init_fail:
    if (*device) free(*device);
    return status;
}
