// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <stdint.h>
#include <threads.h>
#include <unistd.h>


#include <bits/limits.h>
#include <ddk/debug.h>
#include <hw/reg.h>

#include <zircon/assert.h>
#include <zircon/types.h>

#include "aml-i2c.h"
#include "a113-bus.h"

static aml_i2c_dev_t* i2c_b;
static aml_i2c_dev_t* i2c_a;

static int i2c_test_thread(void *arg) {
    aml_i2c_dev_t *dev = arg;

    aml_i2c_set_slave_addr(dev,0x18);

    while (1) {
        uint32_t token_num = 0;
        uint32_t token_reg = 0;

        token_reg |= TOKEN_START << (4*(token_num++));
        token_reg |= TOKEN_SLAVE_ADDR_WR << (4*(token_num++));
        token_reg |= TOKEN_DATA << (4*(token_num++));
        token_reg |= TOKEN_STOP << (4*(token_num++));

        dev->virt_regs->token_list_0 = token_reg;
        dev->virt_regs->token_wdata_0 = 0x00000000;  // set ptr to reg 0

        aml_i2c_start_xfer(dev);

        while (dev->virt_regs->control & 0x4) ;;    // wait for idle

        token_num = 0;
        token_reg  = TOKEN_START << (4*(token_num++));
        token_reg |= TOKEN_SLAVE_ADDR_RD << (4*(token_num++));
        token_reg |= TOKEN_DATA << (4*(token_num++));
        token_reg |= TOKEN_DATA << (4*(token_num++));
        token_reg |= TOKEN_DATA << (4*(token_num++));
        token_reg |= TOKEN_DATA_LAST << (4*(token_num++));
        token_reg |= TOKEN_STOP << (4*(token_num++));

        dev->virt_regs->token_list_0 = token_reg;
        aml_i2c_start_xfer(dev);
        while (dev->virt_regs->control & 0x4) ;;    // wait for idle

        int8_t x = (int8_t)((dev->virt_regs->token_rdata_0 >> 24) & 0xff);
        uint32_t id = dev->virt_regs->token_rdata_0 & 0xff;
        printf("x acc = %d\n",x);
        printf("id = %x\n",id);
        sleep(1);

    }

    return 0;
}


zx_status_t aml_i2c_test(aml_i2c_dev_t *dev) {

    thrd_t thrd;
    thrd_create_with_name(&thrd, i2c_test_thread, dev, "i2c_test_thread");
    return ZX_OK;
}


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

zx_status_t aml_i2c_start_xfer(aml_i2c_dev_t *dev) {

    dev->virt_regs->control &= ~0x00000001;
    dev->virt_regs->control |= 0x00000003;
    return ZX_OK;
}

zx_status_t aml_i2c_write(aml_i2c_dev_t *dev, uint8_t *buff, uint32_t len) {


    uint32_t token_num = 0;
    uint32_t token_reg = 0;

    token_reg |= TOKEN_START << (4*(token_num++));
    token_reg |= TOKEN_SLAVE_ADDR_WR << (4*(token_num++));
    token_reg |= TOKEN_DATA << (4*(token_num++));
    token_reg |= TOKEN_DATA << (4*(token_num++));
    token_reg |= TOKEN_STOP << (4*(token_num++));



    dev->virt_regs->token_list_0 = token_reg;
    dev->virt_regs->token_wdata_0 = 0x0000aaaa;

    aml_i2c_dumpstate(dev);

    aml_i2c_start_xfer(dev);

    aml_i2c_dumpstate(dev);

    return ZX_OK;
}

zx_status_t aml_i2c_set_slave_addr(aml_i2c_dev_t *dev, uint16_t addr) {

    addr &= 0x7f;
    uint32_t reg = dev->virt_regs->slave_addr;
    reg = reg & 0xff;
    reg = reg | ((addr << 1) & 0xff);
    dev->virt_regs->slave_addr = reg;

    return ZX_OK;
}

aml_i2c_connection_t *aml_i2c_connect(aml_i2c_dev_t *dev, uint32_t channel) {
    assert(dev);

    // Check if the i2c channel is already in use
    if (dev->connections) {

    }

}

/* create instance of aml_i2c_t and do basic initialization.  There will
be one of these instances for each of the soc i2c ports.
*/
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


    return ZX_OK;

init_fail:
    if (*device) {
        io_buffer_release(&(*device)->regs_iobuff));
        free(*device);
     };
    return status;
}
