// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <magenta/syscalls.h>

#include <magenta/device/i2c.h>

#include <unistd.h>


#include "pcm5122.h"


#define PCM5122_I2C_ADDRESS 0x4d
#define DEVNAME "/dev/soc/bcm-i2c/i2c1"


static void pcm5122_write_reg(int fd, uint8_t address, uint8_t value) {
    uint8_t argbuff[2] = {address, value};
    write(fd,argbuff,2);
}

mx_status_t pcm5122_init(void) {


    int fd = open(DEVNAME,O_RDWR);
    if (fd<0) {
        printf("opening the audio didn't work\n");
        return ERR_NOT_FOUND;
    }


   i2c_ioctl_add_slave_args_t add_slave_args = {
        .chip_address_width = I2C_7BIT_ADDRESS,
        .chip_address = PCM5122_I2C_ADDRESS,
    };

    ssize_t ret = ioctl_i2c_bus_add_slave(fd, &add_slave_args);
    if (ret < 0) {
        printf("Error when adding I2C slave. (%zd)\n", ret);
        return ERR_INTERNAL;
    }

    pcm5122_write_reg(fd,0x08, 0x08);       // Turn on the LED
    pcm5122_write_reg(fd,0x53, 0x02);
    pcm5122_write_reg(fd,0x56, 0x08);

    pcm5122_write_reg(fd,13, 0x10);         // Clock source for pll = bclk

    pcm5122_write_reg(fd,37, 0x1a);         // Disable clock autoset
    pcm5122_write_reg(fd,27,    1);         //  DDSP divider 1 (=/2)
    pcm5122_write_reg(fd,28,   15);         // DAC Divider = /16
    pcm5122_write_reg(fd,29,    3);         // CP Divider = /4
    pcm5122_write_reg(fd,30,    7);         // OSR Divider = /8
    pcm5122_write_reg(fd,14, 0x10);         // DAC CLK Mux = PLL


    pcm5122_write_reg(fd,4 , (1 << 0));     // Enable the PLL
    pcm5122_write_reg(fd,20,  0 );              // P = 0
    pcm5122_write_reg(fd,21, 16 );              // J = 16
    pcm5122_write_reg(fd,22,  0 );              // D = 0
    pcm5122_write_reg(fd,23,  0 );              //      (D uses two registers)
    pcm5122_write_reg(fd,24,  1 );              // R = 2

    close(fd);

    return NO_ERROR;
}


bool pcm5122_is_valid_mode( audio2_stream_cmd_set_format_req_t req  ){

    //Yes this is lame.  Will make it better as we add more modes.
    if  ( (req.frames_per_second == 44100)  &&
          (req.packing == AUDIO2_BIT_PACKING_16BIT_LE) &&
          (req.channels == 2) ) {
        return true;
    }

    return false;
}