// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <errno.h>
#include <fcntl.h>
#include <magenta/syscalls.h>
#include <magenta/types.h>
#include <magenta/device/i2c.h>
#include <mxio/util.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

const char* prog_name;

#define PCM5122_I2C_ADDRESS 0x4d
#define DEVNAME "/dev/soc/bcm-i2c/i2c1"

static const char dev[] = DEVNAME;

void print_usage(void) {
    printf("Usage:\n");
    printf("\n");
    printf("\tummm... not really sure\n");
}

int add_slave(int fd, uint32_t address) {

    i2c_ioctl_add_slave_args_t add_slave_args = {
        .chip_address_width = I2C_7BIT_ADDRESS,
        .chip_address = address,
    };

    ssize_t ret = ioctl_i2c_bus_add_slave(fd, &add_slave_args);
    if (ret < 0) {
        printf("Error when adding I2C slave. (%zd)\n", ret);
        return 1;
    }

    return 0;
}

int cmd_read(int fd, int argc, const char** argv) {
    if (argc < 1) {
        print_usage();
        return 1;
    }

    long int length = strtol(argv[0], NULL, 10);
    if (errno) {
        print_usage();
        return errno;
    }

    uint8_t* buf = malloc(length);
    if (!buf) {
        printf("Failed to allocate buffer.\n");
        return 1;
    }

    int ret = read(fd, buf, length);
    if (ret < 0) {
        printf("Error reading from slave. (%d)\n", ret);
        goto cmd_read_finish;
    }

    for (int i = 0; i < length; i++) {
        printf(" %02x", buf[i]);
        if (i % 32 == 31) printf("\n");
    }
    printf("\n");

cmd_read_finish:
    free(buf);
    return ret;
}

int cmd_write(int fd, int argc, const char** argv) {
    if (argc < 1) {
        print_usage();
        return 1;
    }

    uint8_t* buf = malloc(argc);
    if (!buf) {
        printf("Failed to allocate buffer.\n");
        return 1;
    }

    int ret = 0;

    for (int i = 0; i < argc; i++) {
        buf[i] = strtol(argv[i], NULL, 16);
        if (errno) {
            ret = errno;
            print_usage();
            goto cmd_write_finish;
        }
    }

    ret = write(fd, buf, argc);
    if (ret < 0)
        printf("Error writing to slave. (%d)\n", ret);

cmd_write_finish:
    free(buf);
    return ret;
}

int cmd_transfer(int fd, int argc, const char** argv) {
    const size_t base_size = sizeof(i2c_slave_ioctl_segment_t);
    int ret = NO_ERROR;

    // Figure out how big our buffers need to be.
    // Start the counters with enough space for the I2C_SEGMENT_TYPE_END
    // segment.
    size_t in_len = base_size;
    size_t out_len = 0;
    int segments = 1;
    int count = argc;
    const char** arg = argv;
    while (count) {
        if (count < 2) {
            print_usage();
            goto cmd_transfer_finish_2;
        }

        in_len += base_size;

        int read;
        if (!strcmp(arg[0], "r")) {
            read = 1;
        } else if (!strcmp(arg[0], "w")) {
            read = 0;
        } else {
            print_usage();
            goto cmd_transfer_finish_2;
        }
        segments++;

        long int length = strtol(arg[1], NULL, 10);
        if (errno) {
            print_usage();
            return errno;
        }
        arg += 2;
        count -= 2;
        if (read) {
            out_len += length;
        } else {
            in_len += length;
            if (length > count) {
                print_usage();
                goto cmd_transfer_finish_2;
            }
            arg += length;
            count -= length;
        }
    }

    // Allocate the input and output buffers.
    void* in_buf = malloc(in_len);
    void* out_buf = malloc(out_len);
    if (!in_buf || !out_buf) {
        ret = 1;
        goto cmd_transfer_finish_1;
    }
    uint8_t* data_addr = (uint8_t*)in_buf + segments * base_size;
    uint8_t* data_buf = data_addr;

    // Fill the "input" buffer which is sent to the ioctl.
    uintptr_t in_addr = (uintptr_t)in_buf;
    int i = 0;
    i2c_slave_ioctl_segment_t* ioctl_segment = (i2c_slave_ioctl_segment_t*)in_addr;
    while (i < argc) {
        if (!strcmp(argv[i++], "r")) {
            ioctl_segment->type = I2C_SEGMENT_TYPE_READ;
            ioctl_segment->len = strtol(argv[i++], NULL, 10);
            if (errno) {
                print_usage();
                return errno;
            }
        } else {
            ioctl_segment->type = I2C_SEGMENT_TYPE_WRITE;
            ioctl_segment->len = strtol(argv[i++], NULL, 10);
            if (errno) {
                print_usage();
                return errno;
            }

            for (int seg = 0; seg < ioctl_segment->len; seg++) {
                *data_buf++ = strtol(argv[i++], NULL, 16);
                if (errno) {
                    print_usage();
                    return errno;
                }
            }
        }
        ioctl_segment++;
    }
    ioctl_segment->type = I2C_SEGMENT_TYPE_END;
    ioctl_segment->len = 0;
    ioctl_segment++;
    // We should be at the start of the data section now.
    if ((uint8_t*)ioctl_segment != data_addr) {
        ret = 1;
        goto cmd_transfer_finish_1;
    }

    ret = ioctl_i2c_slave_transfer(fd, in_buf, in_len, out_buf, out_len);
    if (ret < 0)
        goto cmd_transfer_finish_1;

    for (size_t i = 0; i < out_len; i++) {
        printf(" %02x", ((uint8_t*)out_buf)[i]);
        if (i % 32 == 31) printf("\n");
    }
    printf("\n");

    ret = 0;

cmd_transfer_finish_1:
    free(in_buf);
    free(out_buf);
cmd_transfer_finish_2:
    return ret;
}

static void write_reg(int fd, uint8_t address, uint8_t value) {
    uint8_t argbuff[2] = {address, value};
    write(fd,argbuff,2);
}

static uint32_t read_reg(int fd, uint8_t address) {
    write(fd,&address,1);
    uint8_t buff[4];
    read(fd,buff,1);
    return (uint32_t)buff[0];
}

int cmd_init_device(int fd, int argc, const char** argv) {

    write_reg(fd,0x08, 0x08);       // Turn on the LED
    write_reg(fd,0x53, 0x02);
    write_reg(fd,0x56, 0x08);

    write_reg(fd,13, 0x10);         // Clock source for pll = bclk

    write_reg(fd,37, 0x1a);         // Disable clock autoset
    write_reg(fd,27,    1);         //  DDSP divider 1 (=/2)
    write_reg(fd,28,   15);         // DAC Divider = /16
    write_reg(fd,29,    3);         // CP Divider = /4
    write_reg(fd,30,    7);         // OSR Divider = /8
    write_reg(fd,14, 0x10);         // DAC CLK Mux = PLL


    write_reg(fd,4 , (1 << 0));     // Enable the PLL
    write_reg(fd,20,  0 );              // P = 0
    write_reg(fd,21, 16 );              // J = 16
    write_reg(fd,22,  0 );              // D = 0
    write_reg(fd,23,  0 );              //      (D uses two registers)
    write_reg(fd,24,  1 );              // R = 2

    close(fd);
    return 0;
}

static int cmd_read_reg(int fd,int argc,const char **argv) {
    uint address;

    if (argv[0][1] == 'x') {
        address = strtol(argv[0],NULL,16);
    } else address = strtol(argv[0],NULL,10);

    uint8_t value = read_reg(fd,(uint8_t)address);
    printf("0x%02x (%3d) = 0x%02x\n",address,address,value);

    return 0;
}

static int cmd_write_reg(int fd,int argc,const char **argv) {
    uint address;
    uint value;

    if (argv[0][1] == 'x') {
        address = strtol(argv[0],NULL,16);
    } else address = strtol(argv[0],NULL,10);

    if (argv[1][1] == 'x') {
        value = strtol(argv[1],NULL,16);
    } else value = strtol(argv[1],NULL,10);

    write_reg(fd,(uint8_t)address,(uint8_t)value);
    printf("0x%02x (%3d) = 0x%02x\n",address,address,value);

    return 0;
}
static int cmd_write_buff(int fd,int argc,const char **argv) {
    uint length;
    uint value;

    if (argv[0][1] == 'x') {
        length = strtol(argv[0],NULL,16);
    } else length = strtol(argv[0],NULL,10);

    if (argv[1][1] == 'x') {
        value = strtol(argv[1],NULL,16);
    } else value = strtol(argv[1],NULL,10);

    uint8_t* buf = malloc(length);
    for (uint i =0 ; i <  length; i++)
        buf[i] = value;

    int32_t numbytes = write(fd,buf,length);
    printf("wrote %d bytes\n",numbytes);

    return 0;
}

int main(int argc, const char** argv) {
    if (argc < 2) {
        printf("Usage:\n");
        printf("\tpia i              Initializes the HiFiBerry\n");
        printf("\tpia r <addr>       Reads the address in control registers\n");
        printf("\tpia w <addr> <val> Writes the address in control registers\n");
        return 1;
    }

    int fd2 = open("/dev/soc/bcm-pcm/pcm0",O_RDWR);
    if (fd2<0)
        printf("opening the audio didn't work\n");


    const char* cmd = argv[1];
    argv += 2;
    argc -= 2;

    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        printf("Error opening I2C device.\n");
        return 1;
    }
    add_slave(fd,PCM5122_I2C_ADDRESS);

    if (!strcmp("i",cmd)) {
        return cmd_init_device(fd,argc,argv);
    } else if (!strcmp("r",cmd)) {
        return cmd_read_reg(fd,argc,argv);
    } else if (!strcmp("w",cmd)) {
        return cmd_write_buff(fd2,argc,argv);
    }
}
