// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <magenta/syscalls.h>
#include <magenta/compiler.h>

#include <magenta/device/audio2.h>
#include <magenta/types.h>
#include <stdatomic.h>
#include <magenta/syscalls/port.h>
#include <magenta/process.h>
#include <magenta/threads.h>
#include <magenta/device/i2c.h>
#include <mxio/util.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <bcm/dma.h>
#include <bcm/bcm28xx.h>


const char* prog_name;

#define DEVNAME "/dev/soc/bcm-dma"

#define BUFFERSIZE (4096)
#define NOTIFICATIONS 2

mx_handle_t dma_port;

typedef struct {

    mx_handle_t dma_ch;
    mx_handle_t vmo_a;
    mx_handle_t vmo_b;
} dma_test_t;


dma_test_t testchans[10];


void print_usage(void) {
    printf("Usage:\n");
    printf("\n");
    printf("%s DEVICE COMMAND [command arguments]\n", prog_name);
}




mx_status_t dma_init(dma_test_t* dma) {

    int fd = open(DEVNAME, O_RDWR);
    if (fd < 0) return fd;

    int res = mxio_ioctl(fd, BCM_DMA_IOCTL_GET_CHANNEL,
                                NULL, 0,
                                &dma->dma_ch, sizeof(mx_handle_t));
    if (res != 0) {
        printf("DMA channel error\n");
        close(fd);
        return res;
    }
    close(fd);
    printf("Got DMA channel handle 0x%x\n",dma->dma_ch);


    mx_status_t status = mx_port_bind(dma_port, (uint64_t)dma,dma->dma_ch, MX_CHANNEL_READABLE | MX_CHANNEL_PEER_CLOSED);
    if (status != NO_ERROR) return status;
    printf("DMA port bound\n");

/*
    audio2_stream_cmd_set_format_req_t req;
    req.hdr.cmd = AUDIO2_STREAM_CMD_SET_FORMAT;
    req.sample_format = AUDIO2_SAMPLE_FORMAT_16BIT;
    req.frames_per_second = 48000;
    req.channels = 2;

    status = mx_channel_write(stream_ch,0,&req,sizeof(req),NULL,0);
    if (status != NO_ERROR) goto teardown;
*/
    return NO_ERROR;
}

mx_status_t get_vmo(mx_handle_t* vmo, size_t size) {

    mx_vmo_create(size,0,vmo);
    return mx_vmo_op_range(*vmo, MX_VMO_OP_COMMIT, 0, size, NULL, 0);

}
void send_shit(dma_test_t* dma) {
    mx_status_t status;

   bcm_dma_transfer_req_t req;
   req.hdr.cmd = BCM_DMA_SEND_SHIT;
   req.source_offset = 0;
   req.dest_offset = 0;
   req.num_bytes = 100;

   mx_handle_t handles[2];
   handles[0] = dma->vmo_a;
   handles[1] = dma->vmo_b;
   status = mx_channel_write(dma->dma_ch,0,&req,sizeof(req),handles,2);
   mx_nanosleep(MX_MSEC(100));
   printf("Got status = %d\n",status);
}


void send_debug(dma_test_t* dma, bcm_dma_cmd_t command) {
    mx_status_t status;

   bcm_dma_cmd_hdr_t req;
   req.cmd = command;
   status = mx_channel_write(dma->dma_ch,0,&req,sizeof(req),NULL,0);
   mx_nanosleep(MX_MSEC(100));
   printf("DEBUG Got status = %d\n",status);

}


int main(int argc, const char** argv) {

    mx_status_t status;
    prog_name = argv[0];

    status = mx_port_create(0,&dma_port);
    if (status != NO_ERROR) {
        printf("something broke - %d\n",status);
        return status;
    }


   dma_init(&testchans[0]);
   dma_init(&testchans[1]);
   dma_init(&testchans[2]);
   mx_nanosleep(MX_MSEC(100));

   get_vmo(&testchans[0].vmo_a, 4096);
   get_vmo(&testchans[0].vmo_b, 4096);
   printf("Sending two vmo handles: %x   %x\n",testchans[0].vmo_a,testchans[0].vmo_b);
   mx_nanosleep(MX_MSEC(50));

   send_shit(&testchans[0]);

   send_debug(&testchans[0], BCM_DMA_DEBUG_SHOW_CLIENTS);

   mx_nanosleep(MX_MSEC(100));
   mx_handle_close(dma_port);
   mx_handle_close(testchans[0].dma_ch);
exit:



    return 1;
}
