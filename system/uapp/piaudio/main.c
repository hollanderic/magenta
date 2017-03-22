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


const char* prog_name;

#define DEVNAME "/dev/soc/bcm-i2c/i2c1"
#define AUDIODEV "/dev/class/audio2-output/000"

#define BUFFERSIZE (4096)
#define NOTIFICATIONS 2

typedef union {
    audio2_cmd_hdr_t                         hdr;
    audio2_rb_cmd_get_buffer_resp_t          get_buffer_resp;
    audio2_stream_cmd_set_format_resp_t      set_fmt_resp;
    audio2_rb_cmd_start_resp_t               start_resp;
    audio2_rb_cmd_stop_resp_t                stop_resp;
    audio2_rb_position_notify_t              position_notify;
} buffer_packet_t;

mx_handle_t audio_port;
mx_handle_t stream_ch;
mx_handle_t rb_ch;
mx_handle_t vmo;
uint16_t distance=0;

volatile uint32_t rb_pos = 0;
atomic_int free_frames = ATOMIC_VAR_INIT(0);
volatile uint32_t frames_written=0;
double theta = 0;
double phi = 0;
double frequency = 120;
double integrator = 0;

volatile uintptr_t rb_ptr = 0;

void print_usage(void) {
    printf("Usage:\n");
    printf("\n");
    printf("%s DEVICE COMMAND [command arguments]\n", prog_name);
}

bool running = false;

mx_status_t audio_get_rb(void) {

    mx_status_t status;

    audio2_rb_cmd_get_buffer_req_t req;
    req.hdr.cmd = AUDIO2_RB_CMD_GET_BUFFER;

    req.min_ring_buffer_frames = BUFFERSIZE;

    req.notifications_per_ring = NOTIFICATIONS;

    status = mx_channel_write(rb_ch,0,&req,sizeof(req),NULL,0);
    if (status != NO_ERROR) {
        printf("get rb failed\n");
    }
    return status;
}

mx_status_t start_rb(void) {
    mx_status_t status;
    audio2_rb_cmd_start_req_t req;
    req.hdr.cmd = AUDIO2_RB_CMD_START;
    status = mx_channel_write(rb_ch,0,&req,sizeof(req),NULL,0);
    if (status != NO_ERROR) printf("Start command failed\n");
    return status;
}


mx_status_t audio_got_rb(audio2_rb_cmd_get_buffer_resp_t resp, mx_handle_t new_vmo) {

    vmo = new_vmo;
    uint64_t size;
    mx_status_t status = mx_vmo_get_size(vmo,&size);
    mx_nanosleep(MX_MSEC(500));
    if (status == NO_ERROR) {
        printf("Got back a %lu byte vmo\n",size);
        uintptr_t ptr;
        status = mx_vmar_map(mx_vmar_root_self(), 0, vmo, 0, size, MX_VM_FLAG_PERM_READ | MX_VM_FLAG_PERM_WRITE, &ptr);
        if (status != NO_ERROR) {
            printf("shit be broke yo\n");
            while(1);;
        }
        rb_ptr = ptr;
        printf("vmo mapped at %lx\n",rb_ptr);
    }
    return status;
}



int audio_thread(void* arg) {
    mx_status_t status;
    mx_io_packet_t port_out;
    mx_handle_t handles[4];
    uint32_t num_handles;
    buffer_packet_t req;
    uint32_t req_size;
    volatile int last_pos=0;
    int current_pos=0;

    while(running) {

        status = mx_port_wait(audio_port, MX_TIME_INFINITE, &port_out, sizeof(port_out));
        if (status != NO_ERROR) goto thrdexit;

        mx_handle_t channel = (mx_handle_t)port_out.hdr.key;

        if (port_out.signals == MX_CHANNEL_READABLE) {
             status = mx_channel_read(channel,0,&req,sizeof(req),&req_size,handles,4,&num_handles);
             if (status != NO_ERROR) goto thrdexit;
             //printf("Got %x from channel\n",req.hdr.cmd);

             switch(req.hdr.cmd) {
                case AUDIO2_STREAM_CMD_SET_FORMAT:
                    if ((num_handles == 1) && (req.set_fmt_resp.result == NO_ERROR)) {
                        rb_ch = handles[0];
                        //printf("Got rb channel 0x%x\n",rb_ch);
                        status = mx_port_bind(audio_port, (uint64_t)rb_ch,rb_ch, MX_CHANNEL_READABLE | MX_CHANNEL_PEER_CLOSED);
                        if (status != NO_ERROR) goto thrdexit;
                        audio_get_rb();

                    } else {
                        printf("stream channel response error\n");
                    }
                    break;
                case AUDIO2_RB_CMD_GET_BUFFER:
                    // Handle the response
                    if (num_handles == 1) {
                        audio_got_rb(req.get_buffer_resp,handles[0]);
                        printf("Ring buffer successfully retrieved\n");
                    } else printf("No buffer returned!\n");
                    break;
                case AUDIO2_RB_CMD_START:
                    printf("Ring buffer started\n");
                    break;
                case AUDIO2_RB_POSITION_NOTIFY:
                    //printf("%u\n",req.position_notify.ring_buffer_pos);
                    current_pos = req.position_notify.ring_buffer_pos;
                    int delta = (BUFFERSIZE*4 + current_pos - last_pos)%(BUFFERSIZE*4);
                    atomic_fetch_add(&free_frames,delta/4);
                    last_pos = current_pos;
                    break;
              default:
                    printf("unrecognized command on channel\n");
             }
        }
    }

thrdexit:
    printf("Audio thread exiting due to error\n");
    return -1;
}


mx_status_t audio2_init(void) {

    int fd = open(AUDIODEV, O_RDWR);
    if (fd < 0) return fd;

    int res = mxio_ioctl(fd, AUDIO2_IOCTL_GET_CHANNEL,
                                NULL, 0,
                                &stream_ch, sizeof(stream_ch));
    if (res != 0) {
        printf("Stream channel error\n");
        close(fd);
        return res;
    }
    close(fd);
    printf("Got stream channel handle 0x%x\n",stream_ch);

    mx_status_t status = mx_port_create(0,&audio_port);
    if (status != NO_ERROR) return status;
    status = mx_port_bind(audio_port, (uint64_t)stream_ch,stream_ch, MX_CHANNEL_READABLE | MX_CHANNEL_PEER_CLOSED);
    if (status != NO_ERROR) return status;
    printf("Stream port bound\n");

    running = true;
    thrd_t audio_thrd;
    res = thrd_create_with_name(&audio_thrd, audio_thread, NULL, "audio_thread");
    if (res != thrd_success) {
        goto teardown;
    }

    audio2_stream_cmd_set_format_req_t req;
    req.hdr.cmd = AUDIO2_STREAM_CMD_SET_FORMAT;
    req.sample_format = AUDIO2_SAMPLE_FORMAT_16BIT;
    req.frames_per_second = 48000;
    req.channels = 2;

    status = mx_channel_write(stream_ch,0,&req,sizeof(req),NULL,0);
    if (status != NO_ERROR) goto teardown;

    return NO_ERROR;
teardown:
    printf("something went wrong\n");
    if (audio_port != MX_HANDLE_INVALID) mx_handle_close(audio_port);
    if (stream_ch != MX_HANDLE_INVALID) mx_handle_close(stream_ch);
    stream_ch = MX_HANDLE_INVALID;
    audio_port = MX_HANDLE_INVALID;
    return -1;

}

int main(int argc, const char** argv) {
    if (argc < 2)
        return 1;
    uint32_t loop_count = 10;
    frequency = strtol(argv[1],NULL,10);
    //uint32_t span = strtol(argv[2],NULL,10);
    prog_name = argv[0];


    int16_t val;
    double sin_scale = 2.0*M_PI*frequency/48000;
    double theta = 0;
    __UNUSED volatile int16_t* samples = (int16_t*)rb_ptr;
    __UNUSED volatile uint32_t index = 0;

    free_frames = 0;
    frames_written = 0;
    audio2_init();

    while (!rb_ptr);;

    samples = (int16_t*)rb_ptr;

    for (uint i = 0; i < BUFFERSIZE; i+=2) {
        samples[i] = 2000*sin(theta);
        samples[i+1] = 2000*sin(theta);
        frames_written++;
        theta+=sin_scale;
        //rb_pos+=4;
    }

    start_rb();

    uint32_t start;
    uint32_t byte_count;
    //uint64_t now = 0;

    int32_t status;

    while (loop_count) {

        start = rb_pos;
        byte_count=0;
        while (atomic_load(&free_frames)) {

            val = 2000*sin(theta);
            ((int32_t*)rb_ptr)[index] = (val << 16) | ((uint16_t)val);
            index = (index + 1) % BUFFERSIZE;

            rb_pos = (rb_pos + 4) % (BUFFERSIZE*4);
            theta += sin_scale;
            //if (theta > (2*M_PI)) theta-=2*M_PI;

            byte_count+=4;
            atomic_fetch_add(&free_frames,-1);
        }
        theta = fmod(theta,2*M_PI);
        if (byte_count) {
            //printf("%8u  %8u %8u\n",byte_count,start, rb_pos);
            mx_time_t t = mx_time_get(MX_CLOCK_MONOTONIC);
            if (rb_pos > start) {
                status = mx_cache_flush((void*)(rb_ptr + start), rb_pos - start,MX_CACHE_FLUSH_DATA);
                //status = mx_vmo_op_range(vmo, MX_VMO_OP_CACHE_CLEAN,start,rb_pos - start,NULL, 0);
            } else {
                status = mx_cache_flush((void*)(rb_ptr + start), (BUFFERSIZE*4) - start,MX_CACHE_FLUSH_DATA);
                status +=mx_cache_flush((void*)rb_ptr, rb_pos,MX_CACHE_FLUSH_DATA);
                //status = mx_vmo_op_range(vmo, MX_VMO_OP_CACHE_CLEAN,start,(BUFFERSIZE*4) - start,NULL, 0);
                //status = mx_vmo_op_range(vmo, MX_VMO_OP_CACHE_CLEAN,0,rb_pos,NULL, 0);
            }
            // (status!=NO_ERROR) printf("somethign wrong with the cache flush - %d\n",status);
            t = mx_time_get(MX_CLOCK_MONOTONIC) - t;
            printf("%lu %d %lu\n",t,status,mx_ticks_per_second());
        }
    }

exit:



    return 1;
}
