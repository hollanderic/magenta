// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/io-buffer.h>
#include <ddk/protocol/i2c.h>
#include <ddk/protocol/platform-device.h>
#include <ddktl/device.h>
#include <ddktl/device-internal.h>
#include <ddktl/gpio_pin.h>
#include <dispatcher-pool/dispatcher-timer.h>
#include <lib/fzl/pinned-vmo.h>
#include <lib/simple-audio-stream/simple-audio-stream.h>
#include <lib/zx/bti.h>
#include <lib/zx/vmo.h>
#include <fbl/mutex.h>

#include <audio-proto/audio-proto.h>

#include <soc/aml-common/aml-audio.h>

#include "pinned-buffer.h"
#include "tas27xx.h"

namespace audio {
namespace astro {



class AstroAudioStreamOut : public SimpleAudioStream {
public:
    AstroAudioStreamOut(zx_device_t* parent);

protected:
    zx_status_t Init() __TA_REQUIRES(domain_->token()) override;
    zx_status_t ChangeFormat(const audio_proto::StreamSetFmtReq& req)
        __TA_REQUIRES(domain_->token()) override;
    zx_status_t GetBuffer(const audio_proto::RingBufGetBufferReq& req,
                          uint32_t* out_num_rb_frames,
                          zx::vmo* out_buffer) __TA_REQUIRES(domain_->token()) override;
    zx_status_t Start(uint64_t* out_start_time) __TA_REQUIRES(domain_->token()) override;
    zx_status_t Stop() __TA_REQUIRES(domain_->token()) override;
    zx_status_t InitPost();


private:

    friend class fbl::RefPtr<AstroAudioStreamOut>;

    // TODO(hollande) - the fifo bytes are adjustable on the audio fifos and should be scaled
    //                  with the desired sample rate.  Since this first pass has a fixed sample
    //                  sample rate we will set as constant for now.
    //                  We are using fifo B at this stage, which is max of 64 (64-bit wide)
    //                  Using 32 levels for now.
    static constexpr uint8_t kFifoDepth = 0x20;

    zx_status_t AddFormats();
    zx_status_t InitBuffer(size_t size);
    zx_status_t InitPdev();
    zx_status_t ProcessRingNotification();

    uint32_t us_per_notification_ = 0;
    volatile bool running_;

    fbl::RefPtr<dispatcher::Timer> notify_timer_;

    zx_device_t* parent_;

    fbl::RefPtr<ddk::Pdev> pdev_;

    fbl::unique_ptr<Tas27xx> codec_;

    zx::vmo ring_buffer_vmo_;
    fzl::PinnedVmo pinned_ring_buffer_;

    fbl::unique_ptr<AmlTdmDevice> aml_audio_;
    ddk::GpioPin audio_en_;
    ddk::GpioPin audio_fault_;

    zx::bti bti_;

    virtual ~AstroAudioStreamOut();

    //zx_status_t Bind(const char* devname);


};

}  // namespace usb
}  // namespace audio
