// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include <ddk/binding.h>
#include <ddk/driver.h>
#include <ddk/protocol/platform-defs.h>
#include <ddk/debug.h>
#include <ddktl/pdev.h>
#include <math.h>

#include "audio-stream-in.h"

namespace audio {
namespace astro {

// Calculate ring buffer size for 1 second of 16-bit, 48kHz, stereo.
constexpr size_t RB_SIZE = fbl::round_up<size_t, size_t>(48000 * 2 * 2u, PAGE_SIZE);

AstroAudioStreamIn::AstroAudioStreamIn(zx_device_t* parent)
    : SimpleAudioStream(parent, true) {
}

zx_status_t AstroAudioStreamIn::Init() {
    zx_status_t status;

    status = InitPdev();
    if (status != ZX_OK) {
        return status;
    }

    status = AddFormats();
    if (status != ZX_OK) {
        return status;
    }
#if 0
    // Set our gain capabilities.
    cur_gain_state_.cur_gain = codec_->GetGain();
    cur_gain_state_.cur_mute = false;
    cur_gain_state_.cur_agc = false;

    cur_gain_state_.min_gain = codec_->GetMinGain();
    cur_gain_state_.max_gain = codec_->GetMaxGain();
    cur_gain_state_.gain_step = codec_->GetGainStep();
    cur_gain_state_.can_mute = false;
    cur_gain_state_.can_agc = false;
#endif
    snprintf(device_name_, sizeof(device_name_), "astro-audio-in");
    snprintf(mfr_name_, sizeof(mfr_name_), "Spacely Sprockets");
    snprintf(prod_name_, sizeof(prod_name_), "astro");

    unique_id_ = AUDIO_STREAM_UNIQUE_ID_BUILTIN_MICROPHONE;

    zxlogf(INFO,"***********************************\n");
    zxlogf(INFO,"***********************************\n");
    zxlogf(INFO,"***********************************\n");
    zxlogf(INFO,"***********************************\n");
    zxlogf(INFO,"***********************************\n");
    zxlogf(INFO,"***********************************\n");
    zxlogf(INFO,"***********************************\n");
    zxlogf(INFO,"***********************************\n");
    zxlogf(INFO,"***********************************\n");

    return ZX_OK;
}

zx_status_t AstroAudioStreamIn::InitPdev() {

    pdev_ = ddk::Pdev::Create(parent());
    if (!pdev_) {
        return ZX_ERR_NO_RESOURCES;
    }

    zx_status_t status = pdev_->GetBti(0, &bti_);
    if (status  != ZX_OK) {
        zxlogf(ERROR, "%s could not obtain bti - %d\n", __func__, status);
        return status;
    }

    pdm_ = AmlPdmDevice::Create(fbl::move(pdev_->GetMmio(0)),
                                fbl::move(pdev_->GetMmio(1)),
                                      HIFI_PLL, HIFI_PLL, TODDR_B);
    if (pdm_ == nullptr) {
        zxlogf(ERROR, "%s failed to create pdm device\n", __func__);
        return ZX_ERR_NO_MEMORY;
    }
#if 0
    //Initialize the ring buffer
    InitBuffer(RB_SIZE);

    aml_audio_->SetBuffer(pinned_ring_buffer_.region(0).phys_addr,
                          pinned_ring_buffer_.region(0).size);

    aml_audio_->ConfigTdmOutSlot(3, 3, 31, 15);

    //Setup appropriate tdm clock signals
    aml_audio_->SetMclkDiv(124);

    aml_audio_->SetSclkDiv(1, 0, 127);

    aml_audio_->Sync();
#endif
    return ZX_OK;
}


zx_status_t AstroAudioStreamIn::ChangeFormat(const audio_proto::StreamSetFmtReq& req) {
    fifo_depth_ = pdm_->fifo_depth();
    external_delay_nsec_ = 0;

    // At this time only one format is supported, and hardware is initialized
    //  during driver binding, so nothing to do at this time.
    return ZX_OK;
}

//void AstroAudioStreamIn::ShutdownHook() {
//    pdm_->Shutdown();
//}

zx_status_t AstroAudioStreamIn::GetBuffer(const audio_proto::RingBufGetBufferReq& req,
                                           uint32_t* out_num_rb_frames,
                                           zx::vmo* out_buffer) {
#if 0
    uint32_t rb_frames =
        static_cast<uint32_t>(pinned_ring_buffer_.region(0).size) / frame_size_;

    if (req.min_ring_buffer_frames > rb_frames) {
        return ZX_ERR_OUT_OF_RANGE;
    }
    zx_status_t status;
    constexpr uint32_t rights = ZX_RIGHT_READ | ZX_RIGHT_WRITE | ZX_RIGHT_MAP | ZX_RIGHT_TRANSFER;
    status = ring_buffer_vmo_.duplicate(rights, out_buffer);
    if (status != ZX_OK) {
        return status;
    }

    *out_num_rb_frames = rb_frames;

    aml_audio_->SetBuffer(pinned_ring_buffer_.region(0).phys_addr,
                          rb_frames * frame_size_);
#endif
    return ZX_OK;
}


zx_status_t AstroAudioStreamIn::Start(uint64_t* out_start_time) {
#if 0
    *out_start_time = pdm_->Start();

    uint32_t notifs = LoadNotificationsPerRing();
    if (notifs) {
        us_per_notification_ = static_cast<uint32_t>(
            1000 * pinned_ring_buffer_.region(0).size / (frame_size_ * 48 * notifs));
        notify_timer_->Arm(zx_deadline_after(ZX_USEC(us_per_notification_)));
    } else {
        us_per_notification_ = 0;
    }
#endif
    return ZX_OK;
}

zx_status_t AstroAudioStreamIn::Stop() {
    notify_timer_->Cancel();
    us_per_notification_ = 0;
    pdm_->Stop();
    return ZX_OK;
}

zx_status_t AstroAudioStreamIn::AddFormats() {
    fbl::AllocChecker ac;
    supported_formats_.reserve(1, &ac);
    if (!ac.check()) {
        zxlogf(ERROR, "Out of memory, can not create supported formats list\n");
        return ZX_ERR_NO_MEMORY;
    }

    // Add the range for basic audio support.
    audio_stream_format_range_t range;

    range.min_channels = 2;
    range.max_channels = 2;
    range.sample_formats = AUDIO_SAMPLE_FORMAT_16BIT;
    range.min_frames_per_second = 48000;
    range.max_frames_per_second = 48000;
    range.flags = ASF_RANGE_FLAG_FPS_48000_FAMILY;

    supported_formats_.push_back(range);

    return ZX_OK;
}






} //namespace astro
} //namespace audio


__BEGIN_CDECLS

zx_status_t pdm_audio_bind(void* ctx, zx_device_t* device) {

    auto stream =
        audio::SimpleAudioStream::Create<audio::astro::AstroAudioStreamIn>(device);
    if (stream == nullptr) {
        return ZX_ERR_NO_MEMORY;
    }

    __UNUSED auto dummy = stream.leak_ref();

    return ZX_OK;
}

static zx_driver_ops_t aml_pdm_driver_ops = {
    .version = DRIVER_OPS_VERSION,
    .init = nullptr,
    .bind = pdm_audio_bind,
    .create = nullptr,
    .release = nullptr,
};

// clang-format off
ZIRCON_DRIVER_BEGIN(aml_pdm, aml_pdm_driver_ops, "aml-pdm-in", "0.1", 3)
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_VID, PDEV_VID_AMLOGIC),
    BI_ABORT_IF(NE, BIND_PLATFORM_DEV_PID, PDEV_PID_AMLOGIC_S905D2),
    BI_MATCH_IF(EQ, BIND_PLATFORM_DEV_DID, PDEV_DID_ASTRO_PDM),
ZIRCON_DRIVER_END(aml_pdm)
// clang-format on
__END_CDECLS
