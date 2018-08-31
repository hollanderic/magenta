
#include <ddk/debug.h>
#include <ddktl/pdev.h>
#include <math.h>

#include "audio-stream-out.h"

namespace audio {
namespace astro {

// Calculate ring buffer size for 1 second of 16-bit, 48kHz, stereo.
constexpr size_t RB_SIZE = fbl::round_up<size_t, size_t>(48000 * 2 * 2u, PAGE_SIZE);

/*
    Assumes the GPIO for all resources are configured in the board file
*/
AstroAudioStreamOut::AstroAudioStreamOut (zx_device_t* parent) :
    SimpleAudioStream(parent, false) {
    parent_ = parent;
    running_ = false;
}

zx_status_t AstroAudioStreamOut::InitPdev() {

    pdev_ = ddk::Pdev::Create(parent_);

    audio_fault_ = pdev_->GetGpio(0);
    audio_en_ = pdev_->GetGpio(1);

    if (!(audio_fault_.is_valid() && audio_en_.is_valid())) {
        zxlogf(ERROR,"%s failed to allocate gpio\n", __func__);
        return ZX_ERR_NO_RESOURCES;
    }

    codec_ = Tas27xx::Create(pdev_->GetI2cChan(0).release());
    if (!codec_) {
        zxlogf(ERROR,"%s could not get tas27xx\n", __func__);
        return ZX_ERR_NO_RESOURCES;
    }

    pdev_->GetBti(0, &bti_);

    aml_audio_ = AmlTdmDevice::Create(pdev_->GetMmio(0).release(),
        HIFI_PLL, TDM_OUT_B, FRDDR_B, MCLK_A);
    if (aml_audio_ == nullptr) {
        zxlogf(ERROR,"%s failed to create tdm device\n", __func__);
        return ZX_ERR_NO_MEMORY;
    }

    //Enable Codec
    audio_en_.Write(1);

    //Initialize Codec - Needs to happen prior to bringing tdm clocks online
    codec_->Init();

    //Initialize the ring buffer
    InitBuffer(RB_SIZE);
    zx_paddr_t phys;
    phys = pinned_ring_buffer_.region(0).phys_addr;
    zxlogf(INFO,"Ring buffer phys address = %08lx\n", phys);
    zxlogf(INFO,"Ring buffer size = %ld\n", RB_SIZE);

    aml_audio_->SetBuffer(phys, RB_SIZE);

    aml_audio_->ConfigTdmOutSlot(3, 3, 31, 15);

    //Setup appropriate tdm clock signals
    aml_audio_->SetMclkDiv(124);

    aml_audio_->SetSclkDiv(1, 0, 127);

    aml_audio_->TdmOutReset();


    return ZX_OK;
}

zx_status_t AstroAudioStreamOut::Init() {
    zx_status_t status;

    status = InitPdev();
    if (status != ZX_OK) {
        return status;
    }

    status = AddFormats();
    if (status != ZX_OK) {
        return status;
    }

    // Set our gain capabilities.
    cur_gain_state_.cur_gain = -20.0;
    cur_gain_state_.cur_mute = false;
    cur_gain_state_.cur_agc  = false;

    cur_gain_state_.min_gain = -100.0;
    cur_gain_state_.max_gain = 0.0;
    cur_gain_state_.gain_step = 0.5;
    cur_gain_state_.can_mute = false;
    cur_gain_state_.can_agc  = false;

    snprintf(device_name_, sizeof(device_name_), "astro-audio-out");

    //16 byte random number for unique id
    uint8_t id[] = {0xec, 0x56, 0x21, 0x33, 0x96, 0xba, 0x22, 0xc1,
                    0xdd, 0x11, 0x00, 0x77, 0xf9, 0xa1, 0x89, 0x14};
    ::memcpy(unique_id_.data, id, sizeof(id));

    return ZX_OK;
}

zx_status_t AstroAudioStreamOut::InitPost() {

    notify_timer_ = dispatcher::Timer::Create();

    dispatcher::Timer::ProcessHandler thandler(
            [tdm = this](dispatcher::Timer * timer)->zx_status_t {
                OBTAIN_EXECUTION_DOMAIN_TOKEN(t, tdm->domain_);
                return tdm->ProcessRingNotification();
            });

    notify_timer_->Activate(domain_, fbl::move(thandler));
    return ZX_OK;
}

zx_status_t AstroAudioStreamOut::ProcessRingNotification() {

    if (running_) {
        notify_timer_->Arm(zx_deadline_after(ZX_USEC(us_per_notification_)));
    } else {
        notify_timer_->Cancel();
    }

    audio_proto::RingBufPositionNotify resp = { };
    resp.hdr.cmd = AUDIO_RB_POSITION_NOTIFY;

    resp.ring_buffer_pos = aml_audio_->GetRingPosition();
    return NotifyPosition(resp);
}

zx_status_t AstroAudioStreamOut::ChangeFormat(const audio_proto::StreamSetFmtReq& req) {


    fifo_depth_ = kFifoDepth;
    external_delay_nsec_ = 0;

    // At this time only one format is supported, and hardware is initialized
    //  during driver binding, so nothing to do at this time.

    return ZX_OK;
}

zx_status_t AstroAudioStreamOut::GetBuffer(const audio_proto::RingBufGetBufferReq& req,
                          uint32_t* out_num_rb_frames,
                          zx::vmo* out_buffer){

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

    zx_paddr_t phys;
    phys = pinned_ring_buffer_.region(0).phys_addr;

    aml_audio_->SetBuffer(phys, rb_frames * frame_size_);

    return ZX_OK;
}

zx_status_t AstroAudioStreamOut::Start(uint64_t* out_start_time) {
    uint64_t a, b;

    aml_audio_->TdmOutReset();
    aml_audio_->FRDDREnable();
    a = zx_clock_get(ZX_CLOCK_MONOTONIC);
    aml_audio_->TdmOutEnable();
    b = zx_clock_get(ZX_CLOCK_MONOTONIC);
    *out_start_time = ((b - a) >> 1) + a;

    running_ = true;

    uint32_t notifs = LoadNotificationsPerRing();
    if (notifs) {
        us_per_notification_ = static_cast<uint32_t>(
            1000 * pinned_ring_buffer_.region(0).size / (frame_size_ * 48 * notifs));
        notify_timer_->Arm(zx_deadline_after(ZX_USEC(us_per_notification_)));
    } else {
        us_per_notification_ = 0;
    }
    return ZX_OK;
}

zx_status_t AstroAudioStreamOut::Stop(){
    notify_timer_->Cancel();
    running_ = false;
    aml_audio_->TdmOutDisable();
    aml_audio_->FRDDRDisable();
    return ZX_OK;
}

AstroAudioStreamOut::~AstroAudioStreamOut(void) {}

zx_status_t AstroAudioStreamOut::AddFormats() {
    fbl::AllocChecker ac;
    supported_formats_.reserve(1, & ac);
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

zx_status_t AstroAudioStreamOut::InitBuffer(size_t size) {
    zx_status_t status;
    status = zx_vmo_create_contiguous(bti_.get(), size, 0,
                                      ring_buffer_vmo_.reset_and_get_address());
    if (status != ZX_OK) {
        zxlogf(ERROR, "%s failed to allocate ring buffer vmo - %d\n", __func__, status);
        return status;
    }

    status = pinned_ring_buffer_.Pin(ring_buffer_vmo_, bti_, ZX_VM_PERM_READ | ZX_VM_PERM_WRITE);
    if (status != ZX_OK) {
        zxlogf(ERROR, "%s failed to pin ring buffer vmo - %d\n", __func__, status);
        return status;
    }
    if (pinned_ring_buffer_.region_count() != 1) {
        zxlogf(ERROR, "%s buffer is not contiguous", __func__);
        return ZX_ERR_NO_MEMORY;
    }

    return ZX_OK;
}

} //astro
} //audio

extern "C" zx_status_t audio_bind(void* ctx, zx_device_t* device, void** cookie) {

    __UNUSED auto stream =
        audio::SimpleAudioStream::Create<audio::astro::AstroAudioStreamOut>(device);

    __UNUSED auto dummy = stream.leak_ref();

    return ZX_OK;
}

