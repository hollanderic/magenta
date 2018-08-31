
#include <ddk/debug.h>
#include <ddktl/pdev.h>
#include <math.h>

#include "audio-stream-out.h"

namespace audio {
namespace astro {
/*
    Assumes the GPIO for all resources are configured in the board file
*/
zx_status_t AstroAudioStreamOut::Create(zx_device_t* parent) {

    fbl::AllocChecker ac;

    __UNUSED auto stream = fbl::AdoptRef(new (&ac) AstroAudioStreamOut(parent));
    if (!ac.check()) {
        return ZX_ERR_NO_MEMORY;
    }

    fbl::RefPtr<ddk::Pdev> pdev = ddk::Pdev::Create(parent);

    stream->audio_fault_ = pdev->GetGpio(0);
    stream->audio_en_ = pdev->GetGpio(1);
    if (!(stream->audio_fault_.is_valid() && stream->audio_en_.is_valid())) {
        zxlogf(ERROR,"%s failed to allocate gpio\n", __func__);
        return ZX_ERR_NO_RESOURCES;
    }

    stream->codec_ = Tas27xx::Create(pdev->GetI2cChan(0).release());
    if (!stream->codec_) {
        zxlogf(ERROR,"%s could not get tas27xx\n", __func__);
        return ZX_ERR_NO_RESOURCES;
    }

    pdev->GetBti(0, &stream->bti_);

    stream->aml_audio_ = AmlTdmDevice::Create(pdev->GetMmio(0).release(),
        HIFI_PLL, TDM_OUT_B, FRDDR_B, MCLK_A);
    if (stream->aml_audio_ == nullptr) {
        zxlogf(ERROR,"%s failed to create tdm device\n", __func__);
        return ZX_ERR_NO_MEMORY;
    }

    //Enable Codec
    stream->audio_en_.Write(1);

    //Initialize Codec - Needs to happen prior to bringing tdm clocks online
    stream->codec_->Init();

    //Dummy buffer for testing
    //stream->InitBuffer(4096);
    //zx_paddr_t phys;
    //stream->ring_buffer_->LookupPhys(0, &phys);

    //stream->aml_audio_->SetBuffer(phys, 4096);

    stream->aml_audio_->ConfigTdmOutSlot(3, 3, 31, 15);

    //Setup appropriate tdm clock signals
    stream->aml_audio_->SetMclkDiv(124);

    stream->aml_audio_->SetSclkDiv(1, 0, 127);

    stream->aml_audio_->TdmOutReset();
    stream->aml_audio_->FRDDREnable();
    stream->aml_audio_->TdmOutEnable();

    zxlogf(INFO,"%s created successfully\n",__func__);
    __UNUSED auto dummy = stream.leak_ref();
    return ZX_OK;
}

zx_status_t AstroAudioStreamOut::Init() {
    return ZX_OK;
}

zx_status_t AstroAudioStreamOut::ChangeFormat(const audio_proto::StreamSetFmtReq& req) {
    return ZX_OK;
}

zx_status_t AstroAudioStreamOut::GetBuffer(const audio_proto::RingBufGetBufferReq& req,
                          uint32_t* out_num_rb_frames,
                          zx::vmo* out_buffer){
    return ZX_OK;
}

zx_status_t AstroAudioStreamOut::Start(uint64_t* out_start_time) {
    return ZX_OK;
}

zx_status_t AstroAudioStreamOut::Stop(){
    return ZX_OK;
}

AstroAudioStreamOut::~AstroAudioStreamOut(void) {}


// Test routine for putting a tone in the ring buffer
zx_status_t AstroAudioStreamOut::InitBuffer(size_t size) {
#if 0
    ring_buffer_ = PinnedBuffer::Create(size , bti_, ZX_CACHE_POLICY_CACHED);
    uint16_t *buff = static_cast<uint16_t*>(ring_buffer_->GetBaseAddress());
    for (uint16_t i=0; i < 1024; i++) {
        buff[2*i] = buff[2*i+1]= static_cast<int16_t>(10000*sin(2 * M_PI * i / 128));
    }
    zx_cache_flush(buff, 4096, ZX_CACHE_FLUSH_DATA | ZX_CACHE_FLUSH_INVALIDATE);
#endif
    return ZX_OK;
}



} //astro
} //audio

extern "C" zx_status_t audio_bind(void* ctx, zx_device_t* device, void** cookie) {
    audio::astro::AstroAudioStreamOut::Create(device);
    return ZX_OK;
}

