
#include <ddk/debug.h>
#include <ddktl/pdev.h>
#include <math.h>

#include "audio.h"

namespace audio {
namespace astro {
/*
    Assumes the GPIO for all resources are configured in the board file
*/
zx_status_t AmlAudioStream::Create(zx_device_t* parent) {

    fbl::AllocChecker ac;

    __UNUSED auto stream = fbl::AdoptRef(new (&ac) AmlAudioStream(parent));
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

    for (uint8_t i = 0; i<20; i++) {
        zxlogf(INFO,"TAS reg%02x = %02x\n", i, stream->codec_->ReadReg(i));
    }

    stream->audio_en_.Write(1);
    stream->codec_->Init();


    pdev->GetBti(0, &stream->bti_);

    stream->aml_audio_ = AmlAudioDevice::Create(pdev->GetMmio(0).release());
    if (stream->aml_audio_ == nullptr) {
        zxlogf(ERROR,"%s failed to create tdm device\n", __func__);
        return ZX_ERR_NO_MEMORY;
    }

    stream->InitBuffer(4096);
    zx_paddr_t phys;
    stream->ring_buffer_->LookupPhys(0, &phys);

    stream->aml_audio_->ConfigFRDDR(FRDDR_B, TDM_OUT_B,
                        phys, 4096);

    stream->aml_audio_->ConfigTdmOutSlot(TDM_OUT_B, 3, 3, 31, 15);

    //Setup appropriate tdm clock signals
    stream->aml_audio_->SetMclk(MCLK_A, HIFI_PLL, 124);

    stream->aml_audio_->SetSclk(MCLK_A, 1, 0, 127);

    stream->aml_audio_->SetTdmOutClk(TDM_OUT_B, MCLK_A, MCLK_A, false);


    stream->aml_audio_->AudioClkEna(EE_AUDIO_CLK_GATE_TDMOUTB |
                                    EE_AUDIO_CLK_GATE_FRDDRB | 1 );

    zx_nanosleep(zx_deadline_after(ZX_MSEC(20)));

    stream->aml_audio_->TdmOutReset(TDM_OUT_B);
    stream->aml_audio_->FRDDREnable(FRDDR_B);
    stream->aml_audio_->TdmOutEnable(TDM_OUT_B);

    zxlogf(INFO,"%s created successfully\n",__func__);
    __UNUSED auto dummy = stream.leak_ref();
    return ZX_OK;
}

    // DDK device implementation
void AmlAudioStream::DdkUnbind() {}
void AmlAudioStream::DdkRelease() {}

zx_status_t AmlAudioStream::DdkIoctl(uint32_t op,
                         const void* in_buf, size_t in_len,
                         void* out_buf, size_t out_len, size_t* out_actual)
{
    return ZX_OK;
}

AmlAudioStream::~AmlAudioStream(void) {}

zx_status_t AmlAudioStream::InitBuffer(size_t size) {
    ring_buffer_ = PinnedBuffer::Create(size , bti_, ZX_CACHE_POLICY_CACHED);
    uint16_t *buff = static_cast<uint16_t*>(ring_buffer_->GetBaseAddress());
    for (uint16_t i=0; i < 1024; i++) {
        //buff[2*i] = static_cast<uint16_t>(i*64);
        //buff[2*i + 1] = static_cast<uint16_t>(i*64);
#if 0
        buff[2*i] = 0;
        buff[2*i+1] = 0;

        buff[512+2*i] = 0;
        buff[512+2*i+1] = 0;

        buff[1024+2*i] = 0x8000;
        buff[1024+2*i+1] = 0x8000;

        buff[1536+2*i] = 0x7fff;
        buff[1536+2*i+1] = 0x7fff;
#endif
        buff[2*i] = buff[2*i+1]= static_cast<int16_t>(10000*sin(2 * M_PI * i / 128));
        //zxlogf(INFO,"%d\n",buff[2*i]);
    }
    zx_cache_flush(buff, 4096, ZX_CACHE_FLUSH_DATA | ZX_CACHE_FLUSH_INVALIDATE);
    return ZX_OK;
}



} //astro
} //audio

extern "C" zx_status_t audio_bind(void* ctx, zx_device_t* device, void** cookie) {
    audio::astro::AmlAudioStream::Create(device);
    return ZX_OK;
}

