
#include <ddk/debug.h>
#include <ddktl/pdev.h>

#include "audio.h"

namespace audio {
namespace astro {

zx_status_t AmlAudioStream::Create(zx_device_t* parent) {

    fbl::AllocChecker ac;

    __UNUSED auto stream = fbl::AdoptRef(new (&ac) AmlAudioStream(parent));
    if (!ac.check()) {
        return ZX_ERR_NO_MEMORY;
    }

    fbl::RefPtr<ddk::Pdev> pdev = ddk::Pdev::Create(parent);

    ddk::MmioBlock mmio;
    mmio = pdev->GetMmio(0);

    if (!mmio.isMapped()) {
        zxlogf(ERROR,"AmlAudio: Failed to allocate mmio\n");
        return ZX_ERR_NO_RESOURCES;
    }

    stream->tdm_ = AmlTdmDevice::Create(mmio.release());

    if (stream->tdm_ == nullptr) {
        zxlogf(ERROR,"%s failed to create tdm device\n",__func__);
        return ZX_ERR_NO_MEMORY;
    }

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

} //astro
} //audio

extern "C" zx_status_t audio_bind(void* ctx, zx_device_t* device, void** cookie) {
    audio::astro::AmlAudioStream::Create(device);
    return ZX_OK;
}

