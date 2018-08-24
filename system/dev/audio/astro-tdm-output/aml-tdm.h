// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/io-buffer.h>
#include <ddk/protocol/i2c.h>
#include <ddk/protocol/platform-device.h>
#include <ddktl/device.h>
#include <ddktl/device-internal.h>
#include <ddktl/mmio.h>
#include <zircon/listnode.h>
#include <lib/zx/bti.h>
#include <lib/zx/vmo.h>
#include <fbl/mutex.h>
#include <fbl/ref_ptr.h>
#include <fbl/unique_ptr.h>
#include <fbl/vector.h>

#include <soc/aml-common/aml-audio.h>

class AmlAudioDevice : public fbl::unique_ptr<AmlAudioDevice> {

public:
    static fbl::unique_ptr<AmlAudioDevice> Create(ddk::MmioBlock&& mmio);

    //Configure an mclk channel (a..f) with source and divider
    zx_status_t SetMclk(aml_tdm_mclk_t ch, ee_audio_mclk_src_t src, uint32_t div);
    //Configure an sclk/lclk generator block
    zx_status_t SetSclk(uint32_t ch, uint32_t sdiv,
                        uint32_t lrduty, uint32_t lrdiv);
    //Configure signals driving the output block (sclk, lrclk)
    zx_status_t SetTdmOutClk(aml_tdm_out_t tdm_blk, aml_tdm_mclk_t sclk_src,
                                       aml_tdm_mclk_t lrclk_src, bool inv);

    void AudioClkEna(uint32_t audio_blk_mask);
    void TdmOutDisable(aml_tdm_out_t tdm_blk);
    void TdmOutEnable(aml_tdm_out_t tdm_blk);
    void TdmOutReset(aml_tdm_out_t tdm_blk);

private:
    //static int IrqThread(void* arg);

    friend class fbl::unique_ptr<AmlAudioDevice>;

    AmlAudioDevice() { };

    void InitRegs();

    uint32_t tdm_out_ch_;    //Which tdm output block this instance uses
    uint32_t frddr_ch_;     //which fromddr channel is used by this instance

    ddk::MmioBlock mmio_;

    virtual ~AmlAudioDevice();

#if 0
    fbl::Mutex lock_;
    fbl::Mutex req_lock_ __TA_ACQUIRED_AFTER(lock_);

    // Dispatcher framework state
    fbl::RefPtr<dispatcher::Channel> stream_channel_ __TA_GUARDED(lock_);
    fbl::RefPtr<dispatcher::Channel> rb_channel_     __TA_GUARDED(lock_);
    fbl::RefPtr<dispatcher::ExecutionDomain> default_domain_;

    uint32_t ring_buffer_phys_  = 0;
    uint32_t ring_buffer_size_  = 0;
#endif
};

/* Get the resgister block offset for a given tdm out block */
static inline zx_off_t get_tdm_out_off(aml_tdm_out_t tdm_blk) {
    switch (tdm_blk) {
        case TDM_OUT_A: return EE_AUDIO_TDMOUT_A_CTRL0;
        case TDM_OUT_B: return EE_AUDIO_TDMOUT_B_CTRL0;
        case TDM_OUT_C: return EE_AUDIO_TDMOUT_C_CTRL0;
    }
    return EE_AUDIO_TDMOUT_A_CTRL0;
}