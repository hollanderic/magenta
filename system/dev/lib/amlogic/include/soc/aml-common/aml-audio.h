// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/io-buffer.h>
#include <ddk/protocol/i2c.h>
#include <ddk/protocol/platform-device.h>
#include <ddktl/mmio.h>
#include <lib/zx/bti.h>
#include <lib/zx/vmo.h>
#include <fbl/mutex.h>
#include <fbl/ref_ptr.h>
#include <fbl/unique_ptr.h>
#include <fbl/vector.h>

#include <soc/aml-common/aml-audio-regs.h>

typedef enum {
    MCLK_A = 0,
    MCLK_B,
    MCLK_C,
    MCLK_D,
    MCLK_E,
    MCLK_F
} aml_tdm_mclk_t;

typedef enum {
    TDM_OUT_A = 0,
    TDM_OUT_B,
    TDM_OUT_C
} aml_tdm_out_t;

typedef enum {
    FRDDR_A = 0,
    FRDDR_B,
    FRDDR_C
} aml_frddr_t;


/* Get the resgister block offset for a given tdm out block */
static inline zx_off_t get_tdm_out_off(aml_tdm_out_t tdm_blk) {
    switch (tdm_blk) {
        case TDM_OUT_A: return EE_AUDIO_TDMOUT_A_CTRL0;
        case TDM_OUT_B: return EE_AUDIO_TDMOUT_B_CTRL0;
        case TDM_OUT_C: return EE_AUDIO_TDMOUT_C_CTRL0;
    }
    return EE_AUDIO_TDMOUT_A_CTRL0;
}

/* Get the resgister block offset for a given tdm out block */
static inline zx_off_t get_frddr_off(aml_frddr_t frddr) {
    switch (frddr) {
        case FRDDR_A: return EE_AUDIO_FRDDR_A_CTRL0;
        case FRDDR_B: return EE_AUDIO_FRDDR_B_CTRL0;
        case FRDDR_C: return EE_AUDIO_FRDDR_C_CTRL0;
    }
    return EE_AUDIO_FRDDR_A_CTRL0;
}

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
    void ConfigTdmOutSlot(aml_tdm_out_t tdm_blk, aml_frddr_t ddr,
                    uint8_t bit_offset, uint8_t num_slots,
                    uint8_t bits_per_slot, uint8_t bits_per_sample);
    void ConfigFRDDR(aml_frddr_t ddr, aml_tdm_out_t tdm,
                    zx_paddr_t buf, size_t len);
    void FRDDREnable(aml_frddr_t ddr);
    void Position(aml_frddr_t ddr);

private:

    friend class fbl::unique_ptr<AmlAudioDevice>;

    AmlAudioDevice() { };

    void InitRegs();

    uint32_t tdm_out_ch_;    //tdm output block used by this instance
    uint32_t frddr_ch_;     // fromddr channel used by this instance

    ddk::MmioBlock mmio_;

    virtual ~AmlAudioDevice();
};
