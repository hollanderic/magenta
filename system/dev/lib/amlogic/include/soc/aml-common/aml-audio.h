// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddktl/mmio.h>
#include <fbl/unique_ptr.h>

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

class AmlTdmDevice : public fbl::unique_ptr<AmlTdmDevice> {

public:
    static fbl::unique_ptr<AmlTdmDevice> Create(ddk::MmioBlock&& mmio,
        ee_audio_mclk_src_t src, aml_tdm_out_t tdm_dev, aml_frddr_t frddr_dev,
        aml_tdm_mclk_t mclk);

    //Configure an mclk channel divider
    zx_status_t SetMclkDiv(uint32_t div);
    //Configure an sclk/lclk generator block
    zx_status_t SetSclkDiv(uint32_t sdiv, uint32_t lrduty, uint32_t lrdiv);

    void AudioClkEna(uint32_t audio_blk_mask);
    void TdmOutDisable();
    void TdmOutEnable();
    void TdmOutReset();
    void ConfigTdmOutSlot(uint8_t bit_offset, uint8_t num_slots,
                    uint8_t bits_per_slot, uint8_t bits_per_sample);
    void SetBuffer(zx_paddr_t buf, size_t len);
    void FRDDREnable();
    void Position(aml_frddr_t ddr);

private:

    friend class fbl::unique_ptr<AmlTdmDevice>;

    AmlTdmDevice() { };

    void InitRegs();

/* Get the resgister block offset for our ddr block */
    zx_off_t GetFrddrOffset(zx_off_t off) {
        switch (frddr_ch_) {
        case FRDDR_A: return EE_AUDIO_FRDDR_A_CTRL0 + off;
        case FRDDR_B: return EE_AUDIO_FRDDR_B_CTRL0 + off;
        case FRDDR_C: return EE_AUDIO_FRDDR_C_CTRL0 + off;
        }
        return EE_AUDIO_FRDDR_A_CTRL0 +off;
    }
/* Get the register block offset for our tdm block */
    zx_off_t GetTdmOffset(zx_off_t off) {
        switch (tdm_ch_) {
        case TDM_OUT_A: return EE_AUDIO_TDMOUT_A_CTRL0 + off;
        case TDM_OUT_B: return EE_AUDIO_TDMOUT_B_CTRL0 + off;
        case TDM_OUT_C: return EE_AUDIO_TDMOUT_C_CTRL0 + off;
        }
        return EE_AUDIO_TDMOUT_A_CTRL0 + off;
    }

    aml_tdm_out_t tdm_ch_;      // tdm output block used by this instance
    aml_frddr_t frddr_ch_;      // fromddr channel used by this instance
    aml_tdm_mclk_t mclk_ch_;    // mclk channel used by this instance
    ee_audio_mclk_src_t clk_src_;

    ddk::MmioBlock mmio_;

    virtual ~AmlTdmDevice();
};
