// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/debug.h>

#include <soc/aml-common/aml-audio.h>

fbl::unique_ptr<AmlTdmDevice> AmlTdmDevice::Create(ddk::MmioBlock&& mmio,
        ee_audio_mclk_src_t src, aml_tdm_out_t tdm_dev, aml_frddr_t frddr_dev,
        aml_tdm_mclk_t mclk) {

    if (!mmio.isMapped()) {
        return nullptr;
    }
    fbl::AllocChecker ac;
    auto tdm = fbl::unique_ptr<AmlTdmDevice>(new (&ac) AmlTdmDevice());
    if (!ac.check()) {
        return nullptr;
    }
    tdm->clk_src_ = src;
    tdm->tdm_ch_ = tdm_dev;
    tdm->frddr_ch_ = frddr_dev;
    tdm->mclk_ch_ = mclk;
    tdm->mmio_ = mmio.release();

    tdm->InitRegs();

    return tdm;
}

void AmlTdmDevice::InitRegs() {
    AudioClkEna((EE_AUDIO_CLK_GATE_TDMOUTA << tdm_ch_) |
                (EE_AUDIO_CLK_GATE_FRDDRA << frddr_ch_) |
                 EE_AUDIO_CLK_GATE_ARB );

    //  Set chosen mclk channels input to selected source
    //  Since this is init, set the divider to max value assuming it will
    //    be set to proper value later (slower is safer from circuit standpoint)
    //  Leave disabled for now.
    zx_off_t ptr = EE_AUDIO_MCLK_A_CTRL + (mclk_ch_ * sizeof(uint32_t));
    mmio_.Write((clk_src_ << 24) | 0xffff, ptr);

    // Set the sclk and lrclk sources to the chosen mclk channel
    ptr = EE_AUDIO_CLK_TDMOUT_A_CTL + tdm_ch_ * sizeof(uint32_t);
    mmio_.Write((0x03 << 30) | (mclk_ch_ << 24) | (mclk_ch_ << 20), ptr);

    //Enable DDR ARB, and enable this ddr channels bit.
    mmio_.SetBits( (1 << 31) | (1 << (4 + frddr_ch_)), EE_AUDIO_ARB_CTRL);

    //Disable the FRDDR Channel
    //Only use one buffer
    //Interrupts off
    //ack delay = 0
    //set destination tdm block and enable that selection
    mmio_.Write(tdm_ch_ | (1 << 3), GetFrddrOffset(FRDDR_CTRL0_OFFS));
    //use 32 levels of fifo, start transfer request when fifo is at 16
    //set the magic force end bit(12) to cause fetch from start
    //  -this only happens when the bit is set from 0->1 (edge)
    mmio_.Write((1 << 12) | (31 << 24) | (15 << 16), GetFrddrOffset(FRDDR_CTRL1_OFFS));

    // Value to be inserted in a slot if it is muted
    mmio_.Write(0x00000000 , GetTdmOffset(TDMOUT_MUTE_VAL_OFFS));
    // Value to be inserted in a slot if it is masked
    mmio_.Write(0x00000000 , GetTdmOffset(TDMOUT_MASK_VAL_OFFS));
}

/* Notes
    -div is desired divider minus 1. (want /100? write 99)
*/
zx_status_t AmlTdmDevice::SetMclkDiv(uint32_t div) {

    zx_off_t ptr = EE_AUDIO_MCLK_A_CTRL + (mclk_ch_ * sizeof(uint32_t));
    // disable and clear out old divider value
    mmio_.ClearBits((1 <<31) | 0xffff, ptr);

    mmio_.SetBits((1 << 31) | (div & 0xffff), ptr);
    return ZX_OK;
}

/* Notes:
    -sdiv is desired divider -1 (Want a divider of 10? write a value of 9)
    -sclk needs to be at least 2x mclk.  writing a value of 0 (/1) to sdiv
        will result in no sclk being generated on the sclk pin.  However, it
        appears that it is running properly as a lrclk is still generated at
        an expected rate (lrclk is derived from sclk)
*/
zx_status_t AmlTdmDevice::SetSclkDiv(uint32_t sdiv,
                                     uint32_t lrduty, uint32_t lrdiv) {
    zx_off_t ptr = EE_AUDIO_MST_A_SCLK_CTRL0 + 2 * mclk_ch_ * sizeof(uint32_t);
    mmio_.Write(    (0x3 << 30) |      //Enable the channel
                    (sdiv << 20) |     // sclk divider sclk=mclk/sdiv
                    (lrduty << 10) |   // lrclk duty cycle in sclk cycles
                    (lrdiv << 0),      // lrclk = sclk/lrdiv
                    ptr);
    mmio_.Write(0, ptr + sizeof(uint32_t));           //Clear delay lines for phases
    return ZX_OK;
}

void AmlTdmDevice::AudioClkEna(uint32_t audio_blk_mask) {
    mmio_.SetBits( audio_blk_mask, EE_AUDIO_CLK_GATE_EN);
}

void AmlTdmDevice::SetBuffer(zx_paddr_t buf, size_t len) {

    //Write the start and end pointers.  Each fetch is 64-bits, so end poitner
    // is pointer to the last 64-bit fetch (inclusive)
    mmio_.Write( static_cast<uint32_t>(buf), GetFrddrOffset(FRDDR_START_ADDR_OFFS));
    mmio_.Write( static_cast<uint32_t>(buf + len - 8),
        GetFrddrOffset(FRDDR_FINISH_ADDR_OFFS));
}

/*
    bit_offset - bit position in frame where first slot will appear
                    (position 0 is concurrent with frame sync)
    num_slots - number of slots per frame minus one
    bits_per_slot - width of each slot minus one
    bits_per_sample - number of bits in sample minus one
*/
void AmlTdmDevice::ConfigTdmOutSlot(uint8_t bit_offset, uint8_t num_slots,
                                    uint8_t bits_per_slot, uint8_t bits_per_sample) {

    uint32_t reg = bits_per_slot | (num_slots << 5) | (bit_offset << 15);
    mmio_.Write(reg , GetTdmOffset(TDMOUT_CTRL0_OFFS));

    reg = (bits_per_sample << 8) | (frddr_ch_ << 24);
    if (bits_per_sample <= 8) {
        // 8 bit sample, left justify in frame, split 64-bit dma fetch into 8 samples
        reg |= (0 << 4);
    } else if (bits_per_sample <= 16) {
        // 16 bit sample, left justify in frame, split 64-bit dma fetch into 2 samples
        reg |= (2 << 4);
    } else {
        // 32/24 bit sample, left justify in slot, split 64-bit dma fetch into 2 samples
        reg |= (4 << 4);
    }
    mmio_.Write(reg, GetTdmOffset(TDMOUT_CTRL1_OFFS));

    // assign left ch to slot 1, right to slot 1
    mmio_.Write(0x00000010 , GetTdmOffset(TDMOUT_SWAP_OFFS));
    // unmask first two slots
    mmio_.Write(0x00000003 , GetTdmOffset(TDMOUT_MASK0_OFFS));

}

void AmlTdmDevice::TdmOutDisable() {
    mmio_.ClearBits(1 << 31, GetTdmOffset(TDMOUT_CTRL0_OFFS));
}
void AmlTdmDevice::TdmOutEnable() {
    mmio_.SetBits(1 << 31, GetTdmOffset(TDMOUT_CTRL0_OFFS));
}

void AmlTdmDevice::FRDDREnable() {
    mmio_.SetBits(1 << 31, GetFrddrOffset(FRDDR_CTRL0_OFFS));
}

void AmlTdmDevice::TdmOutReset() {
    mmio_.ClearBits(3 << 28, GetTdmOffset(TDMOUT_CTRL0_OFFS));
    mmio_.SetBits(1 << 29, GetTdmOffset(TDMOUT_CTRL0_OFFS));
    mmio_.SetBits(1 << 28, GetTdmOffset(TDMOUT_CTRL0_OFFS));
}

void AmlTdmDevice::Position(aml_frddr_t ddr) {
#if 0
    zxlogf(INFO,"DDR CTL0 = %08x\n",mmio_.Read(get_frddr_off(ddr) + FRDDR_CTRL0_OFFS));
    zxlogf(INFO,"DDR CTL1 = %08x\n",mmio_.Read(get_frddr_off(ddr) + FRDDR_CTRL1_OFFS));
    zxlogf(INFO,"STAUS1 = %08x\n",mmio_.Read(get_frddr_off(ddr) + FRDDR_STATUS1_OFFS));
    zxlogf(INFO,"STAUS2 = %08x\n",mmio_.Read(get_frddr_off(ddr) + FRDDR_STATUS2_OFFS));
    zxlogf(INFO,"START = %08x\n",mmio_.Read(get_frddr_off(ddr) + FRDDR_START_ADDR_OFFS));
    zxlogf(INFO,"END = %08x\n",mmio_.Read(get_frddr_off(ddr) + FRDDR_FINISH_ADDR_OFFS));
    zxlogf(INFO,"TDM STAT = %08x\n", mmio_.Read(get_tdm_out_off(TDM_OUT_B) + TDMOUT_STAT_OFFS));
    zxlogf(INFO,"TDM CTL0 = %08x\n", mmio_.Read(get_tdm_out_off(TDM_OUT_B) + TDMOUT_CTRL0_OFFS));
    zxlogf(INFO,"TDM CTL1 = %08x\n", mmio_.Read(get_tdm_out_off(TDM_OUT_B) + TDMOUT_CTRL1_OFFS));
    zxlogf(INFO, "CLK ENA MASK = %08x\n", mmio_.Read(EE_AUDIO_CLK_GATE_EN));
#endif
}

AmlTdmDevice::~AmlTdmDevice() {
}