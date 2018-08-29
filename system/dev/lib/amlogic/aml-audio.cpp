// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/debug.h>

#include <soc/aml-common/aml-audio.h>

fbl::unique_ptr<AmlAudioDevice> AmlAudioDevice::Create(ddk::MmioBlock&& mmio) {

    if (!mmio.isMapped()) {
        return nullptr;
    }
    auto tdm_dev = fbl::unique_ptr<AmlAudioDevice>(new AmlAudioDevice());

    tdm_dev->mmio_ = mmio.release();

    tdm_dev->InitRegs();

    return tdm_dev;
}


/* Notes
    -div is desired divider minus 1. (want /100? write 99)
*/
zx_status_t AmlAudioDevice::SetMclk(aml_tdm_mclk_t ch, ee_audio_mclk_src_t src, uint32_t div) {
    zx_off_t ptr = EE_AUDIO_MCLK_A_CTRL + (ch * sizeof(uint32_t));
    mmio_.Write(EE_AUDIO_MCLK_ENA | (src << 24) | (div & 0xffff), ptr);
    return ZX_OK;
}

/* Notes:
    -sdiv is desired divider -1 (Want a divider of 10? write a value of 9)
    -sclk needs to be at least 2x mclk.  writing a value of 0 (/1) to sdiv
        will result in no sclk being generated on the sclk pin.  However, it
        appears that it is running properly as a lrclk is still generated at
        an expected rate (lrclk is derived from sclk)
*/
zx_status_t AmlAudioDevice::SetSclk(uint32_t ch, uint32_t sdiv,
                                  uint32_t lrduty, uint32_t lrdiv) {
    zx_off_t ptr = EE_AUDIO_MST_A_SCLK_CTRL0 + 2 * ch * sizeof(uint32_t);
    mmio_.Write(    (0x3 << 30) |      //Enable the channel
                    (sdiv << 20) |     // sclk divider sclk=mclk/sdiv
                    (lrduty << 10) |   // lrclk duty cycle in sclk cycles
                    (lrdiv << 0),      // lrclk = sclk/lrdiv
                    ptr);
    mmio_.Write(0, ptr + sizeof(uint32_t));           //Clear delay lines for phases
    return ZX_OK;
}

zx_status_t AmlAudioDevice::SetTdmOutClk(aml_tdm_out_t tdm_blk, aml_tdm_mclk_t sclk_src,
                                       aml_tdm_mclk_t lrclk_src, bool inv) {

    zx_off_t ptr = EE_AUDIO_CLK_TDMOUT_A_CTL + tdm_blk * sizeof(uint32_t);
    mmio_.Write( (0x3 << 30) | //Enable the clock
                 (inv ? (1 << 29) : 0) | //invert sclk
                 (sclk_src << 24) |
                 (lrclk_src << 20), ptr);
    return ZX_OK;
}

void AmlAudioDevice::AudioClkEna(uint32_t audio_blk_mask) {
    mmio_.SetBits( audio_blk_mask, EE_AUDIO_CLK_GATE_EN);
}



void AmlAudioDevice::InitRegs() {
    //uregs_->SetBits(0x00002000, AML_TDM_CLK_GATE_EN);
}

void AmlAudioDevice::ConfigFRDDR(aml_frddr_t ddr, aml_tdm_out_t tdm, zx_paddr_t buf, size_t len) {
    //Enable DDR ARB, and enable this ddr channels bit.
    mmio_.Write( (1 << 31) | (1 << (4 + ddr)), EE_AUDIO_ARB_CTRL);

    //Disable the FRDDR Channel
    //Only use one buffer
    //Interrupts off
    //ack delay = 0
    //send to selected tdm block
    mmio_.Write(tdm | (1 << 3), get_frddr_off(ddr) + FRDDR_CTRL0_OFFS);

    //set tdm block to use this ddr
    //uint32_t reg = mmio_.Read(get_tdm_out_off(tdm) +  TDMOUT_CTRL1_OFFS);
    //reg = (reg & ~(0x3 << 24)) | (ddr << 24);
    //mmio_.Write(reg, get_tdm_out_off(tdm) +  TDMOUT_CTRL1_OFFS);

    //use 64 levels of fifo, start transfer request when fifo is at 32
    //set the magic force end bit to cause fetch from start????
    mmio_.Write((1 << 12) | (31 << 24) | (15 << 16), get_frddr_off(ddr) + FRDDR_CTRL1_OFFS);

    //Write the start and end pointers.  Each fetch is 64-bits, so end poitner
    // is pointer to the last 64-bit fetch (inclusive)
    mmio_.Write( static_cast<uint32_t>(buf), get_frddr_off(ddr) + FRDDR_START_ADDR_OFFS);
    mmio_.Write( static_cast<uint32_t>(buf + len - 8),
        get_frddr_off(ddr) + FRDDR_FINISH_ADDR_OFFS);
}

/*
    bit_offset - bit position in frame where first slot will appear
                    (position 0 is concurrent with frame sync)
    num_slots - number of slots per frame minus one
    bits_per_slot - width of each slot minus one
    bits_per_sample - number of bits in sample minus one
*/
void AmlAudioDevice::ConfigTdmOutSlot(aml_tdm_out_t tdm_blk, uint8_t bit_offset,
                                uint8_t num_slots, uint8_t bits_per_slot,
                                uint8_t bits_per_sample) {

    uint32_t reg = bits_per_slot | (num_slots << 5) | (bit_offset << 15);
    mmio_.Write(reg , get_tdm_out_off(tdm_blk) + TDMOUT_CTRL0_OFFS);
    mmio_.Write(reg , get_tdm_out_off(tdm_blk) + TDMOUT_CTRL0_OFFS);

    reg = (bits_per_sample << 8) | (FRDDR_B << 24);
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

    mmio_.Write(reg, get_tdm_out_off(tdm_blk) + TDMOUT_CTRL1_OFFS);

    // assign left ch to slot 1, right to slot 1
    mmio_.Write(0x00000010 , get_tdm_out_off(tdm_blk) + TDMOUT_SWAP_OFFS);
    // unmask first two slots
    mmio_.Write(0x00000003 , get_tdm_out_off(tdm_blk) + TDMOUT_MASK0_OFFS);
    // Value to be inserted in a slot if it is muted
    mmio_.Write(0x00000000 , get_tdm_out_off(tdm_blk) + TDMOUT_MUTE_VAL_OFFS);
    // Value to be inserted in a slot if it is masked
    mmio_.Write(0x00000000 , get_tdm_out_off(tdm_blk) + TDMOUT_MASK_VAL_OFFS);

}

void AmlAudioDevice::TdmOutDisable(aml_tdm_out_t tdm_blk) {
    mmio_.ClearBits(1 << 31, get_tdm_out_off(tdm_blk) + TDMOUT_CTRL0_OFFS);
}
void AmlAudioDevice::TdmOutEnable(aml_tdm_out_t tdm_blk) {
    mmio_.SetBits(1 << 31, get_tdm_out_off(tdm_blk) + TDMOUT_CTRL0_OFFS);
}

void AmlAudioDevice::FRDDREnable(aml_frddr_t ddr) {
    mmio_.SetBits(1 << 31, get_frddr_off(ddr) + FRDDR_CTRL0_OFFS);
}

void AmlAudioDevice::TdmOutReset(aml_tdm_out_t tdm_blk) {
    mmio_.ClearBits(3 << 28, get_tdm_out_off(tdm_blk) + TDMOUT_CTRL0_OFFS);
    mmio_.SetBits(1 << 29, get_tdm_out_off(tdm_blk) + TDMOUT_CTRL0_OFFS);
    mmio_.SetBits(1 << 28, get_tdm_out_off(tdm_blk) + TDMOUT_CTRL0_OFFS);
}

void AmlAudioDevice::Position(aml_frddr_t ddr) {
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

}

AmlAudioDevice::~AmlAudioDevice() {
}