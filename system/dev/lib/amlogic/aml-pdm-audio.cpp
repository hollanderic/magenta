// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/debug.h>
#include <fbl/limits.h>
#include <soc/aml-common/aml-pdm-audio.h>

//static
fbl::unique_ptr<AmlPdmDevice> AmlPdmDevice::Create(ddk::MmioBlock pdm_mmio,
                                                   ddk::MmioBlock audio_mmio,
                                                   ee_audio_mclk_src_t pdm_sysclk_src,
                                                   ee_audio_mclk_src_t pdm_dclk_src,
                                                   aml_toddr_t toddr_dev) {

    if (!(pdm_mmio.isMapped() && audio_mmio.isMapped())) {
        zxlogf(ERROR, "%s: MmioBlock not initialized!\n", __func__);
        return nullptr;
    }

    //A and B FRDDR have 128 lines in fifo, C has 256
    uint32_t fifo_depth = 128;
    if (toddr_dev == TODDR_A) {
        fifo_depth = 256;
    }

    fbl::AllocChecker ac;
    auto pdm = fbl::unique_ptr<AmlPdmDevice>(new (&ac)
        AmlPdmDevice(fbl::move(pdm_mmio), fbl::move(audio_mmio), pdm_sysclk_src,
                     pdm_dclk_src, toddr_dev, fifo_depth));
    if (!ac.check()) {
        zxlogf(ERROR, "%s: Could not create AmlPdmDevice\n", __func__);
        return nullptr;
    }


    pdm->InitRegs();

    return pdm;
}

void AmlPdmDevice::InitRegs() {

    //TODO - tease out setting of the dividers to different methods
    audio_mmio_.Write32((1 << 31) | (dclk_src_ << 24) | 499, EE_AUDIO_CLK_PDMIN_CTRL0);
    audio_mmio_.Write32((1 << 31) | (sysclk_src_ << 24) | 7, EE_AUDIO_CLK_PDMIN_CTRL1);

    audio_mmio_.SetBits32((1 << 31) | (1 << toddr_ch_), EE_AUDIO_ARB_CTRL);

    //Enable the audio domain clocks used by this instance.
    AudioClkEna(EE_AUDIO_CLK_GATE_PDM |
                (EE_AUDIO_CLK_GATE_TODDRA << toddr_ch_) |
                 EE_AUDIO_CLK_GATE_ARB);


    zxlogf(INFO,"Writing PDM registers....\n");
    pdm_mmio_.Write32(1, PDM_CLKG_CTRL);
    zxlogf(INFO,"Writing PDM registers....\n");

    pdm_mmio_.Write32((1 << 28) | (1 << 8) | (1 << 0), PDM_CTRL);
    zxlogf(INFO,"Writing PDM registers....\n");

    pdm_mmio_.Write32((28 << 0), PDM_CHAN_CTRL);
    zxlogf(INFO,"Writing PDM registers....\n");

    pdm_mmio_.Write32((1 << 28) | (1 <<31) | (1 << 8) | (1 << 0), PDM_CTRL);
    //pdm_mmio_.SetBits32(1 << 31, PDM_CTRL);
    zxlogf(INFO,"Writing PDM registers....\n");

    zxlogf(INFO,"PDM_CTRL = %08x\n", pdm_mmio_.Read32(PDM_CTRL));
#if 0
    //Set chosen mclk channels input to selected source
    //Since this is init, set the divider to max value assuming it will
    //    be set to proper value later (slower is safer from circuit standpoint)
    //Leave disabled for now.
    zx_off_t ptr = EE_AUDIO_MCLK_A_CTRL + (mclk_ch_ * sizeof(uint32_t));
    mmio_.Write32((clk_src_ << 24) | 0xffff, ptr);

    //Set the sclk and lrclk sources to the chosen mclk channel
    ptr = EE_AUDIO_CLK_TDMOUT_A_CTL + tdm_ch_ * sizeof(uint32_t);
    mmio_.Write32((0x03 << 30) | (mclk_ch_ << 24) | (mclk_ch_ << 20), ptr);

    //Enable DDR ARB, and enable this ddr channels bit.
    mmio_.SetBits32((1 << 31) | (1 << (4 + frddr_ch_)), EE_AUDIO_ARB_CTRL);

    //Disable the FRDDR Channel
    //Only use one buffer
    //Interrupts off
    //ack delay = 0
    //set destination tdm block and enable that selection
    mmio_.Write32(tdm_ch_ | (1 << 3), GetFrddrOffset(FRDDR_CTRL0_OFFS));
    //use entire fifo, start transfer request when fifo is at 1/2 full
    //set the magic force end bit(12) to cause fetch from start
    //    -this only happens when the bit is set from 0->1 (edge)
    mmio_.Write32((1 << 12) | ((fifo_depth_ -1) << 24) | (((fifo_depth_ / 2) - 1) << 16),
        GetFrddrOffset(FRDDR_CTRL1_OFFS));

    //Value to be inserted in a slot if it is muted
    mmio_.Write32(0x00000000, GetTdmOffset(TDMOUT_MUTE_VAL_OFFS));
    //Value to be inserted in a slot if it is masked
    mmio_.Write32(0x00000000, GetTdmOffset(TDMOUT_MASK_VAL_OFFS));
#endif
}

/* Notes
    -div is desired divider minus 1. (want /100? write 99)
*/
zx_status_t AmlPdmDevice::SetMclkDiv(uint32_t div) {

    return ZX_OK;
}

uint32_t AmlPdmDevice::GetRingPosition() {

//    return mmio_.Read32(GetFrddrOffset(FRDDR_STATUS2_OFFS)) -
//           mmio_.Read32(GetFrddrOffset(FRDDR_START_ADDR_OFFS));
    return 0;
}

void AmlPdmDevice::AudioClkEna(uint32_t audio_blk_mask) {
    audio_mmio_.SetBits32(audio_blk_mask, EE_AUDIO_CLK_GATE_EN);
}

void AmlPdmDevice::AudioClkDis(uint32_t audio_blk_mask) {
    audio_mmio_.ClearBits32(audio_blk_mask, EE_AUDIO_CLK_GATE_EN);
}

zx_status_t AmlPdmDevice::SetBuffer(zx_paddr_t buf, size_t len) {
    //Ensure ring buffer resides in lower memory (dma pointers are 32-bit)
    //    and len is at least 8 (size of each dma operation)
    if (((buf + len - 1) > fbl::numeric_limits<uint32_t>::max()) || (len < 8)) {
        return ZX_ERR_INVALID_ARGS;
    }

    //Write32 the start and end pointers.  Each fetch is 64-bits, so end poitner
    //    is pointer to the last 64-bit fetch (inclusive)
    //mmio_.Write32(static_cast<uint32_t>(buf), GetFrddrOffset(FRDDR_START_ADDR_OFFS));
    //mmio_.Write32(static_cast<uint32_t>(buf + len - 8),
    //            GetFrddrOffset(FRDDR_FINISH_ADDR_OFFS));
    return ZX_OK;
}


// Stops the tdm from clocking data out of fifo onto bus
void AmlPdmDevice::PdmInDisable() {
    //mmio_.ClearBits32(1 << 31, GetTdmOffset(TDMOUT_CTRL0_OFFS));
}
// Enables the tdm to clock data out of fifo onto bus
void AmlPdmDevice::PdmInEnable() {
    //mmio_.SetBits32(1 << 31, GetTdmOffset(TDMOUT_CTRL0_OFFS));
}

void AmlPdmDevice::TODDREnable() {
    //Set the load bit, will make sure things start from beginning of buffer
    //mmio_.SetBits32(1 << 12, GetFrddrOffset(FRDDR_CTRL1_OFFS));
    //mmio_.SetBits32(1 << 31, GetFrddrOffset(FRDDR_CTRL0_OFFS));
}

void AmlPdmDevice::TODDRDisable() {
    // Clear the load bit (this is the bit that forces the initial fetch of
    //    start address into current ptr)
    //mmio_.ClearBits32(1 << 12, GetFrddrOffset(FRDDR_CTRL1_OFFS));
    // Disable the frddr channel
    //mmio_.ClearBits32(1 << 31, GetFrddrOffset(FRDDR_CTRL0_OFFS));
}

void AmlPdmDevice::Sync() {
    //mmio_.ClearBits32(3 << 28, GetTdmOffset(TDMOUT_CTRL0_OFFS));
    //mmio_.SetBits32(1 << 29, GetTdmOffset(TDMOUT_CTRL0_OFFS));
    //mmio_.SetBits32(1 << 28, GetTdmOffset(TDMOUT_CTRL0_OFFS));
}

// Resets frddr mechanisms to start at beginning of buffer
//   starts the frddr (this will fill the fifo)
//   starts the tdm to clock out data on the bus
// returns the start time
uint64_t AmlPdmDevice::Start() {
    uint64_t a, b;

    Sync();
    TODDREnable();
    a = zx_clock_get(ZX_CLOCK_MONOTONIC);
    PdmInEnable();
    b = zx_clock_get(ZX_CLOCK_MONOTONIC);
    return ((b - a) >> 1) + a;
}

void AmlPdmDevice::Stop() {
    PdmInDisable();
    TODDRDisable();
}

void AmlPdmDevice::Shutdown() {
    Stop();

    // Disable the output signals
    //zx_off_t ptr = EE_AUDIO_CLK_TDMOUT_A_CTL + tdm_ch_ * sizeof(uint32_t);
    //mmio_.ClearBits32(0x03 << 30, ptr);

    // Disable the audio domain clocks used by this instance.
    //AudioClkDis((EE_AUDIO_CLK_GATE_TDMOUTA << tdm_ch_) |
    //            (EE_AUDIO_CLK_GATE_FRDDRA << frddr_ch_));

    //Note: We are leaving the ARB unit clocked as well as MCLK and
    //  SCLK generation units since it is possible they are used by
    //  some other audio driver outside of this instance
}