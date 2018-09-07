// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <assert.h>
#include <ddktl/mmio.h>
#include <fbl/unique_ptr.h>
#include <soc/aml-common/aml-audio-regs.h>

class AmlPdmDevice {

public:

    DISALLOW_COPY_ASSIGN_AND_MOVE(AmlPdmDevice);

    static constexpr int32_t kMclkDivBits = 16;

    static fbl::unique_ptr<AmlPdmDevice> Create(ddk::MmioBlock pdm_mmio,
                                                ddk::MmioBlock audio_mmio,
                                                ee_audio_mclk_src_t pdm_sysclk_src,
                                                ee_audio_mclk_src_t pdm_dclk_src,
                                                aml_toddr_t toddr_dev);

    //Configure an mclk channel divider
    zx_status_t SetMclkDiv(uint32_t div);

    // Sets the buffer/length pointers for dma engine
    //  must resize in lower 32-bits of address space
    zx_status_t SetBuffer(zx_paddr_t buf, size_t len);

    /*
        Returns offset of dma pointer in the ring buffer
    */
    uint32_t GetRingPosition();

    /*
        Resets state of dma mechanisms and starts clocking data
        onto tdm bus with data fetched from beginning of buffer
    */
    uint64_t Start();

    /*
        Stops clocking data out on the TDM bus
        (physical tdm bus signals remain active)
    */
    void Stop();

    /*
        Synchronize the state of TDM bus signals with fifo/dma engine
    */
    void Sync();

    /*
        Stops the clocking data, shuts down frddr, and quiets output signals
    */
    void Shutdown();

    uint32_t fifo_depth() const { return fifo_depth_;};

private:
    const uint32_t fifo_depth_;
    const aml_toddr_t toddr_ch_;   // fromddr channel used by this instance
    const ee_audio_mclk_src_t sysclk_src_;
    const ee_audio_mclk_src_t dclk_src_;
    const zx_off_t toddr_base_;    // base offset of frddr ch used by this instance
    const ddk::MmioBlock pdm_mmio_;
    const ddk::MmioBlock audio_mmio_;
    friend class fbl::unique_ptr<AmlPdmDevice>;

    AmlPdmDevice(ddk::MmioBlock pdm_mmio, ddk::MmioBlock audio_mmio,
        ee_audio_mclk_src_t sysclk_src, ee_audio_mclk_src_t dclk_src,
        aml_toddr_t toddr, uint32_t fifo_depth) :
        fifo_depth_(fifo_depth),
        toddr_ch_(toddr),
        sysclk_src_(sysclk_src),
        dclk_src_(dclk_src),
        toddr_base_(GetToddrBase(toddr)),
        pdm_mmio_(fbl::move(pdm_mmio)),
        audio_mmio_(fbl::move(audio_mmio)) {};

    ~AmlPdmDevice() = default;

    /* Get the resgister block offset for our ddr block */
    static zx_off_t GetToddrBase(aml_toddr_t ch) {
        switch (ch) {
        case TODDR_A:
            return EE_AUDIO_TODDR_A_CTRL0;
        case TODDR_B:
            return EE_AUDIO_TODDR_B_CTRL0;
        case TODDR_C:
            return EE_AUDIO_TODDR_C_CTRL0;
        }
        //We should never get here, but if we do, make it obvious
        assert(0);
        return 0;
    }

    void AudioClkEna(uint32_t audio_blk_mask);
    void AudioClkDis(uint32_t audio_blk_mask);
    void InitRegs();
    void TODDREnable();
    void TODDRDisable();
    void PdmInDisable();
    void PdmInEnable();

    /* Get the resgister block offset for our ddr block */
    zx_off_t GetToddrOffset(zx_off_t off) {
        return toddr_base_ + off;
    }

};
