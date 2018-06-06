// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <assert.h>
#include <ddk/protocol/platform-device.h>
#include <fbl/alloc_checker.h>
#include <fbl/unique_ptr.h>
#include <hw/arch_ops.h>
#include <lib/zx/vmo.h>

namespace ddk {

class MmioBlock {

public:
    friend class Pdev;

    uint32_t Read(zx_off_t offs) {
        assert(offs + sizeof(uint32_t) < len_);
        assert(ptr_);
        return *reinterpret_cast<uint32_t*>(ptr_ + offs);
    }

    uint32_t ReadMasked(uint32_t mask, zx_off_t offs) {
        return (Read(offs) & mask);
    }

    void Write(uint32_t val, zx_off_t offs) {
        assert(offs + sizeof(uint32_t) < len_);
        assert(ptr_);
        *reinterpret_cast<uint32_t*>(ptr_ + offs) = val;
        hw_mb();
    }

    void SetBits(uint32_t mask, zx_off_t offs) {
        uint32_t val = Read(offs);
        Write(val | mask, offs);
    }

    void ClearBits(uint32_t mask, zx_off_t offs) {
        uint32_t val = Read(offs);
        Write(val & ~mask, offs);
    }

    bool isValid() {
        return ((ptr_ != 0) && (len_ != 0));
    }
    void* GetRaw() {
        return reinterpret_cast<void*>(ptr_);
    }

    MmioBlock& operator=(MmioBlock&& other) = default;
    MmioBlock() : MmioBlock(nullptr, 0, 0) {}
    DISALLOW_COPY_AND_ASSIGN_ALLOW_MOVE(MmioBlock);

private:
    uintptr_t ptr_;
    size_t len_;

    MmioBlock(MmioBlock&&) = default;

    MmioBlock(void* ptr, zx_off_t offs, size_t len) :
            ptr_(reinterpret_cast<uintptr_t>(ptr) + offs),
            len_(len) {}

    zx::vmo vmo_;
};

} //namespace ddk