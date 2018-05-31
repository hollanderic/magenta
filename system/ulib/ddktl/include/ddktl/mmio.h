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

template <typename T>
class MmioBlock {
public:
    static fbl::unique_ptr<MmioBlock<T>> Create(platform_device_protocol_t* pdev,
                                                uint32_t mmio_idx) {

        auto mmio = Create(nullptr, 0, 0);
        if (!mmio) {
            return nullptr;
        }

        zx_status_t res = pdev_map_mmio(pdev, mmio_idx,
            ZX_CACHE_POLICY_UNCACHED_DEVICE, reinterpret_cast<void**>(&mmio->ptr_),
            &mmio->len_, mmio->vmo_.reset_and_get_address());
        if (res != ZX_OK) {
            return nullptr;
        }

        return mmio;
    }

    static fbl::unique_ptr<MmioBlock<T>> Create(void* ptr, zx_off_t offs, size_t len) {
        fbl::AllocChecker ac;
        auto mmio = fbl::unique_ptr<MmioBlock<T>>(
            new (&ac) MmioBlock<T>(ptr, offs, len));
        if (!ac.check()) {
            return nullptr;
        }
        return mmio;
    }

    T Read(zx_off_t offs) {
        assert(offs < len_);
        assert(ptr_);
        return *reinterpret_cast<T*>(ptr_ + offs);
    }

    T ReadMasked(T mask, zx_off_t offs) {
        return (Read(offs) & mask);
    }

    void Write(T val, zx_off_t offs) {
        assert(offs < len_);
        assert(ptr_);
        *reinterpret_cast<T*>(ptr_ + offs) = val;
        hw_mb();
    }

    void SetBits(T mask, zx_off_t offs) {
        T val = Read(offs);
        Write(val | mask, offs);
    }

    void ClearBits(T mask, zx_off_t offs) {
        T val = Read(offs);
        Write(val & ~mask, offs);
    }

    bool isValid() {
        return ((ptr_ != 0) && (len_ != 0));
    }

private:
    MmioBlock(void* ptr, zx_off_t offs, size_t len) {
        ptr_ = (reinterpret_cast<uintptr_t>(ptr) + offs);
        len_ = len;
    }

    uintptr_t ptr_ = 0;
    size_t len_ = 0;
    zx::vmo vmo_;       //Hold a ref if we were created by pdev
};

} //namespace ddk