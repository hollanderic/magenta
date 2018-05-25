// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

namespace ddk {

template <typename T>
class HwReg {
public:
    HwReg& operator=(const T val) {

    }
}


template <typename T>
class Mmio {
public:
    Mmio(void* base, size_t len) {
        base_ptr_ = static_cast<T*>(base);
        len_ = len;
    }


private:
    T* base_ptr_;
}


} //namespace ddk