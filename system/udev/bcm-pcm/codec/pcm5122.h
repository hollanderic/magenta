// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <magenta/device/audio2.h>

#pragma once



mx_status_t pcm5122_init(void);
bool pcm5122_is_valid_mode(audio2_stream_cmd_set_format_req_t req);