// Copyright 2016 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

#include <ddk/iotxn.h>
#include <magenta/listnode.h>


mx_status_t dwc_root_hub_init(dwc_t* dwc, int rh_index);
void dwc_root_hub_free(dwc_root_hub_t* rh);
mx_status_t dwc_start_root_hubs(dwc_t* dwc);
mx_status_t dwc_rh_iotxn_queue(dwc_t* dwc, iotxn_t* txn, int rh_index);
void dwc_handle_root_hub_change(dwc_t* dwc);
