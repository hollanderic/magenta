// Copyright 2017 The Fuchsia Authors
// Copyright (c) 2016, Google, Inc. All rights reserved
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#include <reg.h>
#include <err.h>
#include <debug.h>
#include <trace.h>

#include <arch.h>
#include <dev/psci.h>

extern uint64_t psci_smc_call(ulong arg0, ulong arg1, ulong arg2, ulong arg3);


uint32_t psci_get_version(void) {
    return (uint32_t)psci_smc_call(PSCI_PSCI_VERSION,0,0,0);
}

uint32_t psci_get_affinity_info(uint64_t cluster, uint64_t cpuid) {
    return (uint32_t)psci_smc_call(PSCI64_AFFINITY_INFO, PSCI_TARGET(cluster,cpuid),0,0);
}

uint32_t psci_cpu_on(uint64_t cluster, uint64_t cpuid, paddr_t entry) {

    return (uint32_t)psci_smc_call(PSCI64_CPU_ON,(cluster << SMP_CLUSTER_SHIFT) | cpuid,entry,0);

}