// Copyright 2017 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#pragma once

#define SDRAM_BASE 0x80000000

#define MEMORY_APERTURE_SIZE    (30ULL * 1024 * 1024 * 1024)

//#define iframe arm64_iframe_long


/* map all of 0-1GB into kernel space in one shot */
#define PERIPHERAL_BASE_PHYS (0)
#define PERIPHERAL_BASE_SIZE (0x40000000UL) // 1GB

#define PERIPHERAL_BASE_VIRT (0xffffffffc0000000ULL) // -1GB

/* individual peripherals in this mapping */
#define CPUPRIV_BASE_VIRT   (PERIPHERAL_BASE_VIRT + 0x08000000)
#define CPUPRIV_BASE_PHYS   (PERIPHERAL_BASE_PHYS + 0x08000000)
#define CPUPRIV_SIZE        (0x00020000)
#define UART_BASE           (PERIPHERAL_BASE_VIRT + 0x09000000)
#define UART_SIZE           (0x00001000)
#define RTC_BASE            (PERIPHERAL_BASE_VIRT + 0x09010000)
#define RTC_SIZE            (0x00001000)
#define FW_CFG_BASE         (PERIPHERAL_BASE_VIRT + 0x09020000)
#define FW_CFG_SIZE         (0x00001000)
#define NUM_VIRTIO_TRANSPORTS 32
#define VIRTIO_BASE         (PERIPHERAL_BASE_VIRT + 0x0a000000)
#define VIRTIO_SIZE         (NUM_VIRTIO_TRANSPORTS * 0x200)
#define PCIE_MMIO_BASE_PHYS ((paddr_t)(PERIPHERAL_BASE_PHYS + 0x10000000))
#define PCIE_MMIO_SIZE      (0x2eff0000)
#define PCIE_PIO_BASE_PHYS  ((paddr_t)(PERIPHERAL_BASE_PHYS + 0x3eff0000))
#define PCIE_PIO_SIZE       (0x00010000)
#define PCIE_ECAM_BASE_PHYS ((paddr_t)(PERIPHERAL_BASE_PHYS + 0x3f000000))
#define PCIE_ECAM_SIZE      (0x01000000)
#define GICV2M_FRAME_PHYS   (PERIPHERAL_BASE_PHYS + 0x08020000)


/* interrupts */
#define ARM_GENERIC_TIMER_VIRTUAL_INT 27
#define ARM_GENERIC_TIMER_PHYSICAL_INT 30
#define UART0_INT       (32 + 1)
#define PCIE_INT_BASE   (32 + 3)
#define PCIE_INT_COUNT  (4)
#define VIRTIO0_INT     (32 + 16)

#define MAX_INT 128
