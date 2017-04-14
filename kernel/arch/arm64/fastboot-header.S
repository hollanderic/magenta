// Copyright 2016 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT


// Optional header included in start.S needed to make fastboot happy
_fastboot_start:
    // magic instruction that gives us UEFI "MZ" signature
    add x13, x18, #0x16
    b header_end

    .quad   KERNEL_LOAD_OFFSET
    .quad   0
    .quad   0

    .quad   0
    .quad   0
    .quad   0
    .byte   'A'
    .byte   'R'
    .byte   'M'
    .byte   0x64
    .long   pe_header - _start
    .align 3

pe_header:


header_end:
    // fall through to arm_reset
