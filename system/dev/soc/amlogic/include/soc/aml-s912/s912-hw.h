// Copyright 2017 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once

// DMC registers
#define DMC_REG_BASE        0xc8838000

#define PERIPHS_REG_BASE        (0xc8834000)
#define PERIPHS_REG_SIZE        (0x2000)
//Offsets of peripheral control registers
#define PER_ETH_REG0            (0x400 + (0x50 << 2))
#define PER_ETH_REG1            (0x400 + (0x51 << 2))
#define PER_ETH_REG2            (0x400 + (0x56 << 2))
#define PER_ETH_REG3            (0x400 + (0x57 << 2))
#define PER_ETH_REG4            (0x400 + (0x58 << 2))


#define ETH_MAC_REG_BASE         (0xc9410000)
#define ETH_MAC_REG_SIZE         (0x00010000)

#define DMC_CAV_LUT_DATAL           (0x12 << 2)
#define DMC_CAV_LUT_DATAH           (0x13 << 2)
#define DC_CAV_LUT_ADDR             (0x14 << 2)

#define DC_CAV_LUT_ADDR_INDEX_MASK  0x7
#define DC_CAV_LUT_ADDR_RD_EN       (1 << 8)
#define DC_CAV_LUT_ADDR_WR_EN       (2 << 8)
// Alternate Functions for Ethernet
#define S912_ETH_MDIO       S912_GPIOZ(0)
#define S912_ETH_MDIO_FN    1
#define S912_ETH_MDC        S912_GPIOZ(1)
#define S912_ETH_MDC_FN     1

#define S912_ETH_RGMII_RX_CLK        S912_GPIOZ(2)
#define S912_ETH_RGMII_RX_CLK_FN     1
#define S912_ETH_RX_DV               S912_GPIOZ(3)
#define S912_ETH_RX_DV_FN            1
#define S912_ETH_RXD0                S912_GPIOZ(4)
#define S912_ETH_RXD0_FN             1
#define S912_ETH_RXD1                S912_GPIOZ(5)
#define S912_ETH_RXD1_FN             1
#define S912_ETH_RXD2                S912_GPIOZ(6)
#define S912_ETH_RXD2_FN             1
#define S912_ETH_RXD3                S912_GPIOZ(7)
#define S912_ETH_RXD3_FN             1

#define S912_ETH_RGMII_TX_CLK        S912_GPIOZ(8)
#define S912_ETH_RGMII_TX_CLK_FN     1
#define S912_ETH_TX_EN               S912_GPIOZ(9)
#define S912_ETH_TX_EN_FN            1
#define S912_ETH_TXD0                S912_GPIOZ(10)
#define S912_ETH_TXD0_FN             1
#define S912_ETH_TXD1                S912_GPIOZ(11)
#define S912_ETH_TXD1_FN             1
#define S912_ETH_TXD2                S912_GPIOZ(12)
#define S912_ETH_TXD2_FN             1
#define S912_ETH_TXD3                S912_GPIOZ(13)
#define S912_ETH_TXD3_FN             1

// Alternate Functions for I2C
#define S912_I2C_SDA_A      S912_GPIODV(24)
#define S912_I2C_SDA_A_FN   2
#define S912_I2C_SCK_A      S912_GPIODV(25)
#define S912_I2C_SCK_A_FN   2

#define S912_I2C_SDA_B      S912_GPIODV(26)
#define S912_I2C_SDA_B_FN   2
#define S912_I2C_SCK_B      S912_GPIODV(27)
#define S912_I2C_SCK_B_FN   2

#define S912_I2C_SDA_C      S912_GPIODV(28)
#define S912_I2C_SDA_C_FN   2
#define S912_I2C_SCK_C      S912_GPIODV(29)
#define S912_I2C_SCK_C_FN   2

#define S912_I2C_SDA_D      S912_GPIOX(10)
#define S912_I2C_SDA_D_FN   3
#define S912_I2C_SCK_D      S912_GPIOX(11)
#define S912_I2C_SCK_D_FN   3

#define S912_I2C_SDA_AO     S912_GPIOAO(4)
#define S912_I2C_SDA_AO_FN  2
#define S912_I2C_SCK_AO     S912_GPIOAO(5)
#define S912_I2C_SCK_AO_FN  2
