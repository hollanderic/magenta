// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#pragma once



#define CONFIG_DW_GMAC_DEFAULT_DMA_PBL 8

/* Bus mode register definitions */
#define FIXEDBURST         (1 << 16)
#define PRIORXTX_41        (3 << 14)
#define PRIORXTX_31        (2 << 14)
#define PRIORXTX_21        (1 << 14)
#define PRIORXTX_11        (0 << 14)
#define DMA_PBL            (CONFIG_DW_GMAC_DEFAULT_DMA_PBL<<8)
#define RXHIGHPRIO         (1 << 1)
#define DMAMAC_SRST        (1 << 0)

/* Opmode register */
#define STOREFORWARD       (1 << 21)
#define FLUSHTXFIFO        (1 << 20)
#define TXSTART            (1 << 13)
#define ENAFLOWCTL		   (1 << 8)
#define TXSECONDFRAME      (1 << 2)
#define RXSTART            (1 << 1)





#define DESC_TXSTS_OWNBYDMA            (1 << 31)
#define DESC_TXSTS_MSK                 (0x1FFFF << 0)

/* rx status bits definitions */
#define DESC_RXSTS_OWNBYDMA            (1 << 31)
#define DESC_RXSTS_DAFILTERFAIL        (1 << 30)
#define DESC_RXSTS_FRMLENMSK           (0x3FFF << 16)
#define DESC_RXSTS_FRMLENSHFT          (16)

#define DESC_RXSTS_ERROR               (1 << 15)
#define DESC_RXSTS_RXTRUNCATED         (1 << 14)
#define DESC_RXSTS_SAFILTERFAIL        (1 << 13)
#define DESC_RXSTS_RXIPC_GIANTFRAME    (1 << 12)
#define DESC_RXSTS_RXDAMAGED           (1 << 11)
#define DESC_RXSTS_RXVLANTAG           (1 << 10)
#define DESC_RXSTS_RXFIRST             (1 <<  9)
#define DESC_RXSTS_RXLAST              (1 <<  8)
#define DESC_RXSTS_RXIPC_GIANT         (1 <<  7)
#define DESC_RXSTS_RXCOLLISION         (1 <<  6)
#define DESC_RXSTS_RXFRAMEETHER        (1 <<  5)
#define DESC_RXSTS_RXWATCHDOG          (1 <<  4)
#define DESC_RXSTS_RXMIIERROR          (1 <<  3)
#define DESC_RXSTS_RXDRIBBLING         (1 <<  2)
#define DESC_RXSTS_RXCRC               (1 <<  1)

/* tx control bits definitions */
#define DESC_TXCTRL_TXINT              (1 << 31)
#define DESC_TXCTRL_TXLAST             (1 << 30)
#define DESC_TXCTRL_TXFIRST            (1 << 29)
#define DESC_TXCTRL_TXCHECKINSCTRL     (2 << 27)
#define DESC_TXCTRL_TXCRCDIS           (1 << 26)
#define DESC_TXCTRL_TXRINGEND          (1 << 25)
#define DESC_TXCTRL_TXCHAIN            (1 << 24)

#define DESC_TXCTRL_SIZE1MASK          (0x7FF << 0)
#define DESC_TXCTRL_SIZE1SHFT          (0)
#define DESC_TXCTRL_SIZE2MASK          (0x7FF << 11)
#define DESC_TXCTRL_SIZE2SHFT          (11)

/* rx control bits definitions */
#define DESC_RXCTRL_RXINTDIS           (1 << 31)
#define DESC_RXCTRL_RXRINGEND          (1 << 25)
#define DESC_RXCTRL_RXCHAIN            (1 << 24)

#define DESC_RXCTRL_SIZE1MASK          (0x7FF << 0)
#define DESC_RXCTRL_SIZE1SHFT          (0)
#define DESC_RXCTRL_SIZE2MASK          (0x7FF << 11)
#define DESC_RXCTRL_SIZE2SHFT          (11)

/*DMA interrupt enable bits */
#define DMA_INT_NIE                    (1 << 16)
#define DMA_INT_AIE                    (1 << 15)
#define DMA_INT_ERE                    (1 << 14)
#define DMA_INT_FBE                    (1 << 13)
#define DMA_INT_ETE                    (1 << 10)
#define DMA_INT_RWE                    (1 <<  9)
#define DMA_INT_RSE                    (1 <<  8)
#define DMA_INT_RUE                    (1 <<  7)
#define DMA_INT_RIE                    (1 <<  6)
#define DMA_INT_UNE                    (1 <<  5)
#define DMA_INT_OVE                    (1 <<  4)
#define DMA_INT_TJE                    (1 <<  3)
#define DMA_INT_TUE                    (1 <<  2)
#define DMA_INT_TSE                    (1 <<  1)
#define DMA_INT_TIE                    (1 <<  0)


