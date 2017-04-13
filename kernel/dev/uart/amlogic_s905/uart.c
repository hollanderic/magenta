// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2014-2015 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <reg.h>
#include <stdio.h>
#include <trace.h>
#include <lib/cbuf.h>
#include <kernel/thread.h>
#include <dev/interrupt.h>
#include <dev/uart.h>

#include <mdi/mdi.h>
#include <mdi/mdi-defs.h>
#include <pdev/driver.h>
#include <pdev/uart.h>

/* PL011 implementation */
#if 0
#define UART_DR    (0x00)
#define UART_RSR   (0x04)
#define UART_TFR   (0x18)
#define UART_ILPR  (0x20)
#define UART_IBRD  (0x24)
#define UART_FBRD  (0x28)
#define UART_LCRH  (0x2c)
#define UART_CR    (0x30)
#define UART_IFLS  (0x34)
#define UART_IMSC  (0x38)
#define UART_TRIS  (0x3c)
#define UART_TMIS  (0x40)
#define UART_ICR   (0x44)
#define UART_DMACR (0x48)
#endif

#define UART_WFIFO  (0x0)
#define UART_RFIFO  (0x4)
#define UART_CONTROL (0x8)
#define UART_STATUS (0xc)
#define UART_IRQ_CONTROL (0x10)
#define UART_REG5 (0x14)

#define UARTREG(base, reg)  (*REG32((base)  + (reg)))

#define RXBUF_SIZE 128
#define NUM_UART 5

#define UART0_BASE_PHYS          (0xc11084c0)
#define UART1_BASE_PHYS          (0xc11084dc)
#define UART2_BASE_PHYS          (0xc1108700)
#define UART0_AO_BASE_PHYS       (0xc81004c0)
#define UART1_AO_BASE_PHYS       (0xc81004e0)

static uint32_t portnum = 0;
static uintptr_t s905_uart_base = 0;

//static cbuf_t uart_rx_buf[NUM_UART];

static inline uintptr_t uart_to_ptr(unsigned int n)
{
    switch (portnum) {
        default:
        case 0: return paddr_to_kvaddr(UART0_BASE_PHYS)
        case 1: return paddr_to_kvaddr(UART1_BASE_PHYS)
        case 2: return paddr_to_kvaddr(UART2_BASE_PHYS)
        case 3: return paddr_to_kvaddr(UART0_AO_BASE_PHYS)
        case 4: return paddr_to_kvaddr(UART1_AO_BASE_PHYS)
    }
}

#if 0
static enum handler_return uart_irq(void *arg)
{
    bool resched = false;
    uint port = (uintptr_t)arg;
    uintptr_t base = uart_to_ptr(port);

    /* read interrupt status and mask */
    uint32_t isr = UARTREG(base, UART_TMIS);

    if (isr & (1<<4)) { // rxmis
        cbuf_t *rxbuf = &uart_rx_buf[port];

        /* while fifo is not empty, read chars out of it */
        while ((UARTREG(base, UART_TFR) & (1<<4)) == 0) {
            /* if we're out of rx buffer, mask the irq instead of handling it */
            if (cbuf_space_avail(rxbuf) == 0) {
                UARTREG(base, UART_IMSC) &= ~(1<<4); // !rxim
                break;
            }

            char c = UARTREG(base, UART_DR);
            cbuf_write_char(rxbuf, c, false);

            resched = true;
        }
    }

    return resched ? INT_RESCHEDULE : INT_NO_RESCHEDULE;
}
#endif

void s905_uart_init(mdi_node_ref_t* node, uint level)
{
#if 0
    for (size_t i = 0; i < NUM_UART; i++) {
        uintptr_t base = uart_to_ptr(i);

        // create circular buffer to hold received data
        cbuf_initialize(&uart_rx_buf[i], RXBUF_SIZE);

        // assumes interrupts are contiguous
        register_int_handler(UART0_INT + i, &uart_irq, (void *)i);

        // clear all irqs
        UARTREG(base, UART_ICR) = 0x3ff;

        // set fifo trigger level
        UARTREG(base, UART_IFLS) = 0; // 1/8 rxfifo, 1/8 txfifo

        // enable rx interrupt
        UARTREG(base, UART_IMSC) = (1<<4); // rxim

        // enable receive
        UARTREG(base, UART_CR) |= (1<<9); // rxen

        // enable interrupt
        unmask_interrupt(UART0_INT + i);
    }
#endif
}


int s905_uart_putc(char c)
{
    if (!s905_uart_base)
        return 0;

    /* spin while fifo is full */
    while (UARTREG(s905_uart_base, UART_STATUS) & (1<<21))
        ;
    UARTREG(s905_uart_base, UART_WFIFO) = c;

    return 1;
}

int s905_uart_getc(bool wait)
{
#if 0
    cbuf_t *rxbuf = &uart_rx_buf[port];

    char c;
    if (cbuf_read_char(rxbuf, &c, wait) == 1) {
        UARTREG(uart_to_ptr(port), UART_IMSC) = (1<<4); // rxim
        return c;
    }
#endif
    return s905_uart_pgetc();
}

/* panic-time getc/putc */
int s905_uart_pputc(char c)
{
    if (!s905_uart_base)
        return 0;

    /* spin while fifo is full */
    while (UARTREG(s905_uart_base, UART_STATUS) & (1<<21))
        ;
    UARTREG(s905_uart_base, UART_WFIFO) = c;

    return 1;
}

int s905_uart_pgetc(void)
{
    if (!s905_uart_base)
        return 0;

    if ((UARTREG(s905_uart_base, UART_STATUS) & (1<<20)) == 0) {
        return UARTREG(s905_uart_base, UART_RFIFO);
    } else {
        return -1;
    }
}


void uart_flush_tx(int port)
{
}

void uart_flush_rx(int port)
{
}

void uart_init_port(int port, uint baud)
{
}

static const struct pdev_uart_ops s905_uart_ops = {
    .putc = s905_uart_putc,
    .getc = s905_uart_getc,
    .pputc = s905_uart_pputc,
    .pgetc = s905_uart_pgetc,
};

void s905_uart_init_early(mdi_node_ref_t* node, uint level)
{
#if 0
    for (size_t i = 0; i < NUM_UART; i++) {
        UARTREG(uart_to_ptr(i), UART_CR) = (1<<8)|(1<<0); // tx_enable, uarten
    }
#endif
    bool got_port = false;
    uint32_t port_val = 0;

    mdi_node_ref_t child;
    mdi_each_child(node, &child) {
        switch (mdi_id(&child)) {
            case MDI_KERNEL_DRIVERS_S905_UART_PORTNUM:
                if(mdi_node_uint32(&child, &portval) != NO_ERROR)
                    return;
                break;
        }
    }
    s905_uart_base = uart_to_ptr(port_val);
    pdev_register_uart(&s905_uart_ops);
}

LK_PDEV_INIT(s905_uart_init_early, MDI_KERNEL_DRIVERS_S905_UART, s905_uart_init_early, LK_INIT_LEVEL_PLATFORM_EARLY);
LK_PDEV_INIT(s905_uart_init, MDI_KERNEL_DRIVERS_S905_UART, s905_uart_init, LK_INIT_LEVEL_PLATFORM);
