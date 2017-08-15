// Copyright 2017 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#include <reg.h>
#include <err.h>
#include <arch/arch_ops.h>
#include <pdev/uart.h>

/* PL011 implementation */
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

#define UARTREG(base, reg)  (*REG32((base)  + (reg)))
#define uart_base (0xffffffffd7e32000)


static int ppputc(char c) {
        /* spin while fifo is full */
    while (UARTREG(uart_base, UART_TFR) & (1<<5))
        ;
    UARTREG(uart_base, UART_DR) = c;

    return 1;
}

static int default_putc(char c) {
    return ppputc(c);
}

static int default_getc(bool wait) {
    return -1;
}

static int default_pputc(char c) {
    return ppputc(c);
}

static int default_pgetc(void) {
    return -1;
}

static const struct pdev_uart_ops default_ops = {
    .putc = default_putc,
    .getc = default_getc,
    .pputc = default_pputc,
    .pgetc = default_pgetc,
};

static const struct pdev_uart_ops* uart_ops = &default_ops;

void uart_init(void) {
}

void uart_init_early(void) {
}

int uart_putc(char c) {
    return uart_ops->putc(c);
}

int uart_getc(bool wait)
{
    return uart_ops->getc(wait);
}

int uart_pputc(char c) {
    return uart_ops->pputc(c);
}

int uart_pgetc(void) {
    return uart_ops->pgetc();
}

void pdev_register_uart(const struct pdev_uart_ops* ops) {
    uart_ops = ops;
    smp_mb();
}
