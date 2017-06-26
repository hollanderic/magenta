// Copyright 2016 The Fuchsia Authors
// Copyright (c) 2014-2015 Travis Geiselbrecht
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <reg.h>
#include <stdio.h>
#include <trace.h>
#include <string.h>
#include <lib/cbuf.h>
#include <kernel/thread.h>
#include <dev/interrupt.h>
#include <dev/uart.h>

#include <mdi/mdi.h>
#include <mdi/mdi-defs.h>
#include <pdev/driver.h>
#include <pdev/uart.h>

/* PL011 implementation */
#if 1
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



#define UARTREG(base, reg)  (*(volatile uint32_t*)((base)  + (reg)))

#define RXBUF_SIZE 128
#define NUM_UART 5

#define UART0_BASE_PHYS          (0xc11084c0)
#define UART1_BASE_PHYS          (0xc11084dc)
#define UART2_BASE_PHYS          (0xc1108700)
#define UART0_AO_BASE_PHYS       (0xc81004c0)
#define UART1_AO_BASE_PHYS       (0xc81004e0)


#define UART0_A0_INT            (225)

static cbuf_t uart_rx_buf;
static bool initialized = false;
static uintptr_t s905_uart_base = 0;

//static cbuf_t uart_rx_buf[NUM_UART];

static void uput(char c) {
    volatile uint32_t* wreg = (uint32_t*)(0xffffffffc81004c0 + 0x00);
    volatile uint32_t* sreg = (uint32_t*)(0xffffffffc81004c0 + 0x0C);
    while (*sreg & (1<<21))
        ;
    *wreg = c;
}

static void sput(const char* s,int len){
    for (int i=0;i<len;i++)
        uput(s[i]);
    uput('\r');
    uput('\n');
}



static inline uintptr_t uart_to_ptr(unsigned int n)
{
    switch (n) {
        case 0: return (uintptr_t)paddr_to_kvaddr(UART0_BASE_PHYS);
        case 1: return (uintptr_t)paddr_to_kvaddr(UART1_BASE_PHYS);
        case 2: return (uintptr_t)paddr_to_kvaddr(UART2_BASE_PHYS);
        case 4: return (uintptr_t)paddr_to_kvaddr(UART1_AO_BASE_PHYS);
        default:
        case 3: return (uintptr_t)paddr_to_kvaddr(UART0_AO_BASE_PHYS);

    }
}

#if 1
static enum handler_return uart_irq(void *arg)
{
    sput("INTERRRRRRUUUUUUUUPPPPPPPPTTTTTTT",30);
    uint port = (uintptr_t)arg;
    uintptr_t base = uart_to_ptr(port);

    /* read interrupt status and mask */

    while ( (UARTREG(base, UART_STATUS)&0x0000000f) > 0 ) { // rxmis
        if (cbuf_space_avail(&uart_rx_buf) == 0) {
                break;
        }
        char c = UARTREG(base, UART_RFIFO);
        cbuf_write_char(&uart_rx_buf, c, false);

    }

    return true;
}
#endif

static void s905_uart_init(mdi_node_ref_t* node, uint level)
{
#if 1
        if (!s905_uart_base) {
            return;
        }
        sput("IIIIIIIIIIIIIIIIIIII",20);
        printf("jjjjjjjjjjjjjjjjj\n");
        // create circular buffer to hold received data
        cbuf_initialize(&uart_rx_buf, RXBUF_SIZE);

        // assumes interrupts are contiguous
        register_int_handler(UART0_A0_INT, &uart_irq, (void *)3);

        printf("UART CONTROL REG = %08x\n",UARTREG(s905_uart_base, UART_CONTROL));
        printf("UART STATUS REG = %08x\n",UARTREG(s905_uart_base, UART_STATUS));
        printf("UART IRG REG = %08x\n",UARTREG(s905_uart_base, UART_IRQ_CONTROL));
        printf("UART REG REG = %08x\n",UARTREG(s905_uart_base, UART_REG5));

        UARTREG(s905_uart_base,UART_CONTROL) = 0x00;

        uint32_t temp2 = UARTREG(s905_uart_base,UART_IRQ_CONTROL);
        temp2 &= 0xffff0000;
        temp2 |= (1 << 8) | ( 1 );
        UARTREG(s905_uart_base,UART_IRQ_CONTROL) = temp2;



        uint32_t temp = UARTREG(s905_uart_base,UART_CONTROL);
        temp |=  (1 << 22) | (1 << 23) | (1<<24);
        UARTREG(s905_uart_base,UART_CONTROL) = temp;

        temp &= ~((1 << 22) | (1 << 23) | (1<<24));
        UARTREG(s905_uart_base,UART_CONTROL) = temp;




        temp |= (1 << 12) | (1 << 13);
        UARTREG(s905_uart_base,UART_CONTROL) = temp;

        temp |= (1 << 27);
        UARTREG(s905_uart_base,UART_CONTROL) = temp;


        printf("UART CONTROL REG = %08x\n",UARTREG(s905_uart_base, UART_CONTROL));
        printf("UART STATUS REG = %08x\n",UARTREG(s905_uart_base, UART_STATUS));
        printf("UART IRG REG = %08x\n",UARTREG(s905_uart_base, UART_IRQ_CONTROL));
        printf("UART REG REG = %08x\n",UARTREG(s905_uart_base, UART_REG5));
        while ( (UARTREG(s905_uart_base, UART_STATUS)&0x0000000f) > 0 ) {
            char c = UARTREG(s905_uart_base, UART_RFIFO);
            printf(" %c",c);
        }
        printf("\n");
        initialized = true;

/*

#define UART_CONTROL (0x8)
#define UART_STATUS (0xc)
#define UART_IRQ_CONTROL (0x10)
#define UART_REG5 (0x14)
        // clear all irqs
        UARTREG(s905_uart_base, UART_ICR) = 0x3ff;

        // set fifo trigger level
        UARTREG(s905_uart_base, UART_IFLS) = 0; // 1/8 rxfifo, 1/8 txfifo

        // enable rx interrupt
        UARTREG(s905_uart_base, UART_IMSC) = (1<<4); // rxim

        // enable receive
        UARTREG(s905_uart_base, UART_CR) |= (1<<9); // rxen
*/
        // enable interrupt
        unmask_interrupt(UART0_A0_INT);

#endif
}

/* panic-time getc/putc */
static int s905_uart_pputc(char c)
{
    if (!s905_uart_base)
        return 0;

    /* spin while fifo is full */
    while (UARTREG(s905_uart_base, UART_STATUS) & (1<<21))
        ;
    UARTREG(s905_uart_base, UART_WFIFO) = c;

    return 1;
}

static int s905_uart_pgetc(void)
{
    if (!s905_uart_base)
        return 0;

    if ((UARTREG(s905_uart_base, UART_STATUS) & (1<<20)) == 0) {
        return UARTREG(s905_uart_base, UART_RFIFO);
    } else {
        return -1;
    }
}


static int s905_uart_putc(char c)
{
    if (!s905_uart_base)
        return 0;

    /* spin while fifo is full */
    while (UARTREG(s905_uart_base, UART_STATUS) & (1<<21))
        ;
    UARTREG(s905_uart_base, UART_WFIFO) = c;

    return 1;
}

static int s905_uart_getc(bool wait)
{
#if 0
    cbuf_t *rxbuf = &uart_rx_buf[port];

    char c;
    if (cbuf_read_char(rxbuf, &c, wait) == 1) {
        UARTREG(uart_to_ptr(port), UART_IMSC) = (1<<4); // rxim
        return c;
    }
#endif

    if (!s905_uart_base)
        return -1;

    if (initialized) {
        // do cbuf stuff here

        char c;
        if (cbuf_read_char(&uart_rx_buf, &c, false) == 1)
            return c;

        return -1;

    } else {
        //Interupts not online yet, use the panic calls for now.
        return s905_uart_pgetc();
    }
}




static const struct pdev_uart_ops s905_uart_ops = {
    .putc = s905_uart_putc,
    .getc = s905_uart_getc,
    .pputc = s905_uart_pputc,
    .pgetc = s905_uart_pgetc,
};

static void s905_uart_init_early(mdi_node_ref_t* node, uint level)
{
#if 0
    for (size_t i = 0; i < NUM_UART; i++) {
        UARTREG(uart_to_ptr(i), UART_CR) = (1<<8)|(1<<0); // tx_enable, uarten
    }
#endif
    sput("RRRRRRRRRRRRRRRRRRRR",20);
    uint32_t port_val = 0;

    mdi_node_ref_t child;
    mdi_each_child(node, &child) {
        switch (mdi_id(&child)) {
            case MDI_KERNEL_DRIVERS_S905_UART_PORTNUM:
                if(mdi_node_uint32(&child, &port_val) != NO_ERROR)
                    return;
                break;
        }
    }

    s905_uart_base = uart_to_ptr(port_val);

    pdev_register_uart(&s905_uart_ops);
}

LK_PDEV_INIT(s905_uart_init_early, MDI_KERNEL_DRIVERS_S905_UART, s905_uart_init_early, LK_INIT_LEVEL_PLATFORM_EARLY);
LK_PDEV_INIT(s905_uart_init, MDI_KERNEL_DRIVERS_S905_UART, s905_uart_init, LK_INIT_LEVEL_PLATFORM);
