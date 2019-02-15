
#include <stdint.h>
#include "uart_16550.h"


/* Struct for 16550 register space */
struct __attribute__ ((aligned (4))) uart_pio
{
  /* 0x000 */
  union __attribute__ ((aligned (4))) {
    /* Receive buffer when read */
    const volatile uint8_t rbr;
    /* Transmit holding register when written */
    volatile uint8_t thr;
    /* Divisor latch (LSB) register when bit 7 of LCR is set */
    volatile uint8_t dll;
  };

  /* 0x004 */
  union __attribute__ ((aligned (4))) {
    /* Divisor latch (MSB) register when bit 7 of LCR is set */
    volatile uint8_t dlm;
    /* Interrupt enable register */
    volatile enum ier_t ier;
  };

  /* 0x008 */
  union __attribute__ ((aligned (4))) {
    /* Interrupt identification register */
    const volatile enum iir_t iir;
    /* FIFO control register when bit 7 of LCR is set */
    volatile enum fcr_t fcr;
  };

  /* 0x00c -- Line control register (LCR) */
  volatile enum lcr_t lcr __attribute__ ((aligned (4)));

  /* 0x010 -- Modem control register */
  volatile enum mcr_t mcr __attribute__ ((aligned (4)));

  /* 0x014 -- Line status register (LSR) */
  volatile enum lsr_t lsr __attribute__ ((aligned (4)));

  /* 0x018 -- Modem status register */
  volatile uint8_t msr __attribute__ ((aligned (4)));

  /* 0x01c -- Scratch register */
  volatile uint8_t scr __attribute__ ((aligned (4)));
};


static struct uart_pio * pio0 = (void*)UART0_BASE;
static struct uart_pio * pio1 = (void*)UART1_BASE;

__attribute__ ((constructor))
int uart0_init(void)
{
  uint32_t divisor;
  divisor = UART_CLOCK_RATE / (16 * DEFAULT_BAUDRATE);

  /* Disable all interrupts */
  pio0->ier = 0;

  /* DLAB=1, Set Divisor Latch MSB and LSB registers */
  pio0->lcr |= LCR_DLAB;
  pio0->dll = divisor & 0xff;
  pio0->dlm = (divisor >> 8) & 0xff;

  /* DLAB=0 Allow access to RBR, THR, IER, IIR registers*/
  pio0->lcr &= ~LCR_DLAB;

  /* 8 bits/char, 2 stop bits */
  pio0->lcr = 7;

  /* Enable FIFOs */
  pio0->fcr = FCR_FE;

  /* Drive RTSN (request to send) low */
  pio0->mcr = MCR_RTS;

  return 0;
}


int uart0_rxready(void)
{
  return ((pio0->lsr & LSR_DR) != 0);
}


int uart0_rxchar(void)
{
  while ((pio0->lsr & LSR_DR) == 0)
    ;  /* Wait */

  return pio0->rbr;
}


int uart0_txchar(int c)
{
  while ((pio0->lsr & LSR_THRE) == 0)
    ;  /* Wait */

  pio0->thr = c;

  return c;
}

/* Wait for transmitter shift register/FIFO to empty */
void uart0_flush(void)
{
  while ((pio0->lsr & LSR_TEMT) == 0)
    ;  /* Wait */

  while ((pio0->lsr & LSR_THRE) == 0)
    ;  /* Wait */
}

int uart1_init(void)
{
  uint32_t divisor;
  divisor = UART_CLOCK_RATE / (16 * UART1_BAUDRATE);

  /* Disable all interrupts */
  pio1->ier = 0;

  /* DLAB=1, Set Divisor Latch MSB and LSB registers */
  pio1->lcr |= LCR_DLAB;
  pio1->dll = divisor & 0xff;
  pio1->dlm = (divisor >> 8) & 0xff;

  /* DLAB=0 Allow access to RBR, THR, IER, IIR registers*/
  pio1->lcr &= ~LCR_DLAB;

  /* 8 bits/char, 2 stop bits */
  pio1->lcr = 7;

  /* Enable FIFOs */
  pio1->fcr = FCR_FE;

  /* Drive RTSN (request to send) low */
  pio1->mcr = MCR_RTS;

  return 0;
}


int uart1_rxready(void)
{
  return ((pio1->lsr & LSR_DR) != 0);
}


int uart1_rxchar(void)
{
  while ((pio1->lsr & LSR_DR) == 0)
    ;  /* Wait */

  return pio1->rbr;
}


int uart1_txchar(int c)
{
  while ((pio1->lsr & LSR_THRE) == 0)
    ;  /* Wait */

  pio1->thr = c;

  return c;
}

/* Wait for transmitter shift register/FIFO to empty */
void uart1_flush(void)
{
  while ((pio1->lsr & LSR_TEMT) == 0)
    ;  /* Wait */

  while ((pio1->lsr & LSR_THRE) == 0)
    ;  /* Wait */
}

