#include <stdint.h>

#define GPIO_BASE (0x62330000)

/* Struct for GPIO register space */
struct __attribute__ ((aligned (4))) gpio_reg_str
{
	/* 0x00 -- Channel 1 GPIO data register */
	volatile uint8_t gpio1_data __attribute__ ((aligned (4)));

	/* 0x04 -- Channel 1 GPIO 3-state control register */
	volatile uint8_t gpio1_tri __attribute__ ((aligned (4)));

	/* 0x08 -- Channel 2 GPIO data register */
	volatile uint8_t gpio2_data __attribute__ ((aligned (4)));

	/* 0x0C -- Channel 2 GPIO 3-state control register */
	volatile uint8_t gpio2_tri __attribute__ ((aligned (4)));
};

void gpio1_write(int i);
void gpio1_clear(int i);
void gpio2_write(int i);
void gpio2_clear(int i);