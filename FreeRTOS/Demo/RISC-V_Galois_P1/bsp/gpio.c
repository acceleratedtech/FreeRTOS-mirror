#include "gpio.h"

static struct gpio_reg_str * gpio_reg = (void*)GPIO_BASE;

void gpio1_write(int i)
{
	gpio_reg->gpio1_data |= (1<<i);
}

void gpio1_clear(int i)
{
	gpio_reg->gpio1_data &= ~(1<<i);
}

void gpio2_write(int i)
{
	gpio_reg->gpio2_data |= (1<<i);
}

void gpio2_clear(int i)
{
	gpio_reg->gpio2_data &= ~(1<<i);
}