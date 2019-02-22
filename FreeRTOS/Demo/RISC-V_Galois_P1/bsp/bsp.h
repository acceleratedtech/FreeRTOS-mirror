#ifndef RISCV_BLUESPEC_BSP_H
#define RISCV_BLUESPEC_BSP_H

#include "stdint.h"
#include "plic_driver.h"

// PLIC defines
#define PLIC_BASE_ADDR (0xC000000ULL)

#define PLIC_SOURCE_UART0 0x1
#define PLIC_SOURCE_UART1 0x2
#define PLIC_SOURCE_IIC 0x3
#define PLIC_SOURCE_SPI 0x4

#define PLIC_PRIORITY_UART0 0x3
#define PLIC_PRIORITY_UART1 0x3
#define PLIC_PRIORITY_IIC 0x7
#define PLIC_PRIORITY_SPI 0x2

// IIC Defines
#define XPAR_XIIC_NUM_INSTANCES     1
#define XPAR_IIC_0_DEVICE_ID 0
#define XPAR_IIC_0_BASEADDR (0x62310000ULL)
#define XPAR_IIC_0_TEN_BIT_ADR 0
#define XPAR_IIC_0_GPO_WIDTH 32

// UART defines
#define UART_CLOCK_RATE  (83000000ULL) // 83MHz
#define XPAR_DEFAULT_BAUD_RATE 9600

#define XPAR_XUARTNS550_NUM_INSTANCES 2

#define XPAR_UARTNS550_0_DEVICE_ID 0
#define XPAR_UARTNS550_0_BASEADDR 0x62300000ULL
#define XPAR_UARTNS550_0_CLOCK_HZ UART_CLOCK_RATE

#define XPAR_UARTNS550_1_DEVICE_ID 1
#define XPAR_UARTNS550_1_BASEADDR 0x62340000ULL
#define XPAR_UARTNS550_1_CLOCK_HZ UART_CLOCK_RATE

extern plic_instance_t Plic;

/* Prepare haredware to run the demo. */
void prvSetupHardware( void );

void external_interrupt_handler( uint32_t cause );

#endif /* RISCV_BLUESPEC_BSP_H */
