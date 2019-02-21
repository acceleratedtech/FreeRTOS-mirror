#include "bsp.h"
#include "xuartns550.h"
#include "plic_driver.h"

#include "FreeRTOS.h"
#include <task.h>


// to communicate with the debugger in spike
volatile uint64_t tohost __attribute__((aligned(64)));
volatile uint64_t fromhost __attribute__((aligned(64)));

plic_instance_t Plic;
XUartNs550 Uart0;	/* Instance of the UART Device */

void prvSetupHardware( void )
{
    // Resets PLIC, threshold 0, nothing enabled
    PLIC_init(&Plic, PLIC_BASE_ADDR, 4, 4);

    // Set priorities
    PLIC_set_priority(&Plic, PLIC_SOURCE_UART0, PLIC_PRIORITY_UART0);
    PLIC_set_priority(&Plic, PLIC_SOURCE_UART1, PLIC_PRIORITY_UART1);
    PLIC_set_priority(&Plic, PLIC_SOURCE_IIC, PLIC_PRIORITY_IIC);
    PLIC_set_priority(&Plic, PLIC_SOURCE_SPI, PLIC_PRIORITY_SPI);

    // Enable external interrupts
    // TODO: the interrupts should be enabled from the peripherals
    //PLIC_enable_interrupt(&Plic,PLIC_SOURCE_UART0);
    //PLIC_enable_interrupt(&Plic,PLIC_SOURCE_UART1);
    //PLIC_enable_interrupt(&Plic,PLIC_SOURCE_IIC);
    //PLIC_enable_interrupt(&Plic,PLIC_SOURCE_SPI);

    // Initialize UART0
    int status;
    status = XUartNs550_Initialize(&Uart0, 0);
    configASSERT(status == 0);

	//status = XUartNs550_SetOptions(&Uart0, XUN_OPTION_LOOPBACK);
    //configASSERT(status == 0);

    status = XUartNs550_SelfTest(&Uart0);
    configASSERT(status == 0);

    //uint8_t HelloWorld[] = "Hello World";
	//uint8_t sent_cnt = 0;
    //XUartNs550_Send(&Uart0, HelloWorld, sizeof(HelloWorld));

    __asm volatile( "ebreak" );

	//UART_init();
	//I2C_init();
	//SPI_init();
	//GPIO_init();
 	//PLIC_init();
 	//UART_init( &g_uart, COREUARTAPB0_BASE_ADDR, BAUD_VALUE_115200, ( DATA_8_BITS | NO_PARITY ) );
}
// Define an external interrupt handler
// cause = 0x8000000b == Machine external interrupt
void external_interrupt_handler( uint32_t cause ) {
    if (cause != 0x8000000b) {
        // unknown cause
        __asm volatile( "ebreak" );
    }

    plic_source src = PLIC_claim_interrupt(&Plic);

    if ((source_id >=1 ) && (source_id < PLIC_NUM_INTERRUPTS)) {
        Plic.HandlerTable[source_id].Handler(Plic.HandlerTable[source_id].CallBackRef);
    }

    // TODO: check return status from the handlers
    // TODO: should we process all interrupts while we are here?

    // clear interrupt
    PLIC_complete_interrupt(&Plic,src);
}

