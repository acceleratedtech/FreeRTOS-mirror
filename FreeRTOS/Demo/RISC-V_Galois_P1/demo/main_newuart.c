/*
 * FreeRTOS Kernel V10.1.1
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/******************************************************************************
 * NOTE 1:  This project provides two demo applications.  A simple blinky
 * style project, and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting in main.c is used to select
 * between the two.  See the notes on using mainCREATE_SIMPLE_BLINKY_DEMO_ONLY
 * in main.c.  This file implements the simply blinky style version.
 *
 * NOTE 2:  This file only contains the source code that is specific to the
 * basic demo.  Generic functions, such FreeRTOS hook functions, and functions
 * required to configure the hardware are defined in main.c.
 ******************************************************************************
 *
 * main_barcode() creates one task to test receivng a barcode and then starts the
 * scheduler.
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Xilinx driver includes. */
#include "xuartns550.h"
#include "bsp.h"
#include "plic_driver.h"

/*-----------------------------------------------------------*/

/*
 * Called by main when PROG=main_barcode
 */
void main_newuart( void );

/*
 * The tasks as described in the comments at the top of this file.
 */
static void vUartSelfTest( void *pvParameters );

/*-----------------------------------------------------------*/

void main_newuart( void )
{
	/* Create barcode test */
	xTaskCreate( vUartSelfTest, "UART Self Test", 1000, NULL, 0, NULL );

	/* Start the kernel.  From here on, only tasks and interrupts will run. */
	vTaskStartScheduler();


	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the Idle and/or
	timer tasks to be created.  See the memory management section on the
	FreeRTOS web site for more details on the FreeRTOS heap
	http://www.freertos.org/a00111.html. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Instance of UART device */
static XUartNs550 UartNs550;

void vUartSelfTest( void *pvParameters )
{
	/* Test UART1's self-test functionality  */

	(void) pvParameters;

	int Status_Init, Status_SelfTest;

	/* Initialize the UartNs550 driver so that it's ready to use */
	Status_Init = XUartNs550_Initialize(&UartNs550, XPAR_UARTNS550_1_DEVICE_ID);
	configASSERT(Status_Init == XST_SUCCESS);

	/* Perform a self-test to ensure that the hardware was built correctly */
	Status_SelfTest = XUartNs550_SelfTest(&UartNs550);
	configASSERT(Status_SelfTest == XST_SUCCESS);

	vTaskDelete(NULL);

}

/*-----------------------------------------------------------*/