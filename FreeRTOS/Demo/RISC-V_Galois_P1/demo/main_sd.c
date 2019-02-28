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
 * main_spi() creates one task to test the loopback mode of the AXI SPI
 * and then starts the task scheduler.
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Demo includes. */
#include "uart_16550.h"

/* Xilinx driver includes */
#include "xspi.h"
#include "bsp.h"
#include "plic_driver.h"

/* SD includes */
#include "diskio.h"

#define BUFFER_SIZE 16

/*-----------------------------------------------------------*/

/*
 * Called by main when PROG=main_spi
 */
void main_sd( void );

/*
 * The tasks as described in the comments at the top of this file.
 */
static void vTestSD( void *pvParameters );

/*-----------------------------------------------------------*/

void main_sd( void )
{
	/* Create SPI test */
	// xTaskCreate( vTestSPI, "SPI Test", 1000, NULL, 0, NULL );
  xTaskCreate( vTestSD, "SD Test", 1000, NULL, 0, NULL );

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
/* Instance of Spi device */
static XSpi SpiInstance;

/* Buffers used to read and write to SPI device */
unsigned char ReadBuffer[BUFFER_SIZE];
unsigned char WriteBuffer[BUFFER_SIZE];

void vTestSD( void *pvParameters )
{                                           
  (void) pvParameters;

  unsigned int Count;
  unsigned char Test;

  disk_initialize(SpiInstance, 0);

  /* Put data in write buffer, initialize read buffer to zero */
  Test = 0x10;
  for (Count = 0; Count < BUFFER_SIZE; Count++) {
    WriteBuffer[Count] = (char) (Count + Test);
    ReadBuffer[Count] = 0;
  }

  /* Write to one block starting at sector 100 */
  disk_write(SpiInstance, 0, WriteBuffer, 100, 1);

  /* Read the same block */
  disk_read(SpiInstance, 0, ReadBuffer, 100, 1);

  /* Compare received data with transmitted data */
  for (Count = 0; Count < BUFFER_SIZE; Count++) {
    configASSERT(WriteBuffer[Count] == ReadBuffer[Count]);
  }

  vTaskDelete(NULL);

}

/*-----------------------------------------------------------*/