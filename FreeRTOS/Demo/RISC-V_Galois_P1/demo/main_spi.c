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
#include "xparameters.h"
#include "xspi.h"

#define BUFFER_SIZE 16

#define SPI_SRR (*(unsigned int *) 0x62320040)
#define SPICR (*(unsigned int *) 0x62320060)
#define SPISR (*(unsigned int *) 0x62320064)
#define SPI_DTR (*(unsigned int *) 0x62320068)
#define SPI_DRR (*(unsigned int *) 0x6232006c)
#define SPI_SSR (*(unsigned int *) 0x62320070)

/*-----------------------------------------------------------*/

/*
 * Called by main when PROG=main_spi
 */
void main_spi( void );

/*
 * The tasks as described in the comments at the top of this file.
 */
static void vTestSPI( void *pvParameters );

static void vTestXilinxSPI( void *pvParameters );

/*-----------------------------------------------------------*/

void main_spi( void )
{
	/* Create SPI test */
	// xTaskCreate( vTestSPI, "SPI Test", 1000, NULL, 0, NULL );
  xTaskCreate( vTestXilinxSPI, "Xilinx SPI Test", 1000, NULL, 0, NULL );

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

void vTestSPI( void *pvParameters )
{
	/* vTestSPI() tests the AXI SPI on the VCU118 in 
  loopback mode */

  (void) pvParameters;

  char spi_rx0;
  char spi_rx1;
  char spi_rx2;

  /***** INIT *****/
  /* Software reset */
  SPI_SRR = 0xa;

  /* Set to Master mode */
  SPICR |= 0x4;

  /* Enable loopback mode (LOOP) */
  SPICR |= 0x1;

	/***** TRANSMIT *****/
	/* Inhibit master */
  SPICR |= 0x100;

  /* Write data to data transmit register */
  SPI_DTR = 'a';
  SPI_DTR = 'b';
  SPI_DTR = 'c';

  /* SPI system enable (SPE) */
  SPICR |= 0x2;

  /* Deinhibit master */
  SPICR &= ~(0x100);

  /* Wait until TX is empty */
  while ( (SPISR & 0x4) == 0 )
     ; // wait

  /* While RX empty is true, wait */
  while ( (SPISR & 0x1) != 0 )
     ; // wait

  /* Receive and print out */
  spi_rx0 = (char) SPI_DRR;
  spi_rx1 = (char) SPI_DRR;
  spi_rx2 = (char) SPI_DRR;

  printf( "Received: %c %c %c \n", spi_rx0, spi_rx1, spi_rx2);

  vTaskDelete( NULL );
}

/*-----------------------------------------------------------*/
/* Instance of Spi device */
static XSpi SpiInstance;

/* Buffers used to read and write to SPI device */
unsigned char ReadBuffer[BUFFER_SIZE];
unsigned char WriteBuffer[BUFFER_SIZE];

void vTestXilinxSPI( void *pvParameters )
{
  (void) pvParameters;

  int Status_CfgInitialize, Status_SelfTest, Status_SetOptions;
  unsigned int Count;
  unsigned char Test;
  XSpi_Config *ConfigPtr; /* Pointer to Configuration data */


  /* Initalize SPI device driver */
  ConfigPtr = XSpi_LookupConfig(XPAR_SPI_0_DEVICE_ID);
  configASSERT(ConfigPtr != NULL);

  Status_CfgInitialize = XSpi_CfgInitialize(&SpiInstance, ConfigPtr,
          ConfigPtr->BaseAddress);
  configASSERT(Status_CfgInitialize == XST_SUCCESS);

  /* Perform a self-test to ensure hardware was built correctly */
  Status_SelfTest = XSpi_SelfTest(&SpiInstance);
  configASSERT(Status_SelfTest == XST_SUCCESS);

  /* Set device to master mode and loopback mode */
  Status_SetOptions = XSpi_SetOptions(&SpiInstance, XSP_MASTER_OPTION |
          XSP_LOOPBACK_OPTION);
  configASSERT(Status_SetOptions == XST_SUCCESS);

  /* Start the SPI driver so that the device is enabled */
  XSpi_Start(&SpiInstance);

  /* Disable Global interrupt to use polled mode operation */
  XSpi_IntrGlobalDisable(&SpiInstance);

  /* Put data in write buffer, initialize read buffer to zero */
  Test = 0x10;
  for (Count = 0; Count < BUFFER_SIZE; Count++) {
    WriteBuffer[Count] = (char) (Count + Test);
    ReadBuffer[Count] = 0;
  }
  
  /* Transmit the data */
  XSpi_Transfer(&SpiInstance, WriteBuffer, ReadBuffer, BUFFER_SIZE);

  /* Compare received data with transmitted data */
  for (Count = 0; Count < BUFFER_SIZE; Count++) {
    configASSERT(WriteBuffer[Count] == ReadBuffer[Count]);
  }

  vTaskDelete(NULL);

}

// void vTestSPISD( void *pvParameters )
// {
// 	/* vTestSPISD() tests writing to an SD card using AXI SPI and a
// 	Digilent PMOD SD connector */

// 	/***** INIT *****/
// 	/* Software reset */
// 	SPI_SRR = 0xa;

// 	/* Set to Master mode */
// 	SPICR |= 0x4;

// 	/* Set CPHA=0, CPOL=0 */
// 	SPI_CR &= ~(0x18);

// 	/***** Set SD card to SPI mode *****/
// 	/* Need to set to SPI mode first? */

// 	/* Send CMD0 (GO_IDLE_STATE) and assert chip select */
// 	/* Inhibit master */
// 	SPICR |= 0x100;

// 	/* Write CMD0 (reset) to data transmit register */
// 	/* Needs CRC value? */
// 	SPI_DTR = 00000000;

// 	/* SPI system enable (SPE) */
// 	SPICR |= 0x2;

// 	/* Assert chip select */
// 	SPI_SSR &= 0;

// 	/* Deinhibit master */
// 	SPICR &= ~(0x100);

// 	/* Deassert chip select */
// 	SPI_SSR |= 1;

// 	/* Inhibit master */
// 	SPICR |= 0x100;

// 	/* Check for R1 response? */

// 	 Initiate initialization process w/ CMD1 (or ACDM41?) 

// 	/* Check response until end of initialization */

// 	/* R1 resp should change from 0x01 to 0x00 */

// 	/* Send CMD8 (SEND_IF_COND) */
	

// 	/***** WRITE *****/
// 	/* Inhibit master */
// 	SPICR |= 0x100;

// 	/* Write data to data transmit register */
// 	SPI_DTR = 'h';
// 	SPI_DTR = 'e';
// 	SPI_DTR = 'l';
// 	SPI_DTR = 'l';
// 	SPI_DTR = 'o';
// 	SPI_DTR = '!';

// 	/* SPI system enable (SPE) */
// 	SPICR |= 0x2;

// 	/* Assert chip select */
// 	SPI_SSR &= 0;

// 	/* Deinhibit master */
// 	SPICR &= ~(0x100);

// 	/* Deassert chip select */
// 	SPI_SSR |= 1;

// 	/* Inhibit master */
// 	SPICR |= 0x100;

// 	/***** READ *****/
// 	/* Send read data command (and address) to SD card */

// 	/* Assert chip select */
// 	SPI_SSR &= 0;

// 	/* Deinhibit master */
// 	SPICR &= ~(0x100);

// 	/* Deassert chip select */
// 	SPI_SSR |= 1;

// 	/* Inhibit master */
// 	SPICR |= 0x100;

// 	/* Read data read register */

// }

/*-----------------------------------------------------------*/