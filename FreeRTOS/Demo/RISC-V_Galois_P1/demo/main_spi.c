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

#define BUFFER_SIZE 16

/* Indicates whether transfer in progress based on interrupts */
volatile int TransferInProgress;

/* Track errors that occur during interrupt processing */
int Error;

/*-----------------------------------------------------------*/

/*
 * Called by main when PROG=main_spi
 */
void main_spi( void );

/*
 * The tasks as described in the comments at the top of this file.
 */

void vTestXilinxSPI( void *pvParameters );

void vTestXilinxSPIInterrupts( void *pvParameters );

void vTestSpiLCD( void *pvParameters );

/* Interrupt functions */
static int SpiSetupIntrSystem(XSpi *SpiPtr);

static void SpiStatusHandler(void *CallBackRef, int StatusEvent, int ByteCount);

/*-----------------------------------------------------------*/

void main_spi( void )
{
	/* Create SPI test */
	// xTaskCreate( vTestSPI, "SPI Test", 1000, NULL, 0, NULL );
  xTaskCreate( vTestSpiLCD, "Test SPI LCD", 1000, NULL, 0, NULL );

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
// unsigned char ReadBuffer[BUFFER_SIZE];
unsigned char WriteBuffer[BUFFER_SIZE];
// unsigned char WriteBuffer[4];

void vTestSpiLCD( void *pvParameters )
{
  /* Testing Xilinx SPI driver and interrupts using LCD */

  (void) pvParameters;

  int Status_CfgInitialize, Status_SetOptions, Status_SetupIntrSystem, Status_SetSlaveSelect;
  unsigned int Count;
  unsigned char Test;
  XSpi_Config *ConfigPtr; /* Pointer to Configuration data */

  /* Initalize SPI device driver */
  ConfigPtr = XSpi_LookupConfig(XPAR_SPI_0_DEVICE_ID);
  configASSERT(ConfigPtr != NULL);

  Status_CfgInitialize = XSpi_CfgInitialize(&SpiInstance, ConfigPtr,
          ConfigPtr->BaseAddress);
  configASSERT(Status_CfgInitialize == XST_SUCCESS);

  /* Setup interrupt system */
  Status_SetupIntrSystem = SpiSetupIntrSystem(&SpiInstance);
  configASSERT(Status_SetupIntrSystem == XST_SUCCESS);

  /* Setup SPI status handler to indicate that SpiIntrHandler
  should be called when there is an interrupt (doesn't PLIC table do this?) */
  XSpi_SetStatusHandler(&SpiInstance, &SpiInstance, 
          (XSpi_StatusHandler) SpiStatusHandler);

  /* Set device to master mode */
  Status_SetOptions = XSpi_SetOptions(&SpiInstance, XSP_MASTER_OPTION |
          XSP_MANUAL_SSELECT_OPTION);
  configASSERT(Status_SetOptions == XST_SUCCESS);

  /* Chooses slaves, doesn't set SS yet */
  Status_SetSlaveSelect = XSpi_SetSlaveSelect(&SpiInstance, 1);
  configASSERT(Status_SetSlaveSelect == XST_SUCCESS);

  /* Start the SPI driver so that the device and interrupts are enabled */
  XSpi_Start(&SpiInstance);

  /* Put data to send in write buffer */
  WriteBuffer[0] = '|';
  WriteBuffer[1] = '-';
  WriteBuffer[2] = 'h';
  WriteBuffer[3] = 'i';
  
  /* Put data to send in write buffer, initialize read buffer to zero */
  Test = 0x10;
    for (Count = 0; Count < BUFFER_SIZE; Count++) {
    WriteBuffer[Count] = (char) (Count + Test);
  }

  /* Transmit the data */
  TransferInProgress = TRUE;
  // XSpi_Transfer(&SpiInstance, WriteBuffer, NULL, 4);
  XSpi_Transfer(&SpiInstance, WriteBuffer, NULL, BUFFER_SIZE);

  /* Wait for transfer to finish */
  while (TransferInProgress);

  // vTaskDelay( pdMS_TO_TICKS(25) );

  vTaskDelete(NULL);

}
/*-----------------------------------------------------------*/


// void vTestXilinxSPIInterrupts( void *pvParameters )
// {

//   /* Testing Xilinx SPI driver and interrupts using LCD */

//   (void) pvParameters;

//   int Status_CfgInitialize, Status_SetOptions, Status_SetupIntrSystem;
//   unsigned int Count;
//   unsigned char Test;
//   XSpi_Config *ConfigPtr; /* Pointer to Configuration data */

//   /* Initalize SPI device driver */
//   ConfigPtr = XSpi_LookupConfig(XPAR_SPI_0_DEVICE_ID);
//   configASSERT(ConfigPtr != NULL);

//   Status_CfgInitialize = XSpi_CfgInitialize(&SpiInstance, ConfigPtr,
//           ConfigPtr->BaseAddress);
//   configASSERT(Status_CfgInitialize == XST_SUCCESS);

//   /* Setup interrupt system */
//   Status_SetupIntrSystem = SpiSetupIntrSystem(&SpiInstance);
//   configASSERT(Status_SetupIntrSystem == XST_SUCCESS);

//   /* Setup SPI status handler to indicate that SpiIntrHandler
//   should be called when there is an interrupt (doesn't PLIC table do this?) */
//   XSpi_SetStatusHandler(&SpiInstance, &SpiInstance, 
//           (XSpi_StatusHandler) SpiStatusHandler);

//   /* Set device to master mode and loopback mode */
//   Status_SetOptions = XSpi_SetOptions(&SpiInstance, XSP_MASTER_OPTION |
//           XSP_LOOPBACK_OPTION);
//   configASSERT(Status_SetOptions == XST_SUCCESS);

//   /* Start the SPI driver so that the device and interrupts are enabled */
//   XSpi_Start(&SpiInstance);

//   /* Put data to send in write buffer, initialize read buffer to zero */
//   Test = 0x10;
//   for (Count = 0; Count < BUFFER_SIZE; Count++) {
//     WriteBuffer[Count] = (char) (Count + Test);
//     ReadBuffer[Count] = 0;
//   }

//   /* To receive confirmation that buffers contain same data */
//   printf("WriteBuffer[3] is %d\n", WriteBuffer[3]);
  
//   /* Transmit the data */
//   TransferInProgress = TRUE;
//   XSpi_Transfer(&SpiInstance, WriteBuffer, ReadBuffer, BUFFER_SIZE);

//   /* Wait for transfer to finish */
//   while (TransferInProgress) {
//   }

//   /* Compare received data with transmitted data */
//   for (Count = 0; Count < BUFFER_SIZE; Count++) {
//     configASSERT(WriteBuffer[Count] == ReadBuffer[Count]);
//   }

//   /* To receive confirmation that buffers contain same data */
//   printf("ReaderBuffer[3] is %d\n", ReadBuffer[3]);

//   vTaskDelete(NULL);

// }

// /*-----------------------------------------------------------*/

// void vTestXilinxSPI( void *pvParameters )
// {
//   (void) pvParameters;

//   int Status_CfgInitialize, Status_SelfTest, Status_SetOptions;
//   unsigned int Count;
//   unsigned char Test;
//   XSpi_Config *ConfigPtr; /* Pointer to Configuration data */


//   /* Initalize SPI device driver */
//   ConfigPtr = XSpi_LookupConfig(XPAR_SPI_0_DEVICE_ID);
//   configASSERT(ConfigPtr != NULL);

//   Status_CfgInitialize = XSpi_CfgInitialize(&SpiInstance, ConfigPtr,
//           ConfigPtr->BaseAddress);
//   configASSERT(Status_CfgInitialize == XST_SUCCESS);

//   /* Perform a self-test to ensure hardware was built correctly */
//   Status_SelfTest = XSpi_SelfTest(&SpiInstance);
//   configASSERT(Status_SelfTest == XST_SUCCESS);

//   /* Set device to master mode and loopback mode */
//   Status_SetOptions = XSpi_SetOptions(&SpiInstance, XSP_MASTER_OPTION |
//           XSP_LOOPBACK_OPTION);
//   configASSERT(Status_SetOptions == XST_SUCCESS);

//   /* Start the SPI driver so that the device is enabled */
//   XSpi_Start(&SpiInstance);

//   /* Disable Global interrupt to use polled mode operation */
//   XSpi_IntrGlobalDisable(&SpiInstance);

//   /* Put data in write buffer, initialize read buffer to zero */
//   Test = 0x10;
//   for (Count = 0; Count < BUFFER_SIZE; Count++) {
//     WriteBuffer[Count] = (char) (Count + Test);
//     ReadBuffer[Count] = 0;
//   }
  
//   /* Transmit the data */
//   XSpi_Transfer(&SpiInstance, WriteBuffer, ReadBuffer, BUFFER_SIZE);

//   /* Compare received data with transmitted data */
//   for (Count = 0; Count < BUFFER_SIZE; Count++) {
//     configASSERT(WriteBuffer[Count] == ReadBuffer[Count]);
//   }

//   vTaskDelete(NULL);

// }

/*-----------------------------------------------------------*/

static int SpiSetupIntrSystem(XSpi *SpiPtr)
{
  int Status;

  /*
   * Connect a device driver handler that will be called when an interrupt
   * for the device occurs, the device driver handler performs the
   * specific interrupt processing for the device
   */
  Status = PLIC_register_interrupt_handler(&Plic, PLIC_SOURCE_SPI,
          XSpi_InterruptHandler, SpiPtr);
  if (Status != PLIC_SOURCE_SPI) {
    return XST_FAILURE;
  }

  return XST_SUCCESS;

}

/*-----------------------------------------------------------*/

static void SpiStatusHandler(void *CallBackRef, int StatusEvent, int ByteCount)
{
  (void) CallBackRef;

  /* Indicate transfer no longer in progress */
  TransferInProgress = FALSE;

  /* If event was not transfer done, track it as an error */
  if (StatusEvent != XST_SPI_TRANSFER_DONE) {
    Error++;
  }
}
