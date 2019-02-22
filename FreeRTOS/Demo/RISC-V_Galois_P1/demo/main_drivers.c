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
 * main_blinky() creates one queue, and two tasks.  It then starts the
 * scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  prvQueueSendTask() sits in a loop that causes it to repeatedly
 * block for 1000 milliseconds, before sending the value 100 to the queue that
 * was created within main_blinky().  Once the value is sent, the task loops
 * back around to block for another 1000 milliseconds...and so on.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() sits in a loop where it repeatedly
 * blocks on attempts to read data from the queue that was created within
 * main_blinky().  When data is received, the task checks the value of the
 * data, and if the value equals the expected 100, writes 'Blink' to the UART
 * (the UART is used in place of the LED to allow easy execution in QEMU).  The
 * 'block time' parameter passed to the queue receive function specifies that
 * the task should be held in the Blocked state indefinitely to wait for data to
 * be available on the queue.  The queue receive task will only leave the
 * Blocked state when the queue send task writes to the queue.  As the queue
 * send task writes to the queue every 1000 milliseconds, the queue receive
 * task leaves the Blocked state every 1000 milliseconds, and therefore toggles
 * the LED every 200 milliseconds.
 */

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* xillinx driver include */
#include "xiic.h"
#include "plic_driver.h"
#include "bsp.h"
#include "uart_16550.h"
#include "xuartns550.h"

/* Priorities used by the tasks. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the pdMS_TO_TICKS() macro. */
#define mainQUEUE_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 1000 )

/* The maximum number items the queue can hold.  The priority of the receiving
task is above the priority of the sending task, so the receiving task will
preempt the sending task and remove the queue items each time the sending task
writes to the queue.  Therefore the queue will never have more than one item in
it at any time, and even with a queue length of 1, the sending task will never
find the queue full. */
#define mainQUEUE_LENGTH					( 1 )

/*-----------------------------------------------------------*/

/*
 * Called by main when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1 in
 * main.c.
 */
void main_drivers( void );

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvIicTestTask( void *pvParameters );

/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

/*-----------------------------------------------------------*/

void main_drivers( void )
{
	/* Create the queue. */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint32_t ) );

	if( xQueue != NULL )
	{
		xTaskCreate( prvIicTestTask, "IIC_test_task", configMINIMAL_STACK_SIZE * 2U, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );

		/* Start the tasks and timer running. */
		vTaskStartScheduler();
	}

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the Idle and/or
	timer tasks to be created.  See the memory management section on the
	FreeRTOS web site for more details on the FreeRTOS heap
	http://www.freertos.org/a00111.html. */
	for( ;; );
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/
#define IIC_DEVICE_ID 0

// the i2c address
#define VCNL4010_I2CADDR_DEFAULT 0x13

// commands and constants
#define VCNL4010_COMMAND 0x80
#define VCNL4010_PRODUCTID 0x81
#define VCNL4010_PROXRATE 0x82
#define VCNL4010_IRLED 0x83
#define VCNL4010_AMBIENTPARAMETER 0x84
#define VCNL4010_AMBIENTDATA 0x85
#define VCNL4010_PROXIMITYDATA 0x87
#define VCNL4010_INTCONTROL 0x89
#define VCNL4010_PROXINITYADJUST 0x8A
#define VCNL4010_INTSTAT 0x8E
#define VCNL4010_MODTIMING 0x8F

#define VCNL4010_MEASUREAMBIENT 0x10
#define VCNL4010_MEASUREPROXIMITY 0x08
#define VCNL4010_AMBIENTREADY 0x40
#define VCNL4010_PROXIMITYREADY 0x20

#define VCNL4010_16_625 3
/*
 * The following structure contains fields that are used with the callbacks
 * (handlers) of the IIC driver. The driver asynchronously calls handlers
 * when abnormal events occur or when data has been sent or received. This
 * structure must be volatile to work when the code is optimized.
 */
volatile struct {
	int  EventStatus;
	int  RemainingRecvBytes;
	int EventStatusUpdated;
	int RecvBytesUpdated;
	int TransmitComplete;
} HandlerInfo;

XIic Iic; /* The driver instance for IIC Device */

int TempSensorExample(u16 IicDeviceId);
static int SetupInterruptSystem(XIic *IicPtr);
static void RecvHandler(void *CallbackRef, int ByteCount);
static void SendHandler(void *CallbackRef, int ByteCount);
static void StatusHandler(void *CallbackRef, int Status);

static bool vcnl4010_init(XIic *InstancePtr);
static uint16_t vcnl4010_readProximity(XIic *InstancePtr);
static uint16_t vcnl4010_readAmbient(XIic *InstancePtr);
static void vcnl4010_write_u8(XIic *InstancePtr, uint8_t addr, uint8_t values);
static uint8_t vcnl4010_read_u8(XIic *InstancePtr, uint8_t addr);
static uint8_t vcnl4010_read_u16(XIic *InstancePtr, uint8_t addr);
/*-----------------------------------------------------------*/

static uint16_t vcnl4010_readProximity(XIic *InstancePtr) {
  uint8_t i = vcnl4010_read_u8(InstancePtr, VCNL4010_INTSTAT);
  i &= ~0x80;
  vcnl4010_write_u8(InstancePtr, VCNL4010_INTSTAT, i);

  vcnl4010_write_u8(InstancePtr, VCNL4010_COMMAND, VCNL4010_MEASUREPROXIMITY);
  while (1) {
    //Serial.println(read8(VCNL4010_INTSTAT), HEX);
    uint8_t result = vcnl4010_read_u8(InstancePtr, VCNL4010_COMMAND);
    //Serial.print("Ready = 0x"); Serial.println(result, HEX);
    if (result & VCNL4010_PROXIMITYREADY) {
      return vcnl4010_read_u16(InstancePtr, VCNL4010_PROXIMITYDATA);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static uint16_t vcnl4010_readAmbient(XIic *InstancePtr) {
  uint8_t i = vcnl4010_read_u8(InstancePtr, VCNL4010_INTSTAT);
  i &= ~0x40;
  vcnl4010_write_u8(InstancePtr, VCNL4010_INTSTAT, i);
  vcnl4010_write_u8(InstancePtr, VCNL4010_COMMAND, VCNL4010_MEASUREAMBIENT);
  while (1) {
	//printf("Read VCNL4010_INTSTAT: %u\r\n", vcnl4010_read_u8(InstancePtr, VCNL4010_INTSTAT));
    uint8_t result = vcnl4010_read_u8(InstancePtr, VCNL4010_COMMAND);
	//printf("Ready: %u\r\n",result);
    if (result & VCNL4010_AMBIENTREADY) {
      return vcnl4010_read_u16(InstancePtr, VCNL4010_AMBIENTDATA);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void vcnl4010_write_u8(XIic *InstancePtr, uint8_t addr, uint8_t values) {
	uint8_t data[2] = {addr, values};
	(void)XIic_MasterSend(InstancePtr, data, 2);
	vTaskDelay(pdMS_TO_TICKS(20));
}

static uint8_t vcnl4010_read_u16(XIic *InstancePtr, uint8_t addr) {
	uint8_t data[2] = {0};
	data[0] = addr;
	(void)XIic_MasterSend(InstancePtr, data, 1);
	vTaskDelay(pdMS_TO_TICKS(10));
	(void)XIic_MasterRecv(InstancePtr, data, 2);
	vTaskDelay(pdMS_TO_TICKS(10)); 
	return (uint16_t) (data[0] << 8 | data [1]);
}

static uint8_t vcnl4010_read_u8(XIic *InstancePtr, uint8_t addr) {
	uint8_t data;
	data = addr;
	(void)XIic_MasterSend(InstancePtr, &data, 1);
	vTaskDelay(pdMS_TO_TICKS(10)); // requires 170us delay
  	(void)XIic_MasterRecv(InstancePtr, &data, 1);
	vTaskDelay(pdMS_TO_TICKS(10));
	return data;
}

bool vcnl4010_init(XIic *InstancePtr) {
	XIic_SetAddress(InstancePtr, XII_ADDR_TO_SEND_TYPE, VCNL4010_I2CADDR_DEFAULT);

	uint8_t rev = vcnl4010_read_u8(InstancePtr, VCNL4010_PRODUCTID);
  	if ((rev & 0xF0) != 0x20) {
    	return false;
  	}

	vcnl4010_write_u8(InstancePtr, VCNL4010_IRLED, 20);
	vcnl4010_write_u8(InstancePtr, VCNL4010_MODTIMING, VCNL4010_16_625);

	vcnl4010_write_u8(InstancePtr, VCNL4010_INTCONTROL, 0x08);
	return true;
}

static void prvIicTestTask( void *pvParameters ) {
	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;

	int status;
	printf("\r\nStarting IIC test task\r\n");
	status =  TempSensorExample(IIC_DEVICE_ID);
	configASSERT(status == 0);
	printf("TempSensorExample returned\r\n");
}

/*****************************************************************************/
/**
* The function reads the temperature of the IIC temperature sensor on the
* IIC bus. It initializes the IIC device driver and sets it up to communicate
* with the temperature sensor. This function does contain a loop that polls
* for completion of the IIC processing such that it may not return if
* interrupts or the hardware are not working.
*
* @param	IicDeviceId is the XPAR_<IIC_instance>_DEVICE_ID value from
*		xparameters.h for the IIC Device
* @param	TempSensorAddress is the address of the Temperature Sensor device
*		on the IIC bus
* @param	TemperaturePtr is the data byte read from the temperature sensor
*
* @return	XST_SUCCESS to indicate success, else XST_FAILURE to indicate
*		a Failure.
*
* @note		None.
*
*******************************************************************************/
int TempSensorExample(u16 IicDeviceId)
{
	int Status;
	static int Initialized = FALSE;
	XIic_Config *ConfigPtr;	/* Pointer to configuration data */

	if (!Initialized) {
		Initialized = TRUE;

		/*
		 * Initialize the IIC driver so that it is ready to use.
		 */
		ConfigPtr = XIic_LookupConfig(IicDeviceId);
		if (ConfigPtr == NULL) {
			return XST_FAILURE;
		}

		Status = XIic_CfgInitialize(&Iic, ConfigPtr,
						ConfigPtr->BaseAddress);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}


		/*
		 * Setup handler to process the asynchronous events which occur,
		 * the driver is only interrupt driven such that this must be
		 * done prior to starting the device.
		 */
		XIic_SetRecvHandler(&Iic, (void *)&HandlerInfo, RecvHandler);
		XIic_SetSendHandler(&Iic, (void *)&HandlerInfo, SendHandler);
		XIic_SetStatusHandler(&Iic, (void *)&HandlerInfo, StatusHandler);

		/*
		 * Connect the ISR to the interrupt and enable interrupts.
		 */
		Status = SetupInterruptSystem(&Iic);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		/*
		 * Start the IIC driver such that it is ready to send and
		 * receive messages on the IIC interface, set the address
		 * to send to which is the temperature sensor address
		 */
		XIic_Start(&Iic);
		//XIic_SetAddress(&Iic, XII_ADDR_TO_SEND_TYPE, TempSensorAddress);
	}
	
	printf("Initializing\r\n");
	configASSERT(vcnl4010_init(&Iic) == TRUE);

	while(1) {
		printf("Reading\r\n");
		printf("ambient: %u\r\n", vcnl4010_readAmbient(&Iic));
		vTaskDelay(pdMS_TO_TICKS(100));
		printf("Proximity: %u\r\n",vcnl4010_readProximity(&Iic));
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	// /*
	//  * Attempt to receive a byte of data from the temperature sensor
	//  * on the IIC interface, ignore the return value since this example is
	//  * a single master system such that the IIC bus should not ever be busy
	//  */
	// (void)XIic_MasterRecv(&Iic, TemperaturePtr, 1);

	// /*
	//  * The message is being received from the temperature sensor,
	//  * wait for it to complete by polling the information that is
	//  * updated asynchronously by interrupt processing
	//  */
	// while(1) {
	// 	if(HandlerInfo.RecvBytesUpdated == TRUE) {
	// 		/*
	// 		 * The device information has been updated for receive
	// 		 * processing,if all bytes received (1), indicate
	// 		 * success
	// 		 */
	// 		if (HandlerInfo.RemainingRecvBytes == 0) {
	// 			printf("all bytes were processed\r\n");
	// 			Status = XST_SUCCESS;
	// 		}
	// 		break;
	// 	}

	// 	/*
	// 	 * Any event status which occurs indicates there was an error,
	// 	 * so return unsuccessful, for this example there should be no
	// 	 * status events since there is a single master on the bus
	// 	 */
	// 	if (HandlerInfo.EventStatusUpdated == TRUE) {
	// 		printf("Event updated\r\n");
	// 		break;
	// 	}
	// }

	// printf("Handler info:\r\n");
	// printf("HandlerInfo.EventStatusUpdated = %i\r\n",HandlerInfo.EventStatusUpdated);
	// printf("HandlerInfo.RemainingRecvBytes = %i\r\n",HandlerInfo.RemainingRecvBytes);
	// printf("HandlerInfo.Eventstatus = %i\r\n",HandlerInfo.EventStatus);
	// return Status;
}


static int SetupInterruptSystem(XIic *IicPtr)
{
	int Status;
	/*
	 * Connect a device driver handler that will be called when an interrupt
	 * for the device occurs, the device driver handler performs the
	 * specific interrupt processing for the device
	 */
	Status = PLIC_register_interrupt_handler(&Plic, PLIC_SOURCE_IIC,
					XIic_InterruptHandler, IicPtr);
	if (Status != PLIC_SOURCE_IIC) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}


/*****************************************************************************/
/**
* This receive handler is called asynchronously from an interrupt context and
* and indicates that data in the specified buffer was received. The byte count
* should equal the byte count of the buffer if all the buffer was filled.
*
* @param	CallBackRef is a pointer to the IIC device driver instance which
*		the handler is being called for.
* @param	ByteCount indicates the number of bytes remaining to be received of
*		the requested byte count. A value of zero indicates all requested
*		bytes were received.
*
* @return	None.
*
* @notes	None.
*
****************************************************************************/
static void RecvHandler(void *CallbackRef, int ByteCount)
{
	/* Remove compiler warning about unused parameter. */
	( void ) CallbackRef;
	HandlerInfo.RemainingRecvBytes = ByteCount;
	HandlerInfo.RecvBytesUpdated = TRUE;
}

/*****************************************************************************/
/**
* This status handler is called asynchronously from an interrupt context and
* indicates that the conditions of the IIC bus changed. This  handler should
* not be called for the application unless an error occurs.
*
* @param	CallBackRef is a pointer to the IIC device driver instance which the
*		handler is being called for.
* @param	Status contains the status of the IIC bus which changed.
*
* @return	None.
*
* @notes	None.
*
****************************************************************************/
static void StatusHandler(void *CallbackRef, int Status)
{
	/* Remove compiler warning about unused parameter. */
	( void ) CallbackRef;
	HandlerInfo.EventStatus |= Status;
	HandlerInfo.EventStatusUpdated = TRUE;
}


/*****************************************************************************/
/**
* This Send handler is called asynchronously from an interrupt
* context and indicates that data in the specified buffer has been sent.
*
* @param	InstancePtr is not used, but contains a pointer to the IIC
*		device driver instance which the handler is being called for.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void SendHandler(void *CallbackRef, int ByteCount)
{
	/* Remove compiler warning about unused parameter. */
	( void ) CallbackRef;
	(void) ByteCount;
	HandlerInfo.TransmitComplete = TRUE;
}