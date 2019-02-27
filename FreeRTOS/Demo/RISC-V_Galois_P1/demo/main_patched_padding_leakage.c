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

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Demo includes */
#include "patched_padding_leakage.h"

/*-----------------------------------------------------------*/

/*
 * Called by main when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1 in
 * main.c.
 */
void main_patched_padding_leakage( void );

/*-----------------------------------------------------------*/

/* Variable to hold handle of Task2 */
TaskHandle_t xTask2Handle = NULL;

/*-----------------------------------------------------------*/

void main_patched_padding_leakage( void )
{
	/* Create queue for sending data from Task1 to Task2 */
	xQueue = xQueueCreate( 1, sizeof( struct test ) ); 

	if( xQueue != NULL ) {
		
		/* Create task to check alignment rules */ 
		xTaskCreate( vTaskCheck, "Check Align", 1000, NULL, 1, NULL ); 

		/* Create Task1 */
		xTaskCreate( prvTask1, "Task1", 1000, NULL, 2, NULL );

		/* Create Task2 */ 
		xTaskCreate( vTask2, "Task2", 1000, NULL, 1, &xTask2Handle );  

		/* Start the kernel. From here on, only tasks and interrupts will run. */ 
		vTaskStartScheduler();
	} else {
		printf( "The queue could not be created." );
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

/* Used to check processor-specific alignment
   Only used for debugging. */
void vTaskCheck( void *pvParameters ) {
	
	(void) pvParameters;

	printf( "Alignment of char = %zu\n", alignof( char ) );
	printf( "Alignment of int = %zu\n", alignof( char ) );
	printf( "Alignment of struct = %zu\n", alignof( struct { int a; char b; int c; } ) );

	vTaskDelete( NULL );
}

/*-----------------------------------------------------------*/

/* Print bytes, used to demonstrate leakage in char padding */
void printBytes( void *ptr, int size ) {

	unsigned char *p = ptr;
	int i;
	for( i=(size-1); i>=0; i-- ) {
		printf( "%02hhX", p[i] );
	} 
	printf("\n");
}

/*-----------------------------------------------------------*/

/* Task1 writes secret to memory, then writes a struct to the same
   memory location. It sends a copy of the elements of that struct
   without any padding bytes to Task2, which leads to leakage of 
   the secret. */
void prvTask1( void *pvParameters ) {

	(void) pvParameters;

	/* Status of queue */
	BaseType_t xStatus;

	/* Allocate memory for secret */ 
	int* secret_mem = pvPortMalloc( 3 * sizeof( int ) );

		
	/* Write secret */
	int secret[3] = { 0xffffffff, 0xffffffff, 0xffffffff };	
	if( secret_mem != NULL ) {
		secret_mem = secret;
	} else {
		printf( "Memory could not be allocated for secret\n" );
		vTaskDelete( NULL );
	}	

	/* Check that secret is same size as struct test */	
	assert( sizeof( secret ) == sizeof( struct test ) );

	/* Cast pointer from int pointer to struct pointer */ 
	struct test* struct_mem = (struct test *) secret_mem;

	/* Check for intermediate padding */
	static_assert( offsetof( struct test, c ) == 
			offsetof( struct test, padding_3 ) + 1,
			"Struct contains intermediate padding" );

	/* Check for trailing padding */
	static_assert( sizeof( struct test ) == 
			offsetof( struct test, c ) + sizeof( int ),
			"Struct contains trailing padding" );

	/* Write struct to allocated memory */ 
	if( struct_mem != NULL ) {
		struct_mem->a = 1;
		struct_mem->b = 2;
		struct_mem->c = 3;
		struct_mem->padding_1 = 0;
		struct_mem->padding_2 = 0;
		struct_mem->padding_3 = 0;
	} else {
		printf( "Memory could not be allocated for struct\n" );
		vTaskDelete( NULL );
	}

	/* Send data to Task2 using queue, which queues by copy */
	xStatus = xQueueSend( xQueue, struct_mem, 0 );	
	if( xStatus != pdPASS ) {
		printf( "Could not send to the queue. \r\n" );
	}

	/* Get priority of Task1 */ 
	UBaseType_t uxPriority;
	uxPriority = uxTaskPriorityGet( NULL );

	/* Set Task2 priority to above Task1 priority */
	vTaskPrioritySet( xTask2Handle, (uxPriority + 1) );

	/* Delete task when done */
	vTaskDelete( NULL );

}

/*-----------------------------------------------------------*/

/* Task2 receives struct and reveals leakage through padding bytes */
void vTask2 ( void *pvParameters ) {

	(void) pvParameters;

	/* Status of queue */
	BaseType_t xStatus;
	
	/* Receive struct from Task1 through queue */ 
	
	// vv should this be malloc instead?
	struct test lReceivedStruct;
	xStatus = xQueueReceive( xQueue, &lReceivedStruct, 0 );	

	/* Print struct, including padding bytes */ 
	if( xStatus == pdPASS ) {
		printBytes( ( &lReceivedStruct.b ), 4 );
	} else {
		printf( "Could not receive from the queue. \r\n" );
	}	

	vTaskDelete( NULL );
}

/*-----------------------------------------------------------*/