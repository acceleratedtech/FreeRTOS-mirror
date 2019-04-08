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
 *
 * See the padding_leakage_explanation for a full explanation of this code.
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
#include "padding_leakage.h"

/*-----------------------------------------------------------*/

/*
 * Called by main when PROG=main_padding_leakage
 */
void main_padding_leakage( void );

/*-----------------------------------------------------------*/

/* Variable to hold handle of Task2 */
TaskHandle_t xTask2Handle = NULL;

/*-----------------------------------------------------------*/

void main_padding_leakage( void )
{
	/* Create queue for sending data from Task1 to Task2 */
	xQueue = xQueueCreate( 1, sizeof( struct test ) ); 

	if( xQueue != NULL ) {

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
   memory location. It sends a copy of that struct to Task2,
   which leads to leakage of the secret. */
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

	/* Write struct to allocated memory */ 
	if( struct_mem != NULL ) {
		struct_mem->a = 1;
		struct_mem->b = 2;
		struct_mem->c = 3;
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
	struct test lReceivedStruct;
	xStatus = xQueueReceive( xQueue, &lReceivedStruct, 0 );	

	/* Print struct, including padding bytes */ 
	if( xStatus == pdPASS ) {
		printBytes( ( &lReceivedStruct.b ), 4 );
	} else {
		printf( "Could not receive from the queue. \r\n" );
	}

	/* Delete task when done */
	vTaskDelete( NULL ); 
}

/*-----------------------------------------------------------*/