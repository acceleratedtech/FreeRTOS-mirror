/*
padding_leakage.h
Header file for main_padding_leakage.c, which demonstrates information 
leakage through padding bytes in a struct.
*/

#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <stdalign.h>
#include <string.h>
#include <assert.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Struct test to demonstrate leakage */
struct test {
	int a;
	char b;
	int c;
};

/* Queue handle */  
QueueHandle_t xQueue;

/* Variable for holding handle of Task2 */
TaskHandle_t xTask2Handle;

void vTaskCheck( void *pvParameters );

void printBytes( void *ptr, int size );

void prvTask1( void *pvParameters );

void vTask2( void *pvParameters );
