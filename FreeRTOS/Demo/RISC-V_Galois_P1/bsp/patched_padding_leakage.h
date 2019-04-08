/*
patched_padding_leakage.h
Header file for main_patched_padding_leakage.c, which demonstrates a patch 
of an information leakage vulerability through padding bytes in a struct.
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
	char padding_1, padding_2, padding_3;
	int c;
};

/* Queue handle */  
QueueHandle_t xQueue;

/* Variable for holding handle of Task2 */
TaskHandle_t xTask2Handle;

void printBytes( void *ptr, int size );

void prvTask1( void *pvParameters );

void vTask2( void *pvParameters );
