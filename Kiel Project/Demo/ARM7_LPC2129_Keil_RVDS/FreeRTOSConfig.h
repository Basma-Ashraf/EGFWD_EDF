<<<<<<< HEAD

=======
>>>>>>> e62a9df9d2219aaa77dcf4e90bcc898a55488de3
/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <lpc21xx.h>
<<<<<<< HEAD
#include "GPIO.h"


extern int Button1_TaskInTime  , Button1_TaskOutTime , Button1_TaskTotalTime    ;
extern int Button2_TaskInTime  , Button2_TaskOutTime  , Button2_TaskTotalTime    ;
extern int Periodic_TaskInTime       , Periodic_TaskOutTime       ,Periodic_TaskTotalTime         ;
extern int UART_TaskInTime       , UART_TaskOutTime       , UART_TaskTotalTime         ;
extern int Load1_TaskInTime    , Load1_TaskOutTime    , Load1_TaskTotalTime     ;
extern int Load2_TaskInTime    , Load2_TaskOutTime   , Load2_TaskTotalTime      ;

extern int System_Time ;

extern int CPU_Load ;

=======
>>>>>>> e62a9df9d2219aaa77dcf4e90bcc898a55488de3

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/
<<<<<<< HEAD

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			1
#define configUSE_TICK_HOOK			1   
=======
#define configUSE_EDF_SCHEDULER         1
#define configUSE_APPLICATION_TASK_TAG  1
#define PERFORMANCE_EVALUATION					1

#define PROBE_PORT		  	PORT_0	
#define PROBE_TICK				PIN0
#define PROBE_TASK_1			PIN1
#define PROBE_TASK_2			PIN2	
#define PROBE_TASK_3			PIN3
#define PROBE_TASK_4			PIN4	
#define PROBE_TASK_5			PIN5
#define PROBE_TASK_6			PIN6				
#define PROBE_IDLE				PIN7


#if (PERFORMANCE_EVALUATION == 1)	
	#define traceTASK_SWITCHED_IN()																																		\
		do{																																															\
				GPIO_write(PROBE_PORT, TagToPinMap((uint8_t)(pxCurrentTCB->pxTaskTag)), PIN_IS_HIGH);				\
				CPU_Status.taskTime[(uint8_t)(pxCurrentTCB->pxTaskTag)].inTime = T1TC;						\
		}while(0)

	#define traceTASK_SWITCHED_OUT()																																	\
		do{																																															\
				GPIO_write(PROBE_PORT, TagToPinMap((uint8_t)(pxCurrentTCB->pxTaskTag)), PIN_IS_LOW);				\
				CPU_Status.taskTime[(uint8_t)(pxCurrentTCB->pxTaskTag)].totalTime += 						\
								T1TC - CPU_Status.taskTime[(uint8_t)(pxCurrentTCB->pxTaskTag)].inTime;  	\
		}while(0)
	
#else
		#define traceTASK_SWITCHED_IN()	  	GPIO_write(PROBE_PORT, TagToPinMap((uint8_t)(pxCurrentTCB->pxTaskTag)), PIN_IS_HIGH)
		#define traceTASK_SWITCHED_OUT()    GPIO_write(PROBE_PORT, TagToPinMap((uint8_t)(pxCurrentTCB->pxTaskTag)), PIN_IS_LOW)
#endif

		
#define vApplicationIdleTAG_SET()		vTaskSetApplicationTaskTag(NULL, (void *)0);



#define configSUPPORT_DYNAMIC_ALLOCATION 		1

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			1
#define configUSE_TICK_HOOK			1
>>>>>>> e62a9df9d2219aaa77dcf4e90bcc898a55488de3
#define configCPU_CLOCK_HZ			( ( unsigned long ) 60000000 )	/* =12.0MHz xtal multiplied by 5 using the PLL. */
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES		( 4 )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 90 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) 13 * 1024 )
#define configMAX_TASK_NAME_LEN		( 8 )
<<<<<<< HEAD
#define configUSE_TRACE_FACILITY	1
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		0
#define configQUEUE_REGISTRY_SIZE 	0
#define configUSE_TIME_SLICING    0
#define configUSE_MUTEXES					0

/* Application Tag */

#define configUSE_APPLICATION_TASK_TAG   1



=======
#define configUSE_TRACE_FACILITY	0
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		1


#define configQUEUE_REGISTRY_SIZE 	0
>>>>>>> e62a9df9d2219aaa77dcf4e90bcc898a55488de3

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1


<<<<<<< HEAD
/*Trace Hooks*/



#define traceTASK_SWITCHED_OUT()		   	do\
																				{\
																					if((int)pxCurrentTCB->pxTaskTag == 1 )\
																					{\
																						Button1_TaskTotalTime += (T1TC - Button1_TaskInTime);\
																						GPIO_write(PORT_0, PIN1, PIN_IS_LOW);\
																					}\
																					else if((int)pxCurrentTCB->pxTaskTag == 2)\
																					{\
																						Button2_TaskTotalTime += (T1TC - Button2_TaskInTime);\
																						GPIO_write(PORT_0, PIN2, PIN_IS_LOW);\
																					}\
																					else if((int)pxCurrentTCB->pxTaskTag == 3)\
																					{\
																						Periodic_TaskTotalTime += (T1TC - Periodic_TaskInTime);\
																						GPIO_write(PORT_0, PIN3, PIN_IS_LOW);\
																					}\
																					else if((int)pxCurrentTCB->pxTaskTag == 4)\
																					{\
																						UART_TaskTotalTime += (T1TC - UART_TaskInTime);\
																						GPIO_write(PORT_0, PIN4, PIN_IS_LOW);\
																					}\
																					else if((int)pxCurrentTCB->pxTaskTag == 5)\
																					{\
																						Load1_TaskTotalTime += (T1TC - Load1_TaskInTime);\
																						GPIO_write(PORT_0, PIN5, PIN_IS_LOW);\
																					}\
																					else if((int)pxCurrentTCB->pxTaskTag == 6)\
																					{\
																						Load2_TaskTotalTime += (T1TC - Load2_TaskInTime);\
																						GPIO_write(PORT_0, PIN6, PIN_IS_LOW);\
																					}\
																					System_Time=T1TC;\
																					CPU_Load=(((Button1_TaskTotalTime + Button2_TaskTotalTime + Periodic_TaskTotalTime + UART_TaskTotalTime + Load1_TaskTotalTime + Load2_TaskTotalTime)*100)/ (float)System_Time);\
																				}while(0)
																						
#define traceTASK_SWITCHED_IN()    			do\
																				{\
																					if((int)pxCurrentTCB->pxTaskTag == 1 )\
																					{\
																						GPIO_write(PORT_0, PIN1, PIN_IS_HIGH);\
																						Button1_TaskInTime = T1TC;\
																					}\
																					else if((int)pxCurrentTCB->pxTaskTag == 2)\
																					{\
																						GPIO_write(PORT_0, PIN2, PIN_IS_HIGH);\
																						Button2_TaskInTime = T1TC;\
																					}\
																					else if((int)pxCurrentTCB->pxTaskTag == 3)\
																					{\
																						GPIO_write(PORT_0, PIN3, PIN_IS_HIGH);\
																						Periodic_TaskInTime = T1TC;\
																					}\
																					else if((int)pxCurrentTCB->pxTaskTag == 4)\
																					{\
																						GPIO_write(PORT_0, PIN4, PIN_IS_HIGH);\
																						UART_TaskInTime = T1TC;\
																					}\
																					else if((int)pxCurrentTCB->pxTaskTag == 5)\
																					{\
																						GPIO_write(PORT_0, PIN5, PIN_IS_HIGH);\
																						Load1_TaskInTime = T1TC;\
																					}\
																					else if((int)pxCurrentTCB->pxTaskTag == 6)\
																					{\
																						GPIO_write(PORT_0, PIN6, PIN_IS_HIGH);\
																						Load2_TaskInTime = T1TC;\
																					}\
																				}while(0)


																				
/* Configure Run Time Stats */

#define configUSE_STATS_FORMATTING_FUNCTIONS 		1
#define configGENERATE_RUN_TIME_STATS 		1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()   
#define portGET_RUN_TIME_COUNTER_VALUE()    (T1TC)
																																			
																				
																				
#define configUSE_EDF_SCHEDULER		1
#define IDLE_TASK_PERIOD			300																			
#endif /* FREERTOS_CONFIG_H */
=======

#endif /* FREERTOS_CONFIG_H */
>>>>>>> e62a9df9d2219aaa77dcf4e90bcc898a55488de3
