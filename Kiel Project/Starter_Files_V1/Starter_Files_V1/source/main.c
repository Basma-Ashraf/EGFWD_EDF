/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc21xx.h"

/* Peripheral includes. */
//#include "main.h"
#include "serial.h"
#include "GPIO.h"


													

QueueHandle_t xQueueButton_1=NULL ;
QueueHandle_t xQueueButton_2=NULL ;
QueueHandle_t xQueuerTransmiter=NULL;
uint8_t i;

					
typedef struct
{
	pinState_t Button_1_State;
	pinState_t Button_2_State;

}Global_State;

Global_State buttom_states , button_Prev_state;

Global_State * Buttom_States = &buttom_states ;
Global_State * Buttom_Prev_States = &button_Prev_state ;

typedef enum
{	NONE,
	RISING_EDGE,
	FALLING_EDGE,
	PERIODIC_STRING
}EDGE_SENSE;

typedef struct
{
	EDGE_SENSE BUTTOM_1_EDGE;
	EDGE_SENSE BUTTOM_2_EDGE;
	EDGE_SENSE PERIODIC;

}GLOBAL_EDGE_SENSE_STATE;

GLOBAL_EDGE_SENSE_STATE global_Edge_sense;
GLOBAL_EDGE_SENSE_STATE * Global_Edge = &global_Edge_sense;

GLOBAL_EDGE_SENSE_STATE UART_BUFF;


#define NUMBER_OF_TASKS				((uint8_t)6)	
						
#define PROBE_TICK						PIN0
#define PROBE_TASK_1					PIN1
#define PROBE_TASK_2					PIN2	
#define PROBE_TASK_3					PIN3
#define PROBE_TASK_4					PIN4	
#define PROBE_TASK_5					PIN5
#define PROBE_TASK_6					PIN6				
#define PROBE_IDLE						PIN7
														
#define BUTTON_1_PERIODICITY		   ((uint8_t)50)													
#define BUTTON_2_PERIODICITY		  ((uint8_t)50)													
#define TRANSMIT_PERIODICITY	   	((uint8_t)100)													
#define RECEIVE_PERIODICITY	    	((uint8_t)20)																																						
#define SIMULATION_1_PERIODICITY		((uint8_t)10)
#define SIMULATION_2_PERIODICITY		((uint8_t)100)
														
#define ET_TASK_5						  ((uint8_t)5)
#define ET_TASK_6							((uint8_t)12)														
														
#define ET_2_COUNT_MAP				((uint16_t)6666)
#define DUMMY_ET(ET)																									\
					do{																													\
							uint32_t  i;																						\
							for(i=0; i<(ET * ET_2_COUNT_MAP); i++){									\
								i=i;																									\
							}																												\
					}while(0)

					
#define MAX_QUEUE_WAIT_TIME 		((uint8_t)5)
#define QUEUE_LENGTH						((uint8_t)10)	
#define MESSAGE_BUFFER_SIZE			((uint8_t)25)

#define PULSE_TICK() 																										\
					do{																														\
						GPIO_write(PROBE_PORT, PROBE_TICK, PIN_IS_HIGH);						\
						GPIO_write(PROBE_PORT, PROBE_TICK, PIN_IS_LOW);							\
					}while(0)
					

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


typedef enum{
	RISING,
	FALLING,
	NO_CHANGE
}buttonEdge_t;

typedef struct{
		uint8_t ucMessageID;
		char ucData[MESSAGE_BUFFER_SIZE];
}message_t;

typedef struct{
	uint32_t inTime;
	//uint32_t outTime;
	uint32_t totalTime;
}TaskTime_t;

typedef struct{
	TaskTime_t taskTime[NUMBER_OF_TASKS + 1];
	uint32_t temp;
	uint32_t cpu_Load;
}strCPU_STATUS;


strCPU_STATUS CPU_Status;








#if ( configUSE_EDF_SCHEDULER == 1 )
	BaseType_t xTaskPeriodicCreate( TaskFunction_t pxTaskCode,
													const char * const pcName, /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
													const configSTACK_DEPTH_TYPE usStackDepth,
													void * const pvParameters,
													UBaseType_t uxPriority,
													TaskHandle_t * const pxCreatedTask ,
													TickType_t period);
#endif
													
													
/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/

				

void vApplicationTickHook(void){
		PULSE_TICK();
}

void vApplicationIdleHook (void){
	#if (PERFORMANCE_EVALUATION == 1)	
		for(i=1; i<NUMBER_OF_TASKS+1; i++){
			CPU_Status.temp += CPU_Status.taskTime[i].totalTime ;
		}
		CPU_Status.cpu_Load = (CPU_Status.temp/ (float) T1TC) * 100;
		CPU_Status.temp = 0;
	#endif
}
/****************************Button_1_Monitor**********************/
void Button_1_Monitor(void *param){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	Buttom_Prev_States->Button_1_State = GPIO_read(PORT_1 , PIN1);
	
	//volatile uint8_t Edge_Flag = 0;
	
	vTaskSetApplicationTaskTag( NULL, (void *) 1 );

	
	while(1)
	{
		/* Task code goes Here. */
		Buttom_States->Button_1_State = GPIO_read(PORT_1 , PIN1);
		
		/*	RISING EDGE	*/
		if((Buttom_States->Button_1_State == PIN_IS_HIGH) && (Buttom_Prev_States->Button_1_State == PIN_IS_LOW ) )
		{
			Global_Edge->BUTTOM_1_EDGE = RISING_EDGE;
			if(xQueueButton_1 != 0)
			{
				xQueueSend(xQueueButton_1,(void *)&Global_Edge->BUTTOM_1_EDGE ,(TickType_t)0);
			}
		}
		
		
		/*	FALLING EDGE	*/
		if((Buttom_States->Button_1_State == PIN_IS_LOW ) && (Buttom_Prev_States->Button_1_State == PIN_IS_HIGH ) )
		{
			Global_Edge->BUTTOM_1_EDGE = FALLING_EDGE;
			
			if(xQueueButton_1 != 0)
			{
				xQueueSend(xQueueButton_1,(void *)&Global_Edge->BUTTOM_1_EDGE ,(TickType_t)0);
			}
		}

		
		Buttom_Prev_States->Button_1_State = Buttom_States->Button_1_State;		//Storing the new state in the Previous State.
		
		vTaskDelayUntil(&xLastWakeTime , BUTTON_1_PERIODICITY);
		
		/*	IDLE Task Identification*/
	  GPIO_write(PORT_0, PIN0, PIN_IS_LOW);

		
	}
}

///***************************Button_2_Monitor **********************/
void Button_2_Monitor(void *param){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	Buttom_Prev_States->Button_2_State = GPIO_read(PORT_1 , PIN2);
	
	//volatile uint8_t Edge_Flag = 0;
	
	vTaskSetApplicationTaskTag( NULL, (void *) 2 );

	
	while(1)
	{
		/* Task code goes Here. */
		Buttom_States->Button_2_State = GPIO_read(PORT_1 , PIN2);
		
		/*	RISING EDGE	*/
		if((Buttom_States->Button_2_State == PIN_IS_HIGH) && (Buttom_Prev_States->Button_2_State == PIN_IS_LOW ) )
		{
			Global_Edge->BUTTOM_2_EDGE = RISING_EDGE;
			if(xQueueButton_2 != 0)
			{
				xQueueSend(xQueueButton_2 ,(void *)&Global_Edge->BUTTOM_2_EDGE ,(TickType_t)0);
			}
		}
		
		/*	FALLING EDGE	*/
		if((Buttom_States->Button_2_State == PIN_IS_LOW ) && (Buttom_Prev_States->Button_2_State == PIN_IS_HIGH ) )
		{
			Global_Edge->BUTTOM_2_EDGE = FALLING_EDGE;
			
			if(xQueueButton_2 != 0)
			{
				xQueueSend(xQueueButton_2 ,(void *)&Global_Edge->BUTTOM_2_EDGE ,(TickType_t)0);
			}
		}

		
		
		Buttom_Prev_States->Button_2_State = Buttom_States->Button_2_State;		//Storing the new state in the Previous State.
		
		vTaskDelayUntil(&xLastWakeTime , BUTTON_2_PERIODICITY);
		
		/*	IDLE Task Identification*/
	  GPIO_write(PORT_0, PIN0, PIN_IS_LOW);

		
	}
}

/******************************Periodic_Transmitter **********************/
void Periodic_Transmitter(void *param){
	volatile char*	String_Periodic;
	 TickType_t xLastWakeTime = xTaskGetTickCount();
	
	Global_Edge->PERIODIC = PERIODIC_STRING;

	vTaskSetApplicationTaskTag( NULL, (void *) 3 );

	while(1)
	{
		String_Periodic = "Continous\n";
		
		if(xQueuerTransmiter != 0)
    {
			xQueueSend( xQueuerTransmiter, ( void * )&Global_Edge->PERIODIC, (TickType_t) 0 );
    }
		
		
		vTaskDelayUntil(&xLastWakeTime , RECEIVE_PERIODICITY);
		
		/*	IDLE Task Identification*/
	  GPIO_write(PORT_0, PIN0, PIN_IS_LOW);
	}
}

 

/****************************Periodic_Receiver **********************/
void Periodic_Receiver(void *param){
	
	char* BUTTON_1_ON ="Button 1 ON\n";
	char* BUTTON_1_OFF="Button 1 OFF\n";
	char* BUTTON_2_ON ="Button 2 ON\n";
	char* BUTTON_2_OFF="Button 2 OFF\n";
	
	char* Periodic_String ="Continous\n";
	
	char receiverBuffer;

	
	 TickType_t xLastWakeTime = xTaskGetTickCount();

	vTaskSetApplicationTaskTag( NULL, (void *) 4 );
	
	

	while(1)
	{
		
		if(xQueueButton_1!=NULL)
    {
			if ((xQueueReceive(xQueueButton_1,(void *) &(receiverBuffer), (TickType_t)0)) == pdPASS )
      {
         xSerialPutChar('\n');
				if (receiverBuffer == RISING_EDGE) vSerialPutString((const signed char * const)BUTTON_1_ON, 12);
				else if (receiverBuffer == FALLING_EDGE) vSerialPutString((const signed char * const)BUTTON_1_OFF, 13);
      }
			
		}	
		
		
		if(xQueueButton_2 !=NULL)
    {
			if ((xQueueReceive(xQueueButton_2,(void *) &(receiverBuffer), (TickType_t)0)) == pdPASS )
      {
         xSerialPutChar('\n');
				if (receiverBuffer == RISING_EDGE) vSerialPutString((const signed char * const)BUTTON_2_ON, 12);
				else if (receiverBuffer == FALLING_EDGE) vSerialPutString((const signed char * const)BUTTON_2_OFF, 13);
      }
			
		}	


		if(xQueuerTransmiter !=NULL)
    {
			if ((xQueueReceive(xQueuerTransmiter,(void *) &(receiverBuffer), (TickType_t)0)) == pdPASS )
      {
				if (receiverBuffer == PERIODIC_STRING) vSerialPutString((const signed char * const)Periodic_String, 10);
      }
			
		}	
		
		
		vTaskDelayUntil(&xLastWakeTime , RECEIVE_PERIODICITY);
		
		/*	IDLE Task Identification*/
	  GPIO_write(PORT_0, PIN0, PIN_IS_LOW);
	}
}


/****************************** Task 5 **********************/
void Task_5(void *param){
	TickType_t xLastWakeTime;
	vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)5);
	
	xLastWakeTime = xTaskGetTickCount();
	for(;;){
		DUMMY_ET(ET_TASK_5);
		vTaskDelayUntil( &xLastWakeTime, SIMULATION_1_PERIODICITY );
	}
}


/*****************************Task 6 **********************/
void Task_6(void *param){
	TickType_t xLastWakeTime;
	vTaskSetApplicationTaskTag(NULL, (TaskHookFunction_t)6);
	
	xLastWakeTime = xTaskGetTickCount();
	for(;;){	
		DUMMY_ET(ET_TASK_6);
		vTaskDelayUntil( &xLastWakeTime, SIMULATION_2_PERIODICITY );
	}
}

/******************************************************************************/
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	
	
	xQueueButton_1    = xQueueCreate( QUEUE_LENGTH,sizeof(char*) );
	xQueueButton_2    = xQueueCreate( QUEUE_LENGTH,sizeof(char*) );	
	xQueuerTransmiter = xQueueCreate( QUEUE_LENGTH,sizeof(char*) );

	xTaskPeriodicCreate(
		Button_1_Monitor, 
		"Button_1_Monitor", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL,
		(UBaseType_t) 0, 
		(TaskHandle_t *)NULL, 
		BUTTON_1_PERIODICITY);
		
	xTaskPeriodicCreate(
		Button_2_Monitor, 
		"Button_2_Monitor", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL,
		(UBaseType_t)0, 
		(TaskHandle_t *)NULL, 
		BUTTON_2_PERIODICITY);
		
	xTaskPeriodicCreate(
		Periodic_Transmitter, 
		"Periodic_Transmitter", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL,
		(UBaseType_t)0, 
		(TaskHandle_t *)NULL, 
		TRANSMIT_PERIODICITY);

		
	xTaskPeriodicCreate(
		Periodic_Receiver, 
		"Uart_Receiver", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL,
		(UBaseType_t)0, 
		(TaskHandle_t *)NULL, 
		RECEIVE_PERIODICITY);
		
	xTaskPeriodicCreate(
		Task_5, 
		"Load_1_Simulation", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL,
		(UBaseType_t)0, 
		(TaskHandle_t *)NULL, 
		SIMULATION_1_PERIODICITY);
		
	xTaskPeriodicCreate(
		Task_6, 
		"Load_2_Simulation", 
		configMINIMAL_STACK_SIZE, 
		(void *)NULL, 
		(UBaseType_t) 0, 
		(TaskHandle_t *)NULL, 
		SIMULATION_2_PERIODICITY);
	
	
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

