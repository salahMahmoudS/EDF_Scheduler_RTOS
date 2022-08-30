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

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

/* Project definitions and MACROS */
#define ENABLED									(1)
#define DISABLED								(2)

 							 
#define BUTTON_1_TASK					(DISABLED)
#define BUTTON_2_TASK					(DISABLED)
#define TRANSMITTER_TASK	    (DISABLED)
#define UART_RECEIVER_TASK    (DISABLED )
#define LOAD_1_SIM_TASK		    (DISABLED)
#define LOAD_2_SIM_TASK		    (ENABLED)


#define BUTTON_1_TASK_PERIOD				(50	)
#define BUTTON_2_TASK_PERIOD				(50	)
#define TRANSMITTER_TASK_PERIOD	    (100)
#define UART_RECEIVER_TASK_PERIOD   (20 )
#define LOAD_1_SIM_TASK_PERIOD		  (10 ) 
#define LOAD_2_SIM_TASK_PERIOD		  (100) 

/* Corresponding Toggle pins for each task for logic analyzer*/
#define TICK_PIN								(PIN0)  /*16*/
#define BUTTON_1_TASK_PIN				(PIN1)  /*17*/
#define BUTTON_2_TASK_PIN				(PIN2)  /*18*/
#define TRANSMITTER_TASK_PIN		(PIN3)  /*19*/
#define UART_RECEIVER_TASK_PIN	(PIN4)  /*20*/
#define LOAD_1_SIM_TASK_PIN			(PIN5)  /*21*/
#define LOAD_2_SIM_TASK_PIN			(PIN6)  /*22*/
#define IDLE_TASK_PIN						(PIN7)  /*23*/
#define BUTTON1_SWITCH_PIN			(PIN8)  /*24*/
#define BUTTON2_SWITCH_PIN			(PIN9)  /*25*/

/* Declare and define global variables */
TaskHandle_t Button1TaskHandler;
TaskHandle_t Button2TaskHandler;
TaskHandle_t TransmitterTaskHandler;
TaskHandle_t UartTaskHandler;
TaskHandle_t LoadOneSimulatorTaskHandler;
TaskHandle_t LoadTwoSimulatorTaskHandler;

typedef struct
{
	TickType_t 			timeStamp;
	pinX_t     			messageSender;
	unsigned char 	messageLength;
	char * 					messageString;
}xMessage;


QueueHandle_t messageQueue1;

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware();


/* 
 *Tasks definitoin goes here
 */
 
void vApplicationTickHook( void )
{
			GPIO_write(PORT_0, TICK_PIN		, PIN_IS_HIGH);
			GPIO_write(PORT_0, TICK_PIN		, PIN_IS_LOW);

}

void vApplicationIdleHook( void )
{
			GPIO_write(PORT_0, LOAD_1_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, BUTTON_1_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, BUTTON_2_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, TRANSMITTER_TASK_PIN	, PIN_IS_LOW);
			GPIO_write(PORT_0, UART_RECEIVER_TASK_PIN, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_1_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_2_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, IDLE_TASK_PIN					, PIN_IS_HIGH);
			

}


void Button1Monitor(void * PvParameters)
{
	xMessage ButtonMessage;
	char queue1Size;
	static pinState_t Button1OldState = PIN_IS_HIGH;
	/* variable contains last time task is up*/
	TickType_t xLastWakeTime;
	/* variables that will hold button state if event ocured */
  xLastWakeTime = xTaskGetTickCount();
	GPIO_write(PORT_0, BUTTON_1_TASK_PIN		, PIN_IS_HIGH);

	for (;;)
	{
		
			
			GPIO_write(PORT_0, BUTTON_1_TASK_PIN			, PIN_IS_HIGH);
			GPIO_write(PORT_0, BUTTON_2_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, TRANSMITTER_TASK_PIN	, PIN_IS_LOW);
			GPIO_write(PORT_0, UART_RECEIVER_TASK_PIN, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_1_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_2_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, IDLE_TASK_PIN					, PIN_IS_LOW);
		
		if (Button1OldState == PIN_IS_HIGH && GPIO_read(PORT_0,BUTTON1_SWITCH_PIN) == PIN_IS_LOW)
		{
			ButtonMessage.messageLength = 3;
			ButtonMessage.messageString = "F1\n";
			Button1OldState = PIN_IS_LOW;
		}
		else if (Button1OldState == PIN_IS_LOW && GPIO_read(PORT_0,BUTTON1_SWITCH_PIN) == PIN_IS_HIGH)
		{
			ButtonMessage.messageLength = 3;
			ButtonMessage.messageString = "R1\n";
			Button1OldState = PIN_IS_HIGH;
		}
		else
		{
			ButtonMessage.messageString = NULL;
			ButtonMessage.messageLength = 0;
		}
		
		if (ButtonMessage.messageLength != 0 && messageQueue1 !=NULL)
		{
			ButtonMessage.messageSender = BUTTON1_SWITCH_PIN;
			ButtonMessage.timeStamp = xLastWakeTime;
			
			xQueueSendToBack(messageQueue1, (void*)&ButtonMessage,3);
			queue1Size = uxQueueMessagesWaiting(messageQueue1);
		}
	
		GPIO_write(PORT_0, BUTTON_1_TASK_PIN		, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime,BUTTON_1_TASK_PERIOD);
	}
}

void Button2Monitor(void * PvParameters)
{
	xMessage ButtonMessage;
	char queue1Size;
	static pinState_t Button2OldState = PIN_IS_HIGH;
	/* variable contains last time task is up*/
	TickType_t xLastWakeTime;
	/* variables that will hold button state if event ocured */
  xLastWakeTime = xTaskGetTickCount();
	GPIO_write(PORT_0, BUTTON_2_TASK_PIN		, PIN_IS_HIGH);

	for (;;)
	{
		
			
			GPIO_write(PORT_0, BUTTON_1_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, BUTTON_2_TASK_PIN			, PIN_IS_HIGH);
			GPIO_write(PORT_0, TRANSMITTER_TASK_PIN	, PIN_IS_LOW);
			GPIO_write(PORT_0, UART_RECEIVER_TASK_PIN, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_1_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_2_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, IDLE_TASK_PIN					, PIN_IS_LOW);
		
		if (Button2OldState == PIN_IS_HIGH && GPIO_read(PORT_0,BUTTON2_SWITCH_PIN) == PIN_IS_LOW)
		{
			ButtonMessage.messageLength = 3;
			ButtonMessage.messageString = "F2\n";
			Button2OldState = PIN_IS_LOW;
		}
		else if (Button2OldState == PIN_IS_LOW && GPIO_read(PORT_0,BUTTON2_SWITCH_PIN) == PIN_IS_HIGH)
		{
			ButtonMessage.messageLength = 3;
			ButtonMessage.messageString = "R2\n";
			Button2OldState = PIN_IS_HIGH;
		}
		else
		{
			ButtonMessage.messageLength = 0;
			ButtonMessage.messageString = NULL;
		}
		
		if (ButtonMessage.messageLength != 0 && messageQueue1 !=NULL)
		{
			ButtonMessage.messageSender = BUTTON2_SWITCH_PIN;
			
			ButtonMessage.timeStamp = xLastWakeTime;
			xQueueSendToBack(messageQueue1, (void*)&ButtonMessage,3);
			queue1Size = uxQueueMessagesWaiting(messageQueue1);
		}
	
		GPIO_write(PORT_0, BUTTON_2_TASK_PIN		, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime,BUTTON_2_TASK_PERIOD);
	}
}


void Periodic_Transmitter (void * PvParameters)
{
	char queue1Size;
	/* variable contains last time task is up*/
	TickType_t xLastWakeTime;
	/* variables that will periodic message */
	xMessage PeriodicMessage;
  xLastWakeTime = xTaskGetTickCount ();
	GPIO_write(PORT_0, TRANSMITTER_TASK_PIN		, PIN_IS_HIGH);
	for(;;)
	{
			GPIO_write(PORT_0, BUTTON_1_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, BUTTON_2_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, TRANSMITTER_TASK_PIN	, PIN_IS_HIGH);
			GPIO_write(PORT_0, UART_RECEIVER_TASK_PIN, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_1_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_2_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, IDLE_TASK_PIN					, PIN_IS_LOW);
		PeriodicMessage.messageLength = 16;
		PeriodicMessage.messageSender = TRANSMITTER_TASK_PIN;
		PeriodicMessage.messageString = "Project Success\n";
			
		if (messageQueue1 != NULL)
		{
			xQueueSendToBack(messageQueue1, (void*)&PeriodicMessage,3);
			queue1Size = uxQueueMessagesWaiting(messageQueue1);
		}
		GPIO_write(PORT_0, TRANSMITTER_TASK_PIN		, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime,TRANSMITTER_TASK_PERIOD);
	}
}


void UartReceiver (void * PvParameters)
{
	xMessage RxedMessage;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount ();
	GPIO_write(PORT_0, UART_RECEIVER_TASK_PIN, PIN_IS_HIGH);
	
	
	for(;;)
	{
			GPIO_write(PORT_0, BUTTON_1_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, BUTTON_2_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, TRANSMITTER_TASK_PIN	, PIN_IS_LOW);
			GPIO_write(PORT_0, UART_RECEIVER_TASK_PIN, PIN_IS_HIGH);
			GPIO_write(PORT_0, LOAD_1_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_2_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, IDLE_TASK_PIN					, PIN_IS_LOW);
			
		if (messageQueue1 != NULL)
		{
			char queue1Size = uxQueueMessagesWaiting(messageQueue1);
			while (uxQueueMessagesWaiting(messageQueue1) > 0)
			{
				xQueueReceive(messageQueue1, (void*)&RxedMessage, (TickType_t) (2) == 1);
				vSerialPutString(RxedMessage.messageString, RxedMessage.messageLength);
				
			}
		}
		GPIO_write(PORT_0, UART_RECEIVER_TASK_PIN		, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime,UART_RECEIVER_TASK_PERIOD);
	}	
}


void LoadOneSimulator(void * PvParameters)
{
	TickType_t xLastWakeTime;
	unsigned long i;
  xLastWakeTime = xTaskGetTickCount ();
	GPIO_write(PORT_0, LOAD_1_SIM_TASK_PIN		, PIN_IS_HIGH);

	for (;;)
	{
		for (i=0; i<996;i++)
		{
			GPIO_write(PORT_0, BUTTON_1_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, BUTTON_2_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, TRANSMITTER_TASK_PIN	, PIN_IS_LOW);
			GPIO_write(PORT_0, UART_RECEIVER_TASK_PIN, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_1_SIM_TASK_PIN		, PIN_IS_HIGH);
			GPIO_write(PORT_0, LOAD_2_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, IDLE_TASK_PIN					, PIN_IS_LOW);
		}
		
		GPIO_write(PORT_0, LOAD_1_SIM_TASK_PIN		, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime,LOAD_1_SIM_TASK_PERIOD);
	}
}


void LoadTwoSimulator(void * PvParameters)
{
	TickType_t xLastWakeTime;
	unsigned long i;
  xLastWakeTime = xTaskGetTickCount ();
	GPIO_write(PORT_0, LOAD_2_SIM_TASK_PIN		, PIN_IS_HIGH);

	for (;;)
	{
		for (i=0; i<2364;i++)
		{
			GPIO_write(PORT_0, BUTTON_1_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, BUTTON_2_TASK_PIN			, PIN_IS_LOW);
			GPIO_write(PORT_0, TRANSMITTER_TASK_PIN	, PIN_IS_LOW);
			GPIO_write(PORT_0, UART_RECEIVER_TASK_PIN, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_1_SIM_TASK_PIN		, PIN_IS_LOW);
			GPIO_write(PORT_0, LOAD_2_SIM_TASK_PIN		, PIN_IS_HIGH);
			GPIO_write(PORT_0, IDLE_TASK_PIN					, PIN_IS_LOW);
		}
		GPIO_write(PORT_0, LOAD_2_SIM_TASK_PIN		, PIN_IS_LOW);
		vTaskDelayUntil(&xLastWakeTime,LOAD_2_SIM_TASK_PERIOD);
	}
}
/*-----------------------------------------------------------*/


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	
    /* Create Tasks here */
	#if (LOAD_1_SIM_TASK == ENABLED)
		xTaskCreatePeriodic(	LoadOneSimulator,
													"Load 1 Simulator",
													200,
													(void *)0,
													1,
													LOAD_1_SIM_TASK_PERIOD,
													&LoadOneSimulatorTaskHandler);
	#endif

	#if (LOAD_2_SIM_TASK == ENABLED)
		xTaskCreatePeriodic(	LoadTwoSimulator,
													"Load 2 Simulator",
													200,
													(void *)0,
													1,
													LOAD_2_SIM_TASK_PERIOD,
													&LoadTwoSimulatorTaskHandler);
	#endif
								
	#if (BUTTON_1_TASK == ENABLED)
		xTaskCreatePeriodic(	Button1Monitor,
													"Button 1 Monitor",
													200,
													(void *)0,
													1,
													BUTTON_1_TASK_PERIOD,
													&Button1TaskHandler);
	
	#endif
	
													
	#if (BUTTON_2_TASK == ENABLED)
		xTaskCreatePeriodic(	Button2Monitor,
													"Button 2 Monitor",
													200,
													(void *)0,
													1,
													BUTTON_2_TASK_PERIOD,
													&Button2TaskHandler);
	
	#endif
													
													
	#if (TRANSMITTER_TASK == ENABLED)
	xTaskCreatePeriodic(	Periodic_Transmitter,
												"Periodic Transmitter",
												200,
												(void *)0,
												1,
												TRANSMITTER_TASK_PERIOD,
												&TransmitterTaskHandler);
	#endif	

	#if (UART_RECEIVER_TASK == ENABLED)
	   xTaskCreatePeriodic(	UartReceiver,
													"UART receiver",
													200,
													(void *)0,
													1,
													UART_RECEIVER_TASK_PERIOD,
													&UartTaskHandler);
	#endif	
												
messageQueue1 =  xQueueCreate( 10, sizeof( xMessage ) );
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


