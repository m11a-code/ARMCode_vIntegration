/*
    FreeRTOS V6.1.1 - Copyright (C) 2011 Real Time Engineers Ltd.

    ***************************************************************************
    *                                                                         *
    * If you are:                                                             *
    *                                                                         *
    *    + New to FreeRTOS,                                                   *
    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
    *    + Looking for basic training,                                        *
    *    + Wanting to improve your FreeRTOS skills and productivity           *
    *                                                                         *
    * then take a look at the FreeRTOS books - available as PDF or paperback  *
    *                                                                         *
    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
    *                  http://www.FreeRTOS.org/Documentation                  *
    *                                                                         *
    * A pdf reference manual is also available.  Both are usually delivered   *
    * to your inbox within 20 minutes to two hours when purchased between 8am *
    * and 8pm GMT (although please allow up to 24 hours in case of            *
    * exceptional circumstances).  Thank you for your support!                *
    *                                                                         *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/*
    This file has been modified for the usage of Virginia Tech ECE4534 Spring 2013 Group 04:
    Authors:
        Matthew Ibarra
        Seo Townsend
        Richard Clermont
        Dhiraj Pandey
*/

// Cleaned up main.c for milestone 2 and beyond
// Matthew Ibarra 2/10/2013

#include "myDefs.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "system_LPC17xx.h"

#ifndef   PCONP_PCTIM0
/* MTJ_NOTE: This will not compile properly if you do not delete the old version of */
/*       system_LPC17xx.h from the Keil compiler installation */
You should read the note above.
#endif

#include "partest.h"

// Include files for rover racer project
#include "vtUtilities.h"
#include "LCDtask.h"
#include "i2c_ARM.h"
#include "vtI2C.h"
#include "myTimers.h"
#include "conductor.h"
#include "motorControl.h"
#include "navigation.h"
#include "speedLimit.h"
#include "myTypes.h"
//#include "myADC.h"
#include "power.h"
#include "webServer.h"

/* syscalls initialization -- *must* occur first */
#include "syscalls.h"
#include "extUSB.h"
#include <stdio.h>
/*-----------------------------------------------------------*/

/* The time between cycles of the 'check' functionality (defined within the
tick hook). */

#define mainCHECK_DELAY						( ( portTickType ) 5000 / portTICK_RATE_MS )

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY)
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY)
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY)
#define mainUIP_TASK_PRIORITY				( tskIDLE_PRIORITY)
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY)
#define mainGEN_QUEUE_TASK_PRIORITY			( tskIDLE_PRIORITY)
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY)
#define mainLCD_TASK_PRIORITY				( tskIDLE_PRIORITY)
#define mainI2C_TASK_PRIORITY		     	( tskIDLE_PRIORITY)
#define mainUSB_TASK_PRIORITY				( tskIDLE_PRIORITY)
#define mainI2CMONITOR_TASK_PRIORITY		( tskIDLE_PRIORITY)
#define mainCONDUCTOR_TASK_PRIORITY			( tskIDLE_PRIORITY)
#define mainMOTOR_CONTROL_TASK_PRIORITY     ( tskIDLE_PRIORITY)
#define mainNAVIGATION_TASK_PRIORITY        ( tskIDLE_PRIORITY)
#define mainIR_CONTROL_TASK_PRIORITY        ( tskIDLE_PRIORITY)
#define mainSPEED_LIMIT_TASK_PRIORITY       ( tskIDLE_PRIORITY)
#define mainPOWER_TASK_PRIORITY             ( tskIDLE_PRIORITY)
#define mainWEB_SERVER_TASK_PRIORITY        ( tskIDLE_PRIORITY)

/* The WEB server has a larger stack as it utilises stack hungry string
handling library calls. */
#define mainBASIC_WEB_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 4 )

/* The message displayed by the WEB server when all tasks are executing
without an error being reported. */
#define mainPASS_STATUS_MESSAGE				"All tasks are executing without error."

/*-----------------------------------------------------------*/

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/*
 * The task that handles the uIP stack.  All TCP/IP processing is performed in
 * this task.
 */
extern void vuIP_Task( void *pvParameters );
extern void vuIP_SetLCD( void *lcdParams );
extern void setUiPWebServer( webServerStruct *web );

/*
 * The task that handles the USB stack.
 */
extern void vUSBTask( void *pvParameters );

/*
 * Simply returns the current status message for display on served WEB pages.
 */
char *pcGetTaskStatusMessage( void );

/*-----------------------------------------------------------*/

/* Holds the status message displayed by the WEB server. */
static char *pcStatusMessage = mainPASS_STATUS_MESSAGE;


// Required data structure for I2C interrupt handler
static vtI2CStruct vtI2C0;
// Required data structure for I2C task
static myI2CStruct i2cData;
// Required data structure for Conductor task
static vtConductorStruct conductorData;
// Required data structure for LCD task
static vtLCDStruct vtLCDdata;
// Required data structure for Motor Control task
static motorControlStruct motorControl;
// Required data structure for Navigation task
static navigationStruct navData;
// Required data structure for IR Control task
static irControlStruct irData;
// Required data structure for Speed Limit task
static speedLimitControlStruct speedData;
// Required data structure for Web Server task
static webServerStruct webData;
// Required data structure for Power task
static powerStruct powerData;

/*-----------------------------------------------------------*/

int main( void )
{
	/* MTJ: initialize syscalls -- *must* be first */
	// syscalls.c contains the files upon which the standard (and portable) C libraries rely
	init_syscalls();

	// Set up the LED ports and turn them off
	vtInitLED();

	/* Configure the hardware for use by this demo. */
	prvSetupHardware();

  vStartLCDTask(&vtLCDdata,mainLCD_TASK_PRIORITY);

	// Not a standard demo -- but also not one of mine (MTJ)
	/* Create the uIP task.  The WEB server runs in this task. */
  xTaskCreate( vuIP_Task, ( signed char * ) "uIP", mainBASIC_WEB_STACK_SIZE, ( void * ) NULL, mainUIP_TASK_PRIORITY, NULL );
  vuIP_SetLCD(&vtLCDdata);


	// LCD Task creates a queue to receive messages -- what it does with those messages will depend on how the task is configured (see LCDtask.c)
	// Here we set up a timer that will send messages to the LCD task.  You don't have to have this timer for the LCD task, it is just showing
	//  how to use a timer and how to send messages from that timer.

	// First, start up an I2C task and associate it with the I2C0 hardware on the ARM (there are 3 I2C devices, we need this one)
	// See vtI2C.h & vtI2C.c for more details on this task and the API to access the task
	// Initialize I2C0 for I2C0 at an I2C clock speed of 100KHz
	if (vtI2CInit(&vtI2C0,0,mainI2CMONITOR_TASK_PRIORITY,100000) != vtI2CInitSuccess) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	// start up a "conductor" task that will move messages around
	vStartConductorTask(&conductorData,mainCONDUCTOR_TASK_PRIORITY,&vtI2C0,&i2cData,&motorControl,&irData,&speedData,&powerData,&vtLCDdata,&webData);

    // Start the I2C task
    starti2cTask(&i2cData,mainI2C_TASK_PRIORITY,&vtI2C0);
    // Start the Motor Control task
    vStartMotorControlTask(&motorControl,mainMOTOR_CONTROL_TASK_PRIORITY,&i2cData,&webData,&vtLCDdata);
    // Start the Navigation task
    vStartNavigationTask(&navData,mainNAVIGATION_TASK_PRIORITY,&motorControl,&vtLCDdata,&i2cData);
    // Start the IR Control task
    vStartIRTask(&irData,mainIR_CONTROL_TASK_PRIORITY,&navData,&vtLCDdata);
    // Start the Speed Limit task
    vStartSpeedLimitTask(&speedData,mainSPEED_LIMIT_TASK_PRIORITY,&motorControl,&navData,&webData);
    // Start the web server task
    vStartWebServerTask(& webData, mainWEB_SERVER_TASK_PRIORITY, &i2cData, &vtLCDdata);

    setUiPWebServer(&webData);
    xTaskCreate( vuIP_Task, ( signed char * ) "uIP", mainBASIC_WEB_STACK_SIZE, ( void * ) NULL, mainUIP_TASK_PRIORITY, NULL );

    startTimerFori2c(&i2cData);
	//startTimerForMotor(&motorControl);
	//startTimerForNav(&navData);

    /* Create the USB task. MTJ: This routine has been modified from the original example (which is not a FreeRTOS standard demo) */
	#if USE_MTJ_USE_USB == 1
	initUSB();  // MTJ: This is my routine used to make sure we can do printf() with USB
    xTaskCreate( vUSBTask, ( signed char * ) "USB", configMINIMAL_STACK_SIZE, ( void * ) NULL, mainUSB_TASK_PRIORITY, NULL );
	#endif

	/* Start the scheduler. */
	// IMPORTANT: Once you start the scheduler, any variables on the stack from main (local variables in main) can be (will be...) written over
	//            because the stack is used by the interrupt handler
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
static unsigned long ulTicksSinceLastDisplay = 0;

	/* Called from every tick interrupt as described in the comments at the top
	of this file.

	Have enough ticks passed to make it	time to perform our health status
	check again? */
	ulTicksSinceLastDisplay++;
	if( ulTicksSinceLastDisplay >= mainCHECK_DELAY )
	{
		/* Reset the counter so these checks run again in mainCHECK_DELAY
		ticks time. */
		ulTicksSinceLastDisplay = 0;
	}
}
/*-----------------------------------------------------------*/

char *pcGetTaskStatusMessage( void )
{
	/* Not bothered about a critical section here. */
	return pcStatusMessage;
}
/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
	/* Disable peripherals power. */
	SC->PCONP = 0;

	/* Enable GPIO power. */
	SC->PCONP = PCONP_PCGPIO;

	/* Disable TPIU. */
	PINCON->PINSEL10 = 0;

	/*  Setup the peripheral bus to be the same as the PLL output (64 MHz). */
	SC->PCLKSEL0 = 0x05555555;

	/* Configure the LEDs. */
	vParTestInitialise();
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will get called if a task overflows its stack. */

	( void ) pxTask;
	( void ) pcTaskName;

	// MTJ: I have directed this to the fatal error handler
	VT_HANDLE_FATAL_ERROR(0);
	for( ;; );
}
/*-----------------------------------------------------------*/

void vConfigureTimerForRunTimeStats( void )
{
const unsigned long TCR_COUNT_RESET = 2, CTCR_CTM_TIMER = 0x00, TCR_COUNT_ENABLE = 0x01;

	/* This function configures a timer that is used as the time base when
	collecting run time statistical information - basically the percentage
	of CPU time that each task is utilising.  It is called automatically when
	the scheduler is started (assuming configGENERATE_RUN_TIME_STATS is set
	to 1). */

	/* Power up and feed the timer. */
	SC->PCONP |= 0x02UL;
	SC->PCLKSEL0 = (SC->PCLKSEL0 & (~(0x3<<2))) | (0x01 << 2);

	/* Reset Timer 0 */
	TIM0->TCR = TCR_COUNT_RESET;

	/* Just count up. */
	TIM0->CTCR = CTCR_CTM_TIMER;

	/* Prescale to a frequency that is good enough to get a decent resolution,
	but not too fast so as to overflow all the time. */
	TIM0->PR =  ( configCPU_CLOCK_HZ / 10000UL ) - 1UL;

	/* Start the counter. */
	TIM0->TCR = TCR_COUNT_ENABLE;
}
/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
	// Here we decide to go to sleep because we *know* that no other higher priority task is ready *and* we
	//   know that we are the lowest priority task (we are the idle task)
	// Important: We are just being *called* from the idle task, so we cannot run a loop or anything like that
	//   here.  We just go to sleep and then return (which presumably only happens when we wake up).
	vtITMu8(vtITMPortIdle,SCB->SCR);
	__WFI(); // go to sleep until an interrupt occurs
	// DO NOT DO THIS... It is not compatible with the debugger: __WFE(); // go into low power until some (not quite sure what...) event occurs
	vtITMu8(vtITMPortIdle,SCB->SCR+0x10);
}
