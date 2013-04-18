#include "myDefs.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "timers.h"

/* include files. */
#include "vtUtilities.h"
#include "LCDtask.h"
#include "myTimers.h"
#include "myTypes.h"

/* **************************************************************** */
// WARNING: Do not print in this file
//		-- the stack is not large enough for this task
/* **************************************************************** */

/* *********************************************************** */
// Functions for the I2C Task related timer
//
// How often the timer that sends messages to the I2C task should run
// Set the task up to run every 30 ms for the I2C timer
#define i2cWRITE_RATE_BASE	( ( portTickType ) 20 / portTICK_RATE_MS)

// Callback function that is called by the I2C Timer
// This badboy will send a message to the queue that is read by the I2C Task
void i2cTimerCallback(xTimerHandle pxTimer)
{
	if (pxTimer == NULL) {
		VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
	} else {
		// When setting up this timer, I put the pointer to the
		// i2c structure as the "timer ID" so that I could access
		// that structure here -- which I need to do to get the
		// address of the message queue to send to.
		myI2CStruct *ptr = (myI2CStruct *) pvTimerGetTimerID(pxTimer);
		// Make this non-blocking *but* be aware that if the queue is full, this routine
		// will not care, so if you care, you need to check something.
		if (sendi2cTimerMsg(ptr,i2cWRITE_RATE_BASE,0) == errQUEUE_FULL) {
			// Here is where you would do something if you wanted to handle the queue being full
			VT_HANDLE_FATAL_ERROR(I2C_Q_FULL);
		}
	}
}

void startTimerFori2c(myI2CStruct *i2cdata) {
	if (sizeof(long) != sizeof(myI2CStruct *)) {
		VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
	}
	xTimerHandle i2cTimerHandle = xTimerCreate((const signed char *)"i2c Timer",i2cWRITE_RATE_BASE,pdTRUE,(void *) i2cdata,i2cTimerCallback);
	if (i2cTimerHandle == NULL) {
		VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
	} else {
		if (xTimerStart(i2cTimerHandle,0) != pdPASS) {
			VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
		}
	}
}


/* *********************************************************** */
// Functions for the Motor Controller Task related timer
//
// how often the timer that sends messages to the motor controller task should run
// Set the task up to run every 19 ms for the motor control timer
#define motorWRITE_RATE_BASE	( ( portTickType ) 19 / portTICK_RATE_MS)

// Callback function that is called by the Motor Controller Timer
// This puppy sends a message to the queue that is read by the Motor Controller Task
void motorTimerCallback(xTimerHandle pxTimer)
{
	if (pxTimer == NULL) {
		VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
	} else {
		// When setting up this timer, I put the pointer to the
		// i2c structure as the "timer ID" so that I could access
		// that structure here -- which I need to do to get the
		// address of the message queue to send to
		motorControlStruct *ptr = (motorControlStruct *) pvTimerGetTimerID(pxTimer);
		// Make this non-blocking *but* be aware that if the queue is full, this routine
		// will not care, so if you care, you need to check something
		if (sendMotorTimerMsg(ptr,motorWRITE_RATE_BASE,0) == errQUEUE_FULL) {
			// Here is where you would do something if you wanted to handle the queue being full
			VT_HANDLE_FATAL_ERROR(I2C_Q_FULL);
		}
	}
}

void startTimerForMotor(motorControlStruct *motordata){
	if (sizeof(long) != sizeof(motorControlStruct *)) {
		VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
	}
	xTimerHandle motorTimerHandle = xTimerCreate((const signed char *)"Motor Timer",motorWRITE_RATE_BASE,pdTRUE,(void *) motordata,motorTimerCallback);
	if (motorTimerHandle == NULL) {
		VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
	} else {
		if (xTimerStart(motorTimerHandle,0) != pdPASS) {
			VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
		}
	}
}


/* *********************************************************** */
// Functions for the Navigation Task related timer
//
// How often the timer that sends messages to the Navigation task should run
// Set the task up to run every 10 ms for the Navigation task timer
#define navWRITE_RATE_BASE	( ( portTickType ) 10 / portTICK_RATE_MS)

// Callback function that is called by the Navigation Task Timer
// This puppy sends a message to the queue that is read by the Navigation Task
void navTimerCallback(xTimerHandle pxTimer)
{
	if (pxTimer == NULL) {
		VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
	} else {
		// When setting up this timer, I put the pointer to the
		// i2c structure as the "timer ID" so that I could access
		// that structure here -- which I need to do to get the
		// address of the message queue to send to
		navigationStruct *ptr = (navigationStruct *) pvTimerGetTimerID(pxTimer);
		// Make this non-blocking *but* be aware that if the queue is full, this routine
		// will not care, so if you care, you need to check something
		if (sendNavTimerMsg(ptr,navWRITE_RATE_BASE,0) == errQUEUE_FULL) {
			// Here is where you would do something if you wanted to handle the queue being full
			VT_HANDLE_FATAL_ERROR(I2C_Q_FULL);
		}
	}
}

void startTimerForNav(navigationStruct *navData){
	if (sizeof(long) != sizeof(navigationStruct *)) {
		VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
	}
	xTimerHandle navTimerHandle = xTimerCreate((const signed char *)"Nav Timer",navWRITE_RATE_BASE,pdTRUE,(void *) navData,navTimerCallback);
	if (navTimerHandle == NULL) {
		VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
	} else {
		if (xTimerStart(navTimerHandle,0) != pdPASS) {
			VT_HANDLE_FATAL_ERROR(TIMER_ERROR);
		}
	}
}

