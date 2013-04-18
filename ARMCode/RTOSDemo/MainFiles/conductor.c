#include "myDefs.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"

/* include files. */
#include "vtUtilities.h"
#include "vtI2C.h"
#include "i2c_ARM.h"
#include "myTypes.h"
#include "conductor.h"
#include "motorControl.h"
#include "irControl.h"
#include "speedLimit.h"
#include "power.h"
#include "LCDtask.h"
#include "webServer.h"

/* *********************************************** */
// Definitions and data structures that are private to this file

// I have set this to a large stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the i2c operations	-- almost certainly too large, see LCDTask.c for details on how to check the size
#define INSPECT_STACK 1
#define baseStack 2
#if PRINTF_VERSION == 1
#define conSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define conSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif
// end of defs
/* *********************************************** */

/* The Conductor task. */
static portTASK_FUNCTION_PROTO( vConductorUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartConductorTask(vtConductorStruct *params, unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, myI2CStruct * myi2c, motorControlStruct *motorControl, irControlStruct *irData, speedLimitControlStruct *speedData, powerStruct *powerData, vtLCDStruct *lcdData, webServerStruct *webData)
{
	/* Start the task */
	portBASE_TYPE retval;
	params->dev = i2c;
	params->i2cData = myi2c;
	params->motorControl = motorControl;
	params->irData = irData;
	params->speedData = speedData;
	params->powerData = powerData;
	params->lcdData = lcdData;
	params->webData = webData;
	if ((retval = xTaskCreate( vConductorUpdateTask, ( signed char * ) "Conductor", conSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

// End of Public API
/*-----------------------------------------------------------*/

// Private routines used to unpack the message buffers.
// I do not want to access the message buffer data structures outside of these routines.

// THESE ARE SPECIFIC TO THE CONDUCTOR TASK!
// DO NOT CHANGE!

uint8_t vtGetI2CMsgType(uint8_t *buffer) {
	return buffer[0];		// Don't change me! This is NOT referring to the packet protocol we established!
							// Rather, this is referring to the predefined vtI2C packet structure by Jones.
}

uint8_t vtGetI2CMsgCount(uint8_t *buffer) {
	return buffer[1];		// Don't change me! This is NOT referring to the packet protocol we established!
							// Rather, this is referring to the predefined vtI2C packet structure by Jones.
}

// THESE ARE SPECIFIC TO THE CONDUCTOR TASK!
// DO NOT CHANGE!

// End of private routines for message buffers
/*-----------------------------------------------------------*/

static vtConductorStruct *param;
static vtI2CStruct *devPtr;
static myI2CStruct *i2cData;
static motorControlStruct *motorControl;
static irControlStruct *irData;
static speedLimitControlStruct *speedData;
static powerStruct *powerData;
static vtLCDStruct *lcdData;
static webServerStruct *webData;

static uint8_t colorSensorMsgCount, encodersMsgCount, IRMsgCount, ADCMsgCount, powerMsgCount, recvMsgType, rxLen, status, Buffer[vtI2CMLen];

// This is the actual task that is run
static portTASK_FUNCTION( vConductorUpdateTask, pvParameters )
{
	// Get the parameters
	param = (vtConductorStruct *) pvParameters;
	// Get the I2C device pointer
	devPtr = param->dev;
	// Get the I2C task pointer
	i2cData = param->i2cData;
	// Get the Motor Control task pointer
	motorControl = param->motorControl;
	// Get the IR Control task pointer
	irData = param->irData;
	// Get the Speed Limit task pointer
	speedData = param->speedData;
	//Get the Power task pointer
	powerData = param->powerData;
	// Get the LCD pointer information
	lcdData = param->lcdData;
	// Get the web server pointer information
	webData = param->webData;

	// Message counts
	colorSensorMsgCount = 0, encodersMsgCount = 0, IRMsgCount = 0, ADCMsgCount = 0, powerMsgCount = 0, recvMsgType = 0;		// The value 0 is a temporary initial value for recvMsgType, rxLen; it's overwritten when vtI2CDeQ occurs.

	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from an I2C operation
		if (vtI2CDeQ(devPtr,vtI2CMLen,Buffer,&rxLen,&recvMsgType,&status) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Decide where to send the message
		// This isn't a state machine, it is just acting as a router for messages
		switch(recvMsgType) {
			case vtI2CReadMsgType: {
				// If it is a read, send it to the appropriate task
				switch(vtGetI2CMsgType(Buffer)){

					/*---EMPTY MESSAGES---*/

					case COLOR_SENSOR_EMPTY_MESSAGE: {
						notifyRequestRecvd(i2cData,portMAX_DELAY);
						colorSensorMsgCount++;
						if(colorSensorMsgCount != vtGetI2CMsgCount(Buffer)){
							//Send Web Server an error with vtGetI2CMsgCount(Buffer) - colorSensorMsgCount
							colorSensorMsgCount = vtGetI2CMsgCount(Buffer);
						}
						break;
					}
					case ENCODERS_EMPTY_MESSAGE: {
						notifyRequestRecvd(i2cData,portMAX_DELAY);
						encodersMsgCount++;
						if(encodersMsgCount != vtGetI2CMsgCount(Buffer)){
							//Send Web Server an error with vtGetI2CMsgCount(Buffer) - encodersMsgCount
							encodersMsgCount = vtGetI2CMsgCount(Buffer);
						}
						break;
					}
					case IR_EMPTY_MESSAGE: {
						notifyRequestRecvd(i2cData,portMAX_DELAY);
						IRMsgCount++;
						if(IRMsgCount != vtGetI2CMsgCount(Buffer)){
							//Send Web Server an error with vtGetI2CMsgCount(Buffer) - IRMsgCount
							IRMsgCount = vtGetI2CMsgCount(Buffer);
						}
						break;
					}
					case POWER_EMPTY_MESSAGE: {
						notifyRequestRecvd(i2cData,portMAX_DELAY);
						powerMsgCount++;
						if(powerMsgCount != vtGetI2CMsgCount(Buffer)){
							//Send Web Server an error with vtGetI2CMsgCount(Buffer) - powerMsgCount
							powerMsgCount = vtGetI2CMsgCount(Buffer);
						}
						break;
					}
					case GENERIC_EMPTY_MESSAGE: {
						break;
					}

					/*---DATA MESSAGES---*/

					case COLOR_SENSOR_MESSAGE: {
						notifyRequestRecvd(i2cData,portMAX_DELAY);
						conductorSendColorSensorDataMsg(speedData, (Buffer), 8);	// Or (Buffer + 2) ?
						colorSensorMsgCount++;
						if(colorSensorMsgCount != vtGetI2CMsgCount(Buffer)){
							//Send Web Server an error with vtGetI2CMsgCount(Buffer) - colorSensorMsgCount
							colorSensorMsgCount = vtGetI2CMsgCount(Buffer);
						}
						break;
					}
					case ENCODERS_MESSAGE: {
						notifyRequestRecvd(i2cData,portMAX_DELAY);
						conductorSendMotorEncoderDataMsg(motorControl, (Buffer), 8);	// Or (Buffer + 2) ?
						encodersMsgCount++;
						if(encodersMsgCount != vtGetI2CMsgCount(Buffer)){
							//Send Web Server an error with vtGetI2CMsgCount(Buffer) - encodersMsgCount
							encodersMsgCount = vtGetI2CMsgCount(Buffer);
						}
						break;
					}
					case IR_MESSAGE: {
						notifyRequestRecvd(i2cData,portMAX_DELAY);
						conductorSendIRSensorDataMsg(irData, (Buffer), 8);		// Still figuring out if this should be 2, 4, or 8 but I'm pretty sure this should be 8
						IRMsgCount++;
						if(IRMsgCount != vtGetI2CMsgCount(Buffer)){
							//Send Web Server an error with vtGetI2CMsgCount(Buffer) - IRMsgCount
							IRMsgCount = vtGetI2CMsgCount(Buffer);
						}
						break;
					}
					case POWER_MESSAGE: {
						notifyRequestRecvd(i2cData,portMAX_DELAY);
						conductorSendPowerDataMsg(powerData, (Buffer), 4);	// Or (Buffer + 2) ?
						powerMsgCount++;
						if(powerMsgCount != vtGetI2CMsgCount(Buffer)){
							//Send Web Server an error with vtGetI2CMsgCount(Buffer) - powerMsgCount
							powerMsgCount = vtGetI2CMsgCount(Buffer);
						}
						break;
					}
				}	// End switch statement
				break;
			}	// End vtI2CReadMsgType case
			case vtI2CMotorMsgType: {
				// If it is a I2C motor message, just recognize that this is an ack from the slave
				// Nothing else to do here
				break;
			}
			default: {
				VT_HANDLE_FATAL_ERROR(recvMsgType);
				break;
			}
		}	// End switch statement
	}	// End for loop
}

