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

#define baseStack 3
#if PRINTF_VERSION == 1
#define i2cSTACK_SIZE       ((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define i2cSTACK_SIZE       (baseStack*configMINIMAL_STACK_SIZE)
#endif

/* *********************************************** */
// definitions and data structures that are private to this file
// Length of the queue to this task
#define vti2cQLen 10
// actual data structure that is sent in a message
typedef struct __myi2cMsg {
    uint8_t msgType;
    uint8_t length;  // Length of the message to be printed
    uint8_t buf[vti2cMaxLen+1]; // On the way in, message to be sent, on the way out, message received (if any)
} myi2cMsg;

// end of defs
/* *********************************************** */

/* The i2c_ARM task. */
static portTASK_FUNCTION_PROTO( vi2cUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void starti2cTask(myI2CStruct *params, unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c)
{
    // Create the queue that will be used to talk to this task
    if ((params->inQ = xQueueCreate(vti2cQLen,sizeof(myi2cMsg))) == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    /* Start the task */
    portBASE_TYPE retval;
    params->dev = i2c;
    if ((retval = xTaskCreate( vi2cUpdateTask, ( signed char * ) "i2c", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
        VT_HANDLE_FATAL_ERROR(retval);
    }
}

				  #include "json.h"
portBASE_TYPE sendi2cTimerMsg(myI2CStruct *i2cData,portTickType ticksElapsed,portTickType ticksToBlock)
{
    if (i2cData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    myi2cMsg buffer;
    buffer.length = sizeof(ticksElapsed);
    if (buffer.length > vti2cMaxLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(buffer.length);
    }
    memcpy(buffer.buf,(char *)&ticksElapsed,sizeof(ticksElapsed));
    buffer.msgType = i2cTimerMsgType;
    return(xQueueSend(i2cData->inQ,(void *) (&buffer),ticksToBlock));
 //    if (i2cData == NULL) {
 //        VT_HANDLE_FATAL_ERROR(0);
 //    }
 //    myi2cMsg buffer;
 //    buffer.length = 10;//sizeof(ticksElapsed);
 //    if (buffer.length > vti2cMaxLen) {
 //        // no room for this message
 //        VT_HANDLE_FATAL_ERROR(buffer.length);
 //    }
 //    //memcpy(buffer.buf,(char *)&ticksElapsed,sizeof(ticksElapsed));
	// static int count = 0;
	// ++count;

	// static int lastTimer = 0;

	// if (count % 10 == 0)  {
	// ++lastTimer;
	// static char temp[100];
	// JsonNew(temp);
	// JsonSetString(temp, "to", "@motor");
	// JsonSetString(temp, "from", "@arm");
	// JsonSetString(temp, "sub", "set");
	// JsonSetValue(temp, "key", -1);
	// JsonSetValue(temp, "msg", lastTimer);
	// strcpy(buffer.buf, temp);
	// 	//sprintf(buffer.buf, "to:@motor\nfrom:@arm\nsub:set\nkey:-1\nmsg:%d\n", lastTimer);
	// buffer.length = strlen(buffer.buf)+1;
 //    buffer.msgType = i2cTimerMsgType;
	// } else {
	// 	   	sprintf(buffer.buf, "to:@motor\nfrom:@arm\nsub:set\nkey:-1\nmsg:%d\n", lastTimer);
	// buffer.length = strlen(buffer.buf)+1;
 //    buffer.msgType = i2cTimerMsgType;
	// }
 //    return(xQueueSend(i2cData->inQ,(void *) (&buffer),ticksToBlock));
}

portBASE_TYPE sendi2cMotorMsg(myI2CStruct *i2cData, uint8_t leftValue, uint8_t rightValue, portTickType ticksToBlock)
{
    if (i2cData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    myi2cMsg buffer;
    buffer.length = 8;
    if (buffer.length > vti2cMaxLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_I2C_MSG_FORMAT);
    }
    buffer.buf[0] = 0xBB;       //i2c id
    buffer.buf[1] = 0x00;       //class id
    buffer.buf[2] = 0x00;       //parity
    buffer.buf[3] = 0x00;       //count
    buffer.buf[4] = leftValue;  //data[0]
    buffer.buf[5] = rightValue; //data[1]
    buffer.buf[6] = 0x00;       //data[2]
    buffer.buf[7] = 0x00;       //data[3]
    buffer.msgType = vtI2CMotorMsgType;
    return(xQueueSend(i2cData->inQ,(void *) (&buffer),ticksToBlock));
}

portBASE_TYPE sendi2cWebServerMsg(myI2CStruct *i2cData, uint8_t valueOne, uint8_t valueTwo, portTickType ticksToBlock)
{
    if (i2cData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    myi2cMsg buffer;
    buffer.length = 8;
    if (buffer.length > vti2cMaxLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_I2C_MSG_FORMAT);
    }
    buffer.buf[0] = 0xBB;       //i2c id
    buffer.buf[1] = 0x05;       //class id
    buffer.buf[2] = 0x04;       //parity
    buffer.buf[3] = 0x03;       //count
    buffer.buf[4] = valueOne;  //data[0]
    buffer.buf[5] = valueTwo; //data[1]
    buffer.buf[6] = 0xFF;       //data[2]
    buffer.buf[7] = 0xFF;       //data[3]
    buffer.msgType = vtI2CWebServerMsgType;
    return(xQueueSend(i2cData->inQ,(void *) (&buffer),ticksToBlock));
}

portBASE_TYPE notifyRequestRecvd(myI2CStruct *i2cData,portTickType ticksToBlock)
{
    if (i2cData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    myi2cMsg buffer;
    buffer.length = 0;
    if (buffer.length > vti2cMaxLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_I2C_MSG_FORMAT);
    }
    buffer.msgType = notifyRqstRecvdMsgType;
    return(xQueueSend(i2cData->inQ,(void *) (&buffer),ticksToBlock));
}

// End of Public API
/*-----------------------------------------------------------*/

uint8_t requestSent = 0;

// Private routines used for data manipulation, etc.
// There should be NO accessing of our packet protocol from the task struct in these routines.

void notifyRequestSent(){
    if(requestSent == 1){
        // Send I2C Error Message to Web Server
    }
    requestSent = 1;
}

int getMsgType(myi2cMsg *buffer)
{
    return(buffer->msgType);
}

// End of private routines for data manipulation, etc.
/*-----------------------------------------------------------*/

static myI2CStruct *param;
static vtI2CStruct *devPtr;

// Buffer for receiving messages
static myi2cMsg msgBuffer;

// This is the actual task that is run
static portTASK_FUNCTION( vi2cUpdateTask, pvParameters )
{
    // Get the parameters
    param = (myI2CStruct *) pvParameters;
    // Get the I2C device pointer
    devPtr = param->dev;

    // Like all good tasks, this should never exit
    for(;;)
    {
        // Wait for a message from either a timer or from an I2C operation
        if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
            VT_HANDLE_FATAL_ERROR(Q_RECV_ERROR);
        }
        switch(getMsgType(&msgBuffer)) {
            case i2cTimerMsgType: {
                // Poll local 2680 for data
                notifyRequestSent();
                if (vtI2CEnQ(devPtr,vtI2CReadMsgType,SLAVE_ADDR,0,0,I2C_MSG_SIZE) != pdTRUE) {
                    VT_HANDLE_FATAL_ERROR(VT_I2C_Q_FULL);
                }
            }
            case vtI2CMotorMsgType: {
                // Send motor command to local 2680

               //if (msgBuffer.buf[0] == 0xBB)
                 if (vtI2CEnQ(devPtr,vtI2CMotorMsgType,SLAVE_ADDR,msgBuffer.length,msgBuffer.buf,0) != pdTRUE){
                     VT_HANDLE_FATAL_ERROR(VT_I2C_Q_FULL);
                  }
				break;
            }
            case vtI2CWebServerMsgType: {
                // Send web server to local 2680

               //if (msgBuffer.buf[0] == 0xBB)
                 if (vtI2CEnQ(devPtr,vtI2CWebServerMsgType,SLAVE_ADDR,msgBuffer.length,msgBuffer.buf,0) != pdTRUE){
                     VT_HANDLE_FATAL_ERROR(VT_I2C_Q_FULL);
                  }
                break;
            }
            case notifyRqstRecvdMsgType: {
                if(requestSent == 0){
                    // Send I2C Error Message to Web Server
                }
                requestSent = 0;
                break;
            }
            default: {
                VT_HANDLE_FATAL_ERROR(UNKNOWN_I2C_MSG_TYPE);
                break;
            }
        }
    }
}

