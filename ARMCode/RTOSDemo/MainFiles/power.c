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
#include "myTypes.h"
#include "power.h"
/* *********************************************** */

#define INSPECT_STACK 1
#define baseStack 2
#if PRINTF_VERSION == 1
#define conSTACK_SIZE       ((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define conSTACK_SIZE       (baseStack*configMINIMAL_STACK_SIZE)
#endif

// Length of the queue to this task
#define powerQLen 10
// actual data structure that is sent in a message
typedef struct __powerMsg {
    uint8_t msgType;
    uint8_t length;  // Length of the message
    uint8_t buf[maxPowerMsgLen+1]; // On the way in, message to be sent, on the way out, message received (if any)
} powerMsg;

/* The Navigation task. */
static portTASK_FUNCTION_PROTO( vPowerTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API

void vStartPowerTask(powerStruct *params, unsigned portBASE_TYPE uxPriority)
{
    // Create the queue that will be used to talk to this task
    if ((params->inQ = xQueueCreate(powerQLen,sizeof(powerMsg))) == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    /* Start the task */
    portBASE_TYPE retval;
    if ((retval = xTaskCreate( vPowerTask, ( signed char * ) "Power", conSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
        VT_HANDLE_FATAL_ERROR(TASK_CREATION_ERROR);
    }
}

portBASE_TYPE conductorSendPowerDataMsg(powerStruct *powerData, uint8_t *data, uint8_t length)
{
    if (powerData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    powerMsg buffer;
    buffer.length = length;
    if (buffer.length > maxPowerMsgLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_POWER_MSG_FORMAT);
    }
    memcpy(buffer.buf,data,length);
    buffer.msgType = powerDataMsgType;
    return(xQueueSend(powerData->inQ,(void *) (&buffer),portMAX_DELAY));
}

// End of Public API
/*-----------------------------------------------------------*/

// Private routines used to unpack the message buffers.
// I do not want to access the message buffer data structures outside of these routines.
// These routines are specific to accessing our packet protocol from the task struct.

// For accessing data sent between ARM local tasks:

/**     N/A     **/
// This means no ARM tasks (excluding the conductor) are sending data to this task

// For accessing data sent between Rover PIC(s) and the ARM:

uint8_t getPcktProtoID(powerMsg *powerBuf){
    return powerBuf->buf[0];
}

uint8_t getPcktProtoSensorNum(powerMsg *powerBuf){
    return powerBuf->buf[1];
}

uint8_t getPcktProtoParity(powerMsg *powerBuf){
    return powerBuf->buf[2];
}

uint8_t getPcktProtoCount(powerMsg *powerBuf){
    return powerBuf->buf[3];
}

uint8_t getPcktProtoData1(powerMsg *powerBuf){
    return powerBuf->buf[4];
}

uint8_t getPcktProtoData2(powerMsg *powerBuf){
    return powerBuf->buf[5];
}

uint8_t getPcktProtoData3(powerMsg *powerBuf){
    return powerBuf->buf[6];
}

uint8_t getPcktProtoData4(powerMsg *powerBuf){
    return powerBuf->buf[7];
}

// End of private routines for message buffers
/*-----------------------------------------------------------*/

// Private routines used for data manipulation, etc.
// There should be NO accessing of our packet protocol from the task struct in these routines.

int getMsgType(powerMsg *powerBuf)
{
    return(powerBuf->msgType);
}

// End of private routines for data manipulation, etc.
/*-----------------------------------------------------------*/

static powerStruct *param;

// Buffer for receiving messages
static powerMsg msgBuffer;

// This is the actual task that is run
static portTASK_FUNCTION( vPowerTask, pvParameters )
{
    // Get the parameters
    param = (powerStruct *) pvParameters;

    // Like all good tasks, this should never exit
    for(;;)
    {
        // Wait for a message from whomever we decide will need to talk to this task
        if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
            VT_HANDLE_FATAL_ERROR(Q_RECV_ERROR);
        }
        switch(getMsgType(&msgBuffer)){
            case powerDataMsgType:
            {
                break;
            }
            default:
            {
                VT_HANDLE_FATAL_ERROR(UNKNOWN_POWER_MSG_TYPE);
                break;
            }
        }
    }
}
