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
#include "LCDtask.h"
#include "vtUtilities.h"
#include "irControl.h"
#include "myTypes.h"
#include "navigation.h"
/* *********************************************** */

#define INSPECT_STACK 1
#define baseStack 2
#if PRINTF_VERSION == 1
#define conSTACK_SIZE       ((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define conSTACK_SIZE       (baseStack*configMINIMAL_STACK_SIZE)
#endif

// Length of the queue to this task
#define irQLen 10
// actual data structure that is sent in a message
typedef struct __irMsg {
    uint8_t msgType;
    uint8_t length;  // Length of the message
    uint8_t buf[maxIRMsgLen+1]; // On the way in, message to be sent, on the way out, message received (if any)
} irMsg;

/* The Navigation task. */
static portTASK_FUNCTION_PROTO( vIRTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartIRTask(irControlStruct *params, unsigned portBASE_TYPE uxPriority, navigationStruct *navData, vtLCDStruct *lcd)
{
    // Create the queue that will be used to talk to this task
    if ((params->inQ = xQueueCreate(irQLen,sizeof(irMsg))) == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    /* Start the task */
    portBASE_TYPE retval;
    params->navData = navData;
    params->lcdData = lcd;
    if ((retval = xTaskCreate( vIRTask, ( signed char * ) "IR", conSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
        VT_HANDLE_FATAL_ERROR(TASK_CREATION_ERROR);
    }
}

portBASE_TYPE conductorSendIRSensorDataMsg(irControlStruct *irData, uint8_t *data, uint8_t length)
{
    if (irData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    irMsg buffer;
    buffer.length = length;
    if (buffer.length > maxIRMsgLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_IR_MSG_FORMAT);
    }
    memcpy(buffer.buf, data, length);
    buffer.msgType = irDataMsgType;
    return(xQueueSend(irData->inQ,(void *) (&buffer),portMAX_DELAY));
}

// End of Public API
/*-----------------------------------------------------------*/

// Define sensor numbers
#define BACK_LEFT_SEN   1
#define FRONT_LEFT_SEN  2
#define FRONT_SEN   3
#define FRONT_RIGHT_SEN 4
#define BACK_RIGHT_SEN  5

// Compile time constants for physical sensor positions
#define SIDE_SEN_SEPARATION 330.0f      // mm, vertical distance between side sensors "0"
#define COS_SEN_ANGLE_FS    0.8988f     // cos(sensor angle front side)
#define SEN_OFFSET_FS       80.0f       // mm, distance from edge to sensor's "0"
#define COS_SEN_ANGLE_BS    0.9659f     // cos(sensor angle back side)
#define SEN_OFFSET_BS       90.0f       // mm, distance from edge to sensor's "0"
#define SEN_OFFSET_F        50          // mm, distance from edge to sensor's "0"
#define RAD_TO_DEG          180.0f/3.1416f  // Radians to degrees conversion factor

// Maximum errors before shutting off.
#define MAX_IR_ERRORS       10

typedef struct __cur_ir_data {
    unsigned short BLS;     // Back Left Side
    unsigned short FLS;     // Front Left Side
    unsigned short F;       // Left Front
    unsigned short FRS;     // Front Right Side
    unsigned short BRS;     // Back Right Side
} cur_ir_data;

// Private routines used to unpack the message buffers.
// I do not want to access the message buffer data structures outside of these routines.
// These routines are specific to accessing our packet protocol from the task struct.

// For accessing data sent between ARM local tasks:

/**     N/A     **/
// This means no ARM tasks (excluding the conductor) are sending data to this task

// For accessing data sent between Rover PIC(s) and the ARM:

uint8_t getPcktProtoID(irMsg *irBuf){
    return irBuf->buf[0];
}

uint8_t getPcktProtoSensorNum(irMsg *irBuf){
    return irBuf->buf[1];
}

uint8_t getPcktProtoParity(irMsg *irBuf){
    return irBuf->buf[2];
}

uint8_t getPcktProtoCount(irMsg *irBuf){
    return irBuf->buf[3];
}

uint8_t getPcktProtoData1(irMsg *irBuf){
    return irBuf->buf[4];
}

uint8_t getPcktProtoData2(irMsg *irBuf){
    return irBuf->buf[5];
}

uint8_t getPcktProtoData3(irMsg *irBuf){
    return irBuf->buf[6];
}

uint8_t getPcktProtoData4(irMsg *irBuf){
    return irBuf->buf[7];
}

unsigned short getPcktProtoIRDistance(irMsg *irBuf) {
    unsigned short dist = irBuf->buf[4];
    dist = (dist << 8) | irBuf->buf[5];
    return dist;
}

// End of private routines for message buffers
/*-----------------------------------------------------------*/

// Private routines used for data manipulation, etc.
// There should be NO accessing of our packet protocol from the task struct in these routines.

int getMsgType(irMsg *irBuf)
{
    return(irBuf->msgType);
}

// End of private routines for data manipulation, etc.
/*-----------------------------------------------------------*/

// --- Private helper functions ----------------------------------------------
// Temp for MS3 output to LEDs
void updateTestCaseLED(short distance, short angle) {
    vtLEDOff(0xFF);
    // 150mm
    if (140 < distance && distance < 160) {
        vtLEDOn(0x10);
    }
    // 200mm
    else if (190 < distance && distance < 210) {
        vtLEDOn(0x20);
    }
    // 300mm
    else if (290 < distance && distance < 310) {
        vtLEDOn(0x40);
    }
    // 400mm
    else if (390 < distance && distance < 410) {
        vtLEDOn(0x80);
    }
    // -18 deg
    if (-25 < angle && angle < -10) {
        vtLEDOn(0x01);
    }
    // -2 deg
    else if (-10 < angle && angle < 5) {
        vtLEDOn(0x02);
    }
    // 12 deg
    else if (5 < angle && angle < 20) {
        vtLEDOn(0x04);
    }
    // 30 deg
    else if (20 < angle && angle < 40) {
        vtLEDOn(0x08);
    }
}

#define LEFT_SIDE   0
#define RIGHT_SIDE  1
static char LCDupdateFreq;
// Updateds the wall positions and sends to navigation task
// FS is distance from front sensor
// BS is distance from back sensor
// side is the side (left = 0, right = 1)
void updateAndSendSideWall(navigationStruct* nav, vtLCDStruct* lcd, unsigned short FS, unsigned short BS, char side) {
    // Calculate distance to wall from front of rover
    short distance = (short)(((float)FS * COS_SEN_ANGLE_FS - SEN_OFFSET_FS));
    // Calculate the angle the wall is relative to the rover.
    // -positive is clockwise, negative is counterclockwise
    signed char angle = (signed char)(atan2(((float)BS * COS_SEN_ANGLE_BS - SEN_OFFSET_BS) - distance, SIDE_SEN_SEPARATION)*RAD_TO_DEG);

    if (side == LEFT_SIDE) {
    AIUpdateLeftWall(nav, distance, angle);
        printf("Left:   FS = %d    BS = %d    dist = %d    angle = %d\n", FS,BS, distance, angle);
        if (LCDupdateFreq <= 0) {
            char buffer[20];
            if (distance < 400) {
                sprintf(buffer, "L=%3d mm @%3d deg", distance, angle);
            } else {
                sprintf(buffer, "L=INF mm @NUL deg");
            }
            sendLCDDebugString(lcd, 2, buffer, 18);
        }
    }
    else {
    AIUpdateRightWall(nav, distance, angle);
        printf("Right:  FS = %d    BS = %d    dist = %d    angle = %d\n", FS,BS, distance, angle);
        if (LCDupdateFreq <= 0) {
            char buffer[20];
            if (distance < 400) {
                sprintf(buffer, "R=%3d mm @%3d deg", distance, angle);
            } else {
                sprintf(buffer, "R=INF mm @NUL deg");
            }
            sendLCDDebugString(lcd, 3, buffer, 18);
        }
    }
}

void updateAndSendFrontWall(navigationStruct* nav, vtLCDStruct* lcd, unsigned short dist) {
    short distance = dist - SEN_OFFSET_F;
    AIUpdateFrontWall(nav,dist);
    if (LCDupdateFreq <= 0) {
        char buffer[20];
        if (distance < 400) {
            sprintf(buffer, "F=%3d mm", distance);
        } else {
            sprintf(buffer, "F=INF mm");
        }
        sendLCDDebugString(lcd, 1, buffer, 9);
    }
}
// --- End Private helper functions ------------------------------------------

// Here is where the declaration of static task pointers occurs; they will be initialized below.
static irControlStruct *param;
static navigationStruct *navData;
static vtLCDStruct *lcdData;

// Buffer for receiving messages
static irMsg msgBuffer;

// This is the actual task that is run
static portTASK_FUNCTION( vIRTask, pvParameters )
{
    // Get the parameters
    param = (irControlStruct *) pvParameters;
    // Get the other necessary tasks' task pointers like this:
    navData = param->navData;
    lcdData = param->lcdData;

    LCDupdateFreq = 4;

    // Private storage
    cur_ir_data ir_data;
    unsigned char count;
    unsigned char errorCount;

    // Private storage init
    ir_data.BLS = 0;
    ir_data.FLS = 0;
    ir_data.F   = 0;
    ir_data.FRS = 0;
    ir_data.BRS = 0;
    count = 0;
    errorCount = 0;

    // Like all good tasks, this should never exit
    for(;;)
    {
        // Wait for a message from whomever we decide will need to talk to this task
        if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
            VT_HANDLE_FATAL_ERROR(Q_RECV_ERROR);
        }
        switch(getMsgType(&msgBuffer)){
            case irDataMsgType:
            {
                // Error check message
                if (getPcktProtoCount(&msgBuffer) != count) {
                    count = getPcktProtoCount(&msgBuffer);
                    ++errorCount;
                    if (errorCount > MAX_IR_ERRORS) {
                        //VT_HANDLE_FATAL_ERROR(); // Get Matt to add new error type
                    }
                }
                if (getPcktProtoParity(&msgBuffer) != (getPcktProtoData1(&msgBuffer) ^ getPcktProtoData2(&msgBuffer))) {
                    ++errorCount;
                    if (errorCount > MAX_IR_ERRORS) {
                        //VT_HANDLE_FATAL_ERROR(); // Get Matt to add new error type
                    }
                }

                // Save reading and update coresponding wall
                switch (getPcktProtoSensorNum(&msgBuffer)) {
                    case BACK_LEFT_SEN:
                    {
                        ir_data.BLS = getPcktProtoIRDistance(&msgBuffer);
                        updateAndSendSideWall(navData, lcdData, ir_data.FLS, ir_data.BLS,LEFT_SIDE);
                        break;
                    }
                    case FRONT_LEFT_SEN:
                    {
                        --LCDupdateFreq;
                        ir_data.FLS = getPcktProtoIRDistance(&msgBuffer);
                        updateAndSendSideWall(navData, lcdData, ir_data.FLS,ir_data.BLS,LEFT_SIDE);
                        break;
                    }
                    case FRONT_SEN:
                    {
                        ir_data.F = getPcktProtoIRDistance(&msgBuffer);
                        updateAndSendFrontWall(navData, lcdData, ir_data.F);
                        break;
                    }
                    case FRONT_RIGHT_SEN:
                    {
                        ir_data.FRS = getPcktProtoIRDistance(&msgBuffer);
                        updateAndSendSideWall(navData, lcdData, ir_data.FRS,ir_data.BRS, RIGHT_SIDE);
                        if (LCDupdateFreq <= 0) LCDupdateFreq = 4;
                        break;
                    }
                    case BACK_RIGHT_SEN:
                    {
                        ir_data.BRS = getPcktProtoIRDistance(&msgBuffer);
                        updateAndSendSideWall(navData, lcdData, ir_data.FRS,ir_data.BRS, RIGHT_SIDE);
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
                break;
            }
            default:
            {
                VT_HANDLE_FATAL_ERROR(UNKNOWN_IR_MSG_TYPE);
                break;
            }
        }
    }
}
