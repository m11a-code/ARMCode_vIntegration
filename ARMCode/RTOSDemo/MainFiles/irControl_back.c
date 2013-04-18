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
void vStartIRTask(irControlStruct *params, unsigned portBASE_TYPE uxPriority, navigationStruct *navData,  vtLCDStruct *lcdData)
{
    // Create the queue that will be used to talk to this task
    if ((params->inQ = xQueueCreate(irQLen,sizeof(irMsg))) == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    /* Start the task */
    portBASE_TYPE retval;
    params->navData = navData;
    params->lcdData = lcdData;
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

// Compile time constants for physical sensor positions
#define SIDE_SEN_SEPARATION	81		// mm, vertical distance between side sensors "0"
#define COS_SEN_ANGLE_FS	1.0f	// cos(sensor angle front side)
#define SEN_OFFSET_FS		10.0f	// mm, distance from edge to sensor's "0"
#define COS_SEN_ANGLE_BS	1.0f	// cos(sensor angle back side)
#define SEN_OFFSET_BS		10.0f	// mm, distance from edge to sensor's "0"
#define COS_SEN_ANGLE_F		0.966f	// cos(sensor angle front)
#define SEN_OFFSET_F		0.0f	// mm, distance from edge to sensor's "0"
#define RAD_TO_DEG			180.0f/3.1416	// Radians to degrees conversion factor

// Maximum errors before shutting off.
#define MAX_IR_ERRORS		10

typedef struct __cur_ir_data {
	unsigned short BLS;		// Back Left Side
	unsigned short FLS;		// Front Left Side
	unsigned short LF;		// Left Front
	unsigned short RF;		// Right Front
	unsigned short FRS;		// Front Right Side
	unsigned short BRS;		// Back Right Side
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

#define LEFT_SIDE	0
#define RIGHT_SIDE	1
// Updateds the wall positions and sends to navigation task
// FS is distance from front sensor
// BS is distance from back sensor
// side is the side (left = 0, right = 1)
void updateAndSendSideWall(navigationStruct* nav, unsigned short FS, unsigned short BS, char side) {
	// Calculate distance to wall from front of rover
	short distance = (short)(((float)FS * COS_SEN_ANGLE_FS - SEN_OFFSET_FS));
	// Calculate the angle the wall is relative to the rover.
	// -positive is clockwise, negative is counterclockwise
	short angle = (short)(atan2(((float)BS * COS_SEN_ANGLE_BS - SEN_OFFSET_BS) - distance, SIDE_SEN_SEPARATION)*RAD_TO_DEG);

	if (side == LEFT_SIDE) {	
		updateTestCaseLED(distance, angle);
//		char dbgmsg[10];
//		dbgmsg[0]='x';
//		dbgmsg[1]='=';
//		dbgmsg[2]=distance/100;
//		dbgmsg[3]=(distance/10)%10;
//		dbgmsg[4]=distance%10;
//		dbgmsg[5]=',';
//		dbgmsg[6]=angle/10;
//		dbgmsg[7]=angle%10; 
		printf("FS = %d    BS = %d    dist = %d    angle = %d\n", FS,BS, distance, angle);
	}
}

void updateAndSendFrontWall() {
	// Implemented in MS 4. Not needed for midterm demonstration
}
// --- End Private helper functions ------------------------------------------

// Here is where the declaration of static task pointers occurs; they will be initialized below.
static irControlStruct *param;
static navigationStruct *navData;
static  vtLCDStruct *lcdData;

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
	// Private storage
	cur_ir_data ir_data;
	unsigned char count;
	unsigned char errorCount;
	
	// Private storage init
	ir_data.BLS = 0;
	ir_data.FLS = 0;
	ir_data.LF  = 0;
	ir_data.RF  = 0;
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
					case 1: // Back Left Side
					{
            vtLEDOff(0xFF);
             vtLEDOn(1);
						ir_data.BLS = getPcktProtoIRDistance(&msgBuffer);
						updateAndSendSideWall(navData, ir_data.FLS, ir_data.BLS,LEFT_SIDE);
						break;
					}
					case 2: // Front Left Side
					{
            vtLEDOff(0xFF);
             vtLEDOn(2);
						ir_data.FLS = getPcktProtoIRDistance(&msgBuffer);
						updateAndSendSideWall(navData, ir_data.FLS,ir_data.BLS,LEFT_SIDE);
						break;
					}
					case 3: // Left Front
					{
            vtLEDOff(0xFF);
             vtLEDOn(3);
						ir_data.LF = getPcktProtoIRDistance(&msgBuffer);
						updateAndSendFrontWall();
						break;
					}
					case 4: // Right Front
					{
            vtLEDOff(0xFF);
             vtLEDOn(4);
						ir_data.RF = getPcktProtoIRDistance(&msgBuffer);
						updateAndSendFrontWall();
						break;
					}
					case 5: // Front Right Side
					{
            vtLEDOff(0xFF);
             vtLEDOn(5);
						ir_data.FRS = getPcktProtoIRDistance(&msgBuffer);
						updateAndSendSideWall(navData, ir_data.FRS,ir_data.BRS, RIGHT_SIDE);
						break;
					}
					case 6: // Back Right Side
					{
            vtLEDOff(0xFF);
             vtLEDOn(6);
						ir_data.BRS = getPcktProtoIRDistance(&msgBuffer);
						updateAndSendSideWall(navData, ir_data.FRS,ir_data.BRS, RIGHT_SIDE);
            AIUpdateDistances(navData, ir_data.BLS, ir_data.FLS, ir_data.LF, ir_data.BRS, ir_data.FRS, ir_data.RF);

						break;
					}
					default:
					{
            vtLEDOff(0xFF);
             vtLEDOn(0xFF);
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
