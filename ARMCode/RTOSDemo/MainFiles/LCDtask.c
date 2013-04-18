#include "myDefs.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* include files. */
#include "GLCD.h"
#include "vtUtilities.h"
#include "LCDtask.h"
#include "string.h"
#include "myTypes.h"

// I have set this to a larger stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the LCD operations
// I actually monitor the stack size in the code to check to make sure I'm not too close to overflowing the stack
//   This monitoring takes place if INPSECT_STACK is defined (search this file for INSPECT_STACK to see the code for this)
#define INSPECT_STACK 1
#define baseStack 3
#if PRINTF_VERSION == 1
#define lcdSTACK_SIZE       ((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define lcdSTACK_SIZE       (baseStack*configMINIMAL_STACK_SIZE)
#endif

// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtLCDQLen 10

/* definition for the LCD task. */
static portTASK_FUNCTION_PROTO( vLCDUpdateTask, pvParameters );

/*-----------------------------------------------------------*/

void vStartLCDTask(vtLCDStruct *ptr, unsigned portBASE_TYPE uxPriority)
{
    if (ptr == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }

    // Create the queue that will be used to talk to this task
    if ((ptr->inQ = xQueueCreate(vtLCDQLen,sizeof(vtLCDMsg))) == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    /* Start the task */
    portBASE_TYPE retval;
    if ((retval = xTaskCreate( vLCDUpdateTask, ( signed char * ) "LCD", lcdSTACK_SIZE, (void*)ptr, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
        VT_HANDLE_FATAL_ERROR(retval);
    }
}

// Display Debug String
portBASE_TYPE sendLCDDebugString(vtLCDStruct *lcdData, char line, char* msg, char len)
{
    vtLCDMsg buffer;
    buffer.length = len+1;
    if (buffer.length > vtLCDMaxLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_LCD_MSG_FORMAT);
    }
    buffer.buf[0] = line;
    int i;
    for (i=0; i<len; ++i) {
        buffer.buf[i+1] = msg[i];
    }
    buffer.msgType = sendLCDDebugMsg;
    return(xQueueSend(lcdData->inQ,(void *) (&buffer),portMAX_DELAY));
}

// Display the current speed, 0xFF is the maximum, on the LCD screen
portBASE_TYPE sendLCDCurrentSpeed(vtLCDStruct *lcdData, uint8_t speed)
{
    if (lcdData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    vtLCDMsg buffer;
    buffer.length = 1;
    if (buffer.length > vtLCDMaxLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_LCD_MSG_FORMAT);
    }
    buffer.buf[0] = speed;
    buffer.msgType = sendLCDCurrentSpeedMsgType;
    return(xQueueSend(lcdData->inQ,(void *) (&buffer),portMAX_DELAY));
}

// Display the current speed limit zone on the LCD screen
// uint8_t "zone" value corresponds to current speed limit zone
portBASE_TYPE SendLCDSpeedLimitZone(vtLCDStruct *lcdData, uint8_t zone)
{
    if (lcdData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    vtLCDMsg buffer;
    buffer.length = 1;
    if (buffer.length > vtLCDMaxLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_LCD_MSG_FORMAT);
    }
    buffer.buf[0] = zone;
    buffer.msgType = sendLCDSpeedLimitZoneMsgType;
    return(xQueueSend(lcdData->inQ,(void *) (&buffer),portMAX_DELAY));
}

// Display whether or not the finish line has been crossed yet on the LCD screen
//***finish value is boolean!!!!
portBASE_TYPE SendLCDFinishLine(vtLCDStruct *lcdData, uint8_t finish)
{
    if (lcdData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    vtLCDMsg buffer;
    buffer.length = 1;
    if (buffer.length > vtLCDMaxLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_LCD_MSG_FORMAT);
    }
    buffer.buf[0] = finish;
    buffer.msgType = sendLCDFinishLineMsgType;
    return(xQueueSend(lcdData->inQ,(void *) (&buffer),portMAX_DELAY));
}

// Display the current power requirements on the LCD screen
portBASE_TYPE SendLCDPower(vtLCDStruct *lcdData, uint8_t watts)
{
    if (lcdData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    vtLCDMsg buffer;
    buffer.length = 1;
    if (buffer.length > vtLCDMaxLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_LCD_MSG_FORMAT);
    }
    buffer.buf[0] = watts;
    buffer.msgType = sendLCDPowerMsgType;
    return(xQueueSend(lcdData->inQ,(void *) (&buffer),portMAX_DELAY));
}

// Display the fastest time the rover could complete the course on the LCD screen
//with the data gathered.
portBASE_TYPE SendLCDFastestTime(vtLCDStruct *lcdData, uint8_t fastMin, uint8_t fastSec)
{
    if (lcdData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    vtLCDMsg buffer;
    buffer.length = 2;
    if (buffer.length > vtLCDMaxLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_LCD_MSG_FORMAT);
    }
    buffer.buf[0] = fastMin;
    buffer.buf[1] = fastSec;
    buffer.msgType = sendLCDFastestTimeMsgType;
    return(xQueueSend(lcdData->inQ,(void *) (&buffer),portMAX_DELAY));
}

// Display whether or not a speed limit violation is occurring on the LCD screen
//***speedViolation value is boolean!!!!
portBASE_TYPE SendLCDSpeedViolation(vtLCDStruct *lcdData, uint8_t speedViolation)
{
    if (lcdData == NULL) {
        VT_HANDLE_FATAL_ERROR(0);
    }
    vtLCDMsg buffer;
    buffer.length = 1;
    if (buffer.length > vtLCDMaxLen) {
        // no room for this message
        VT_HANDLE_FATAL_ERROR(INCORRECT_LCD_MSG_FORMAT);
    }
    buffer.buf[0] = speedViolation;
    buffer.msgType = sendLCDSpeedViolationMsgType;
    return(xQueueSend(lcdData->inQ,(void *) (&buffer),portMAX_DELAY));
}

// End of Public API
/*-----------------------------------------------------------*/

// Private routines used to unpack the message buffers.
// I do not want to access the message buffer data structures outside of these routines.
// These routines are specific to accessing our packet protocol from the task struct.

// For accessing data sent between ARM local tasks:

// When sendLCDCurrentSpeedMsgType type, speed is in buf[0]
uint8_t getCurrentSpeedInfo(vtLCDMsg *lcdBuf){
    return lcdBuf->buf[0];
}

// When sendLCDSpeedLimitZoneMsgType type, zone is in buf[0]
uint8_t getSpeedLimitZoneInfo(vtLCDMsg *lcdBuf){
    return lcdBuf->buf[0];
}

// When sendLCDFinishLineMsgType type, finish is in buf[0]
uint8_t getFinishLineInfo(vtLCDMsg *lcdBuf){
    return lcdBuf->buf[0];
}

// When sendLCDPowerMsgType type, watts is in buf[0]
uint8_t getPowerInfo(vtLCDMsg *lcdBuf){
    return lcdBuf->buf[0];
}

// When sendLCDFastestTimeMsgType type, fastMin is in buf[0]
uint8_t getFastestTimeMinutesInfo(vtLCDMsg *lcdBuf){
    return lcdBuf->buf[0];
}

// When sendLCDFastestTimeMsgType type, fastSec is in buf[1]
uint8_t getFastestTimeSecondsInfo(vtLCDMsg *lcdBuf){
    return lcdBuf->buf[1];
}

// When sendLCDSpeedViolationMsgType type, speedViolation is in buf[0]
uint8_t getSpeedViolationInfo(vtLCDMsg *lcdBuf){
    return lcdBuf->buf[0];
}

// For accessing data sent between Rover PIC(s) and the ARM:

/**     N/A     **/
// This means no data goes straight from Rover PIC(s) to the web server

// End of private routines for message buffers
/*-----------------------------------------------------------*/

// Private routines used for data manipulation, etc.
// There should be NO accessing of our packet protocol from the task struct in these routines.

int getMsgType(vtLCDMsg *lcdBuf)
{
    return(lcdBuf->msgType);
}

/*

    // Example of int to string data conversion; was useful for ADC task for milestone 1 and 2ish.
        // Figured I would leave this in for now until later one when it may not be necessary as an example anymore.

    void adcIntToString(int value, unsigned char *returnVal){
        returnVal[0] = (value/0xCC) + 48;   //Ones Place
        returnVal[1] = '.';
        returnVal[2] = (((10*value)/0xCC)%10) + 48; //Tenths place
        returnVal[3] = ((((100*value)/0xCC)%100)%10) + 48; //Hundredths place
        returnVal[4] = 0;
    }

    // Another example of int to string data conversion; was useful for ADC task for milestone 1 and 2ish.
        // Figured I would leave this in for now until later one when it may not be necessary as an example anymore.

    void errorCountIntToString(int value, unsigned char *returnVal){
        int index = 0;
        int place;
        int zeroSoFar = 0;
        for(place = 1000000; place > 0; place = place/10){
            returnVal[index] = ((value/place)%10) + 48;
            if((returnVal[index] == 48) && (zeroSoFar == 0)){
                returnVal[index] = ' ';
            }else{
                zeroSoFar = 1;
            }
            index++;
        }
        returnVal[8] = 0;
    }

*/

// End of private routines for data manipulation, etc.
/*-----------------------------------------------------------*/

static vtLCDStruct *lcdPtr;

// Buffer for receiving messages
static vtLCDMsg msgBuffer;

static unsigned short screenColor, tscr;

// This is the actual task that is run
static portTASK_FUNCTION( vLCDUpdateTask, pvParameters )
{
    lcdPtr = (vtLCDStruct *) pvParameters;

    /* Initialize the LCD and set the initial colors */
    GLCD_Init();
    tscr = Black; // may be reset in the LCDMsgTypeTimer code below
    screenColor = White; // may be reset in the LCDMsgTypeTimer code below
    GLCD_SetTextColor(tscr);
    GLCD_SetBackColor(screenColor);
    GLCD_Clear(screenColor);

    //Set up constant text fields
    //GLCD_DisplayString(LINE,0,1, (unsigned char*) "^_^");

    // This task should never exit
    for(;;)
    {
        // Wait for a message
        if (xQueueReceive(lcdPtr->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
            VT_HANDLE_FATAL_ERROR(UNKNOWN_LCD_MSG_TYPE);
        }
        switch(getMsgType(&msgBuffer)){
            case sendLCDCurrentSpeedMsgType: {
                char string[4];
                sprintf(string, "%3d", msgBuffer.buf[0]);
                GLCD_DisplayString(1,0,1, (unsigned char*) string);
                //GLCD_DisplayString(LINE,0,1, (unsigned char*) string);
                //GLCD_DisplayString(1,0,1,(unsigned char *)msgBuffer.buf);
                break;
            }
            case sendLCDDebugMsg: {
                GLCD_DisplayString(msgBuffer.buf[0],0,1, (unsigned char*) (msgBuffer.buf+1));
            }
            default:{
                break;
            }
        }

        // Here is a way to do debugging output via the built-in hardware -- it requires the ULINK cable and the
        //   debugger in the Keil tools to be connected.  You can view PORT0 output in the "Debug(printf) Viewer"
        //   under "View->Serial Windows".  You have to enable "Trace" and "Port0" in the Debug setup options.  This
        //   should not be used if you are using Port0 for printf()
        // There are 31 other ports and their output (and port 0's) can be seen in the "View->Trace->Records"
        //   windows.  You have to enable the prots in the Debug setup options.  Note that unlike ITM_SendChar()
        //   this "raw" port write is not blocking.  That means it can overrun the capability of the system to record
        //   the trace events if you go too quickly; that won't hurt anything or change the program execution and
        //   you can tell if it happens because the "View->Trace->Records" window will show there was an overrun.
        //vtITMu16(vtITMPortLCD,screenColor);
    }
}
