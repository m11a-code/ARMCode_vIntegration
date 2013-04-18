#include "myDefs.h"
#ifndef I2C_TASK_H
#define I2C_TASK_H
#include "vtI2C.h"
#include "LCDtask.h"
// Structure used to pass parameters to the task
// Do not touch...
typedef struct __i2cStruct {
    vtI2CStruct *dev;
    vtLCDStruct *lcdData;
    xQueueHandle inQ;
} myI2CStruct;
// Maximum length of a message that can be received by this task
//#define vti2cMaxLen   100    //was 5
#define vti2cMaxLen   9    //was 5

//#define I2C_MSG_SIZE 120      //was 4
#define I2C_MSG_SIZE 8      //was 4

#define SLAVE_ADDR 0x4F

// Public API
//
// Start the task
// Args:
//   i2cData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   i2c: pointer to the data structure for an i2c task
void starti2cTask(myI2CStruct *i2cData, unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c);
//
// Send a timer message to the i2c task
// Args:
//   i2cData -- a pointer to a variable of type myI2CStruct
//   ticksElapsed -- number of ticks since the last message (this will be sent in the message)
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE sendi2cTimerMsg(myI2CStruct *i2cData, portTickType ticksElapsed, portTickType ticksToBlock);

// Send a value message to the I2C task
// Args:
//   i2cData -- a pointer to a variable of type myI2CStruct
//   leftValue -- The value to send to the left motors
//   rightValue -- The value to send to the right motors
//   ticksToBlock -- how long the routine should wait if the queue is full
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE sendi2cMotorMsg(myI2CStruct *i2cData, uint8_t leftValue, uint8_t rightValue, portTickType ticksToBlock);

portBASE_TYPE sendi2cWebServerMsg(myI2CStruct *i2cData, uint8_t one, uint8_t two, portTickType ticksToBlock);

portBASE_TYPE notifyRequestRecvd(myI2CStruct *i2cData, portTickType ticksToBlock);
#endif
