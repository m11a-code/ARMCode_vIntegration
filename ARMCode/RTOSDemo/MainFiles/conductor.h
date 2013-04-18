#include "myDefs.h"

#ifndef CONDUCTOR_H
#define CONDUCTOR_H

#include "vtI2C.h"
#include "i2c_ARM.h"
#include "motorControl.h"
#include "irControl.h"
#include "speedLimit.h"
#include "power.h"
#include "LCDtask.h"
#include "webServer.h"

// Structure used to pass parameters to the task
// Do not touch...
typedef struct __ConductorStruct {
	vtI2CStruct *dev;
	myI2CStruct *i2cData;
    motorControlStruct *motorControl;
    irControlStruct *irData;
    speedLimitControlStruct *speedData;
    powerStruct *powerData;
    vtLCDStruct *lcdData;
    webServerStruct *webData;
} vtConductorStruct;

// Public API
//
// The job of this task is to read from the message queue that is output by the I2C thread and to distribute the messages to the right
//   threads.
// Start the task.
// Args:
//   conductorData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   i2cData: pointer to the data structure for an i2c interrupt handler
//   myi2cData: pointer to the data structure for the i2c task
//   motorControl: pointer to the data structure for the motor control task
//   irData: pointer to the data structure for the IR control task
//   speedData: pointer to the data structure for the speed limit control task
//   powerData: pointer to the data structure for the power analysis task
//   lcdData: pointer to the data structure for the LCD task
void vStartConductorTask(vtConductorStruct *conductorData, unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, myI2CStruct * myi2c, motorControlStruct *motorControl, irControlStruct *irData, speedLimitControlStruct *speedData, powerStruct *powerData, vtLCDStruct *lcdData, webServerStruct *webData);

#endif
