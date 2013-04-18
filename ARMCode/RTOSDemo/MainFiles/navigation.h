#include "myDefs.h"
#ifndef NAVIGATION_H
#define NAVIGATION_H
#include "myTypes.h"
#include "motorControl.h"
#include "LCDtask.h"

#define maxNavigationMsgLen 9

// Public API
//
// The job of this task is to get the rover from Point A to Point B
// Start the task
// Args:
//   navData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   motorControl: pointer to the data structure for the motor control task
//   lcdData: pointer to the data structure for the LCD task
// Return:
//   void
void vStartNavigationTask(navigationStruct *navData, unsigned portBASE_TYPE uxPriority, motorControlStruct *motorControl, vtLCDStruct *lcdData, myI2CStruct *i2cData);

//Navigation communication API

portBASE_TYPE AIUpdateDistances(navigationStruct *navData, uint8_t l1, uint8_t l2, uint8_t l3, uint8_t r1, uint8_t r2, uint8_t r3);

portBASE_TYPE AIUpdateLeftWall(navigationStruct *navData, short distance, signed char angle);
portBASE_TYPE AIUpdateFrontWall(navigationStruct *navData, short distance);
portBASE_TYPE AIUpdateRightWall(navigationStruct *navData, short distance, signed char angle);

portBASE_TYPE AIUpdateWallAngles(navigationStruct *navData, uint8_t l, uint8_t r);

portBASE_TYPE AIUpdateIsWalls(navigationStruct *navData, uint8_t wallFront, uint8_t wallBack, uint8_t wallLeft, uint8_t wallRight);

portBASE_TYPE AIUpdateFinishLine(navigationStruct *navData, uint8_t finishLine);

portBASE_TYPE sendNavTimerMsg(navigationStruct *navData, portTickType ticksElapsed, portTickType ticksToBlock);

#endif