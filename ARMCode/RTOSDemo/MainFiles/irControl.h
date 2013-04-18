#include "myDefs.h"
#ifndef IR_CONTROL_H
#define IR_CONTROL_H
#include "myTypes.h"
#include "navigation.h"
#include "LCDtask.h"

#define maxIRMsgLen 9

// Public API
//
// The job of this task is maintain a copy of the current map of the room
// Start the task
// Args:
//   mapData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   navData: pointer to the data structure for the navigation task
void vStartIRTask(irControlStruct *irData, unsigned portBASE_TYPE uxPriority, navigationStruct *navData,  vtLCDStruct *lcdData);

// Request the orientation information of the rover from the Navigation thread
// Args:
//   mapData -- a pointer to the data structure for the Mapping Task
//   degrees -- the orientation of rover is facing in degrees
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE conductorSendIRSensorDataMsg(irControlStruct *irData, uint8_t *data, uint8_t length);

#endif
