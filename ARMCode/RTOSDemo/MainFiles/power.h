#include "myDefs.h"
#ifndef POWER_H
#define POWER_H
#include "myTypes.h"

#define maxPowerMsgLen 9

// Public API
//
// The job of this task is maintain a copy of the current map of the room
// Start the task
// Args:
//   mapData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   navData: pointer to the data structure for the navigation task
void vStartPowerTask(powerStruct *powerData, unsigned portBASE_TYPE uxPriority);

// Request the orientation information of the rover from the Navigation thread
// Args:
//   mapData -- a pointer to the data structure for the Mapping Task
//   degrees -- the orientation of rover is facing in degrees
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE conductorSendPowerDataMsg(powerStruct *powerData, uint8_t *data, uint8_t length);

#endif