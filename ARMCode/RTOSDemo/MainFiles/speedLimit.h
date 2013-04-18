#include "myDefs.h"
#ifndef SPEEDLIMIT_H
#define SPEEDLIMIT_H
#include "myTypes.h"
#include "webServer.h"
#include "motorControl.h"
#include "navigation.h"

#define maxSpeedLimitMsgLen 9

// Public API
//
// The job of this task is to keep track of the state of the speed limit
// Start the task
// Args:
//   speedData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   myi2c: pointer to the data structure for the i2c task
//   navData: pointer to the data structure for the Navigation Task
//   mapData: pointer to the data structure for the Mapping Task
void vStartSpeedLimitTask(speedLimitControlStruct *speedData, unsigned portBASE_TYPE uxPriority, motorControlStruct *motorControl, navigationStruct *navData, webServerStruct *webData);

// Send the Speed Limit Control thread color sensor data from the I2C bus
// Args:
//   speedData -- a pointer to the data structure for the Speed Limit Control task
//   data -- buffer containing encoder data
//   length -- length of the data buffer
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE conductorSendColorSensorDataMsg(speedLimitControlStruct *speedData,uint8_t *data, uint8_t length);

#endif