#include "myDefs.h"
#ifndef WEB_SERVER_H
#define WEB_SERVER_H
#include "myTypes.h"
#include "i2c_ARM.h"
#include "LCDtask.h"

#define maxWebServerMsgLen 9

// Public API
//
// The job of this task is maintain a copy of the current map of the room
// Start the task
// Args:
//   mapData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   navData: pointer to the data structure for the navigation task
void vStartWebServerTask(webServerStruct *webServerData, unsigned portBASE_TYPE uxPriority, myI2CStruct *myi2c, vtLCDStruct *lcdData);

//Notify the web server of the current speed, 0xFF is the maximum
portBASE_TYPE webNotifyCurrentSpeed(webServerStruct *webServerData, uint8_t speed);

//Notify the web server of the current speed limit zone
// uint8_t "zone" value corresponds to current speed limit zone
portBASE_TYPE webNotifySpeedLimitZone(webServerStruct *webServerData, uint8_t zone);

//Notify the web server that the finish line is crossed
//***finish value is boolean!!!!
portBASE_TYPE webNotifyFinishLine(webServerStruct *webServerData, uint8_t finish);

//Notify the web server of the current power requirements
portBASE_TYPE webNotifyPower(webServerStruct *webServerData, uint8_t watts);

//Notify the web server the fastest time the rover could complete the course
//with the data gathered.
portBASE_TYPE webNotifyFastestTime(webServerStruct *webServerData, uint8_t fastMin, uint8_t fastSec);

//Notify the web server when a speed limit violation is occurring
//***speedViolation value is boolean!!!!
portBASE_TYPE webNotifySpeedViolation(webServerStruct *webServerData, uint8_t speedViolation);

#endif
