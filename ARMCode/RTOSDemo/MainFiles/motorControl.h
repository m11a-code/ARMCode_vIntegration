#include "myDefs.h"
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "i2c_ARM.h"
#include "webServer.h"
#include "myTypes.h"
#include "LCDtask.h"

#define maxMotorMsgLen 9

// Public API
//
// The job of this task is to provide an API for the Navigation task to move the Rover
// Start the task
// Args:
//   motorControlData: Data structure used by the task
//   uxPriority -- the priority you want this task to be run at
//   myi2c: pointer to the data structure for the i2c task
//   navData: pointer to the data structure for the Navigation Task
void vStartMotorControlTask(motorControlStruct *motorControlData, unsigned portBASE_TYPE uxPriority, myI2CStruct *myi2c, webServerStruct *webData, vtLCDStruct *lcdData);

//Motor Commands

// Tell the rover to move backward
// Args:
//   motorControlData -- a pointer to the data structure for the Motor Control Task
//   centimeters -- the number of centimeters the rover should move backward
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE sendMotorSetDirForward(motorControlStruct *motorControlData);

// Tell the rover to move backward
// Args:
//   motorControlData -- a pointer to the data structure for the Motor Control Task
//   centimeters -- the number of centimeters the rover should move backward
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE sendMotorSetDirReverse(motorControlStruct *motorControlData);

// Tell the rover to move backward
// Args:
//   motorControlData -- a pointer to the data structure for the Motor Control Task
//   centimeters -- the number of centimeters the rover should move backward
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE sendMotorSetSpeed(motorControlStruct *motorControlData, uint8_t speed);

// Tell the rover rotate clockwise
// Args:
//   motorControlData -- a pointer to the data structure for the Motor Control Task
//   degrees -- the number of degrees to rotate (between 0 and 180)
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE sendMotorTurnLeft(motorControlStruct *motorControlData, uint8_t mag);

// Tell the rover rotate counterclockwise
// Args:
//   motorControlData -- a pointer to the data structure for the Motor Control Task
//   degrees -- the number of degrees to rotate (between 0 and 180)
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE sendMotorTurnRight(motorControlStruct *motorControlData, uint8_t mag);

// Tell the rover to abort its current motor operation (AKA Stop the rover)
// Args:
//   motorControlData -- a pointer to the data structure for the Motor Control Task
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE sendMotorStop(motorControlStruct *motorControlData);

// Send the Motor Control thread encoder data from the I2C bus
// Args:
//   motorControlData -- a pointer to the data structure for the Motor Control Task
//   data -- buffer containing encoder data
//   length -- length of the data buffer
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE conductorSendMotorEncoderDataMsg(motorControlStruct *motorControlData, uint8_t *data, uint8_t length);

portBASE_TYPE sendMotorTimerMsg(motorControlStruct *motorData, portTickType ticksElapsed, portTickType ticksToBlock);
#endif
