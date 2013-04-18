#include "myDefs.h"
#ifndef MY_TIMERS_H
#define MY_TIMERS_H

#include "LCDtask.h"
#include "i2c_ARM.h"
#include "navigation.h"
#include "motorControl.h"

void startTimerFori2c(myI2CStruct *i2cdata);
void startTimerForMotor(motorControlStruct *motordata);
void startTimerForNav(navigationStruct *navData);

#endif
