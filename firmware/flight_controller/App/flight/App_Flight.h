#ifndef __APP_FLIGHT_H
#define __APP_FLIGHT_H
#include "Com_Debug.h"
#include "Inf_Motor.h"
#include "Inf_MPU6050.h"
#include "Com_Filter.h"
#include "FreeRTOS.h"
#include "task.h"
#include "outputdata.h"
#include "Com_Config.h"
#include "Com_IMU.h"
#include "Com_PID.h"
#include "Inf_VL53LX1.h"


void App_Flight_Start(void);
void App_Flight_GetGyroAccelWithFilter(GyroAccel_Struct *gyroAccel);
void App_Flight_PIDPosture(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt);
void App_Flight_MotorWithPosturePID(Com_Status isRemoteUnlock);
uint16_t App_Flight_GetHeight(void);
void     App_Flight_GetEulerAngle(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt);
void     App_Flight_PIDHeight(Com_Status isRemoteUnlocked, uint16_t height, float dt);
void     App_Flight_MotorWithHeightPID(Com_Status isRemoteUnlocked);
void     App_Flight_Work(Com_Status isRemoteUnlock);
#endif
