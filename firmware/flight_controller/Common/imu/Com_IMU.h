#ifndef __COM_IMU_H
#define __COM_IMU_H
#include "Com_Debug.h"
#include "Com_Config.h"

/* 陀螺仪原始值 -> 角速度(度/秒), 量程 +-2000度/秒 时灵敏度为 16.4 LSB/(度/秒) */
extern const float Gyro_G;

/* 根据六轴数据解算欧拉角(四元数 + 加速度计 PI 补偿) */
void Common_IMU_GetEulerAngle(GyroAccel_Struct  *gyroAccel,
                              EulerAngle_Struct *eulerAngle,
                              float              dt);

/* 机体Z轴方向的加速度(已包含倾斜时的分量合成) */
float Common_IMU_GetNormAccZ(void);

#endif
