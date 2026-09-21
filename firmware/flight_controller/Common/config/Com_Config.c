#include "Com_Config.h"

/* 定义4个LED */
LedStruct ledLeftTop     = {LEFT_TOP, LED_OFF};
LedStruct ledLeftBottom  = {LEFT_BOTTOM, LED_OFF};
LedStruct ledRightTop    = {RIGHT_TOP, LED_OFF};
LedStruct ledRightBottom = {RIGHT_BOTTOM, LED_OFF};

/* 定义飞控板的状态 */
/* 1. 遥控器是否连接成功 */
Com_Status isRemoteConnected = Com_FAIL;
/* 2. 遥控器是否解锁 */
Com_Status isRemoteUnlocked = Com_FAIL;
/* 3. 是否开启定高模式 */
Com_Status isFixHeight = Com_FAIL;

/* 4个电机 */
Motor_Struct motorLeftTop     = {LEFT_TOP, MOTOR_STOP};
Motor_Struct motorLeftBottom  = {LEFT_BOTTOM, MOTOR_STOP};
Motor_Struct motorRightTop    = {RIGHT_TOP, MOTOR_STOP};
Motor_Struct motorRightBottom = {RIGHT_BOTTOM, MOTOR_STOP};

/* 摇杆数据结构体 */
JoyStick_Struct joyStick;

/* 陀螺仪数据结构体 */
GyroAccel_Struct gyroAccel;

/* 欧拉角 */
EulerAngle_Struct eulerAngle;

void Com_Config_PrintJoyStick(uint8_t *pre)
{
    printf("%s thr = %d, pit = %d, rol = %d, yaw = %d\r\n",
           pre,
           joyStick.THR,
           joyStick.PIT,
           joyStick.ROL,
           joyStick.YAW);
}

void Com_Config_PrintGyroAccel(uint8_t *pre, GyroAccel_Struct *gyroAccel)
{
    printf("%s gx = %d, gy = %d, gz = %d, ax = %d, ay = %d, az= %d\r\n",
           pre,
           gyroAccel->gyro.gyroX,
           gyroAccel->gyro.gyroY,
           gyroAccel->gyro.gyroZ,
           gyroAccel->accel.accelX,
           gyroAccel->accel.accelY,
           gyroAccel->accel.accelZ);
}

void Com_Config_PrintEulerAngle(uint8_t *pre, EulerAngle_Struct *eulerAngle)
{
    printf("%s pitch = %.1f, roll = %.1f, yaw = %.1f\r\n",
           pre,
           eulerAngle->pitch,
           eulerAngle->roll,
           eulerAngle->yaw);
}
