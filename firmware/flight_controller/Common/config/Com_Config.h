#ifndef __COM_CONFIG_H
#define __COM_CONFIG_H

#include "stdint.h"
#include "stdio.h"

typedef enum
{
    Com_OK = 0,
    Com_TIMEOUT,
    Com_FAIL
} Com_Status;

typedef enum
{
    LEFT_TOP,
    LEFT_BOTTOM,
    RIGHT_TOP,
    RIGHT_BOTTOM
} Com_Location;

typedef struct
{
    /* 表示灯的位置 */
    Com_Location location;

    /* 表示灯的状态:
       0: 常亮
       1: 常灭
       2+: 闪烁周期(单位: 灯控任务的执行周期)
     */
    uint8_t status;
} LedStruct;

#define LED_ON  (0U) /* 常亮 */
#define LED_OFF (1U) /* 常灭 */

/* 表示电机的结构体 */
typedef struct
{
    Com_Location location;
    int16_t      speed;
} Motor_Struct;

/* 电机 PWM 比较值范围 */
#define MOTOR_STOP (0)    /* 停转 */
#define MOTOR_MAX  (1000) /* 满速 */

typedef struct
{
    int16_t THR; /* 油门 */
    int16_t PIT; /* 俯仰 */
    int16_t ROL; /* 横滚 */
    int16_t YAW; /* 偏航 */

    uint8_t isPowerDown; /* 是否关机: 1:关机 0:不关机 */
    uint8_t isFixHeight; /* 是否翻转定高的状态 */
} JoyStick_Struct;

/* 定义油门解锁时的几个状态 */
typedef enum
{
    THR_FREE = 0,
    THR_MAX,
    THR_MAX_LEAVE,
    THR_MIN,
    THR_UNLOCK
} Com_RemoteStatus;

/* 存储角速度 */
typedef struct
{
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} Gyro_Struct;

/* 存储加速度 */
typedef struct
{
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
} Accel_Struct;

/* 角速度和加速度 */
typedef struct
{
    Gyro_Struct  gyro;
    Accel_Struct accel;
} GyroAccel_Struct;

/* 欧拉角 */
typedef struct
{
    float pitch;
    float roll;
    float yaw;

} EulerAngle_Struct;

/* 表示四元数的结构体 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_Struct;

/* 表示pid 结构体 */
typedef struct
{
    float kp; /* 比例系数 */
    float ki; /* 积分系数 */
    float kd; /* 微分系数 */

    float dt; /* 采样时间 */

    float integral;  /*保存 积分值 */
    float lastError; /* 上次误差 */

    float desire; /* 期望值 */
    float measure; /* 测量值 */

    float result;  /* pid最终结果 */

    /* 限幅配置: 上限 <= 下限 时表示该限幅不启用(结构体初始化后默认为 0, 即不启用) */
    float outMin;      /* 输出下限 */
    float outMax;      /* 输出上限 */
    float integralMin; /* 积分下限 */
    float integralMax; /* 积分上限 */

    /* 1: 误差取最短角度路径(-180~180°), 用于偏航这类周期性角度量 */
    uint8_t isAngle;
} PID_Struct;

/* ======================== 2.4G 应用层协议 ========================
 * 一帧的字节布局(共 RF_FRAME_LEN 字节):
 *   [0..2]   帧头 FRAME_0 / FRAME_1 / FRAME_2
 *   [3]      有效载荷长度 = RF_PAYLOAD_LEN
 *   [4..13]  有效载荷
 *   [14..17] 累加校验和, 大端
 * =============================================================== */
#define FRAME_0 (0x11)
#define FRAME_1 (0x22)
#define FRAME_2 (0x33)

#define RF_HEADER_LEN   (4U)                                    /* 帧头(3) + 长度(1) */
#define RF_PAYLOAD_LEN  (10U)                                   /* 有效载荷字节数 */
#define RF_CHECKSUM_LEN (4U)                                    /* 校验和字节数 */
#define RF_FRAME_LEN    (RF_HEADER_LEN + RF_PAYLOAD_LEN + RF_CHECKSUM_LEN) /* 整帧 = 18 */

/* 取 x 在 [min, max] 之间的限幅值. 注意: 参数不应带副作用(可能被求值两次) */
#define LIMIT(x, min, max) (((x) >= (max)) ? (max) : (((x) <= (min)) ? (min) : (x)))

/* 取绝对值. 不用标准库的 abs(): 它在 <stdlib.h> 里, 本工程并未显式包含,
   而且对浮点实参会被隐式截断, 不如自己写清楚 */
#define COM_ABS(x) (((x) >= 0) ? (x) : -(x))

extern LedStruct ledLeftTop;
extern LedStruct ledLeftBottom;
extern LedStruct ledRightTop;
extern LedStruct ledRightBottom;

extern Com_Status isRemoteConnected;
extern Com_Status isRemoteUnlocked;
extern Com_Status isFixHeight;

extern Motor_Struct motorLeftTop;
extern Motor_Struct motorLeftBottom;
extern Motor_Struct motorRightTop;
extern Motor_Struct motorRightBottom;

extern JoyStick_Struct joyStick;

extern GyroAccel_Struct gyroAccel;

extern EulerAngle_Struct eulerAngle;

void Com_Config_PrintJoyStick(uint8_t *pre);

void Com_Config_PrintGyroAccel(uint8_t *pre, GyroAccel_Struct *gyroAccel);

void Com_Config_PrintEulerAngle(uint8_t *pre, EulerAngle_Struct *eulerAngle);

#endif
