#include "App_Flight.h"

/* ============================== 控制参数 ============================== */

/* 摇杆: 中位 500, 满量程 +-500, 映射为 +-20 度的期望姿态角 */
#define STICK_CENTER      (500)
#define STICK_TO_ANGLE    (0.04f)
#define STICK_MOVE_MARGIN (100) /* 偏离中位超过该值认为在主动打杆 */

/* 油门: 遥控器输出 0~1000, 直接送给电机比较值偏猛, 取 0.7 的系数 */
#define THROTTLE_GAIN (0.7f)
/* 低于该油门不做输出, 直接停转 */
#define THROTTLE_MIN (30)

/* 定高的辅助输出限幅(叠加到4个电机上的公共量) */
#define HEIGHT_ASSIST_LIMIT (150.0f)
/* 定高过程中, 油门相对"进入定高时"的变化超过该值就退出定高 */
#define HEIGHT_THR_EXIT_DELTA (100)
/* 激光测距单次跳变超过该值(mm)视为野值 */
#define HEIGHT_GLITCH_MM (500)
/* 高度环每 HEIGHT_PID_DIV 个控制周期计算一次 */
#define HEIGHT_PID_DIV (5)
/* z轴速度互补滤波权重: 加速度积分占 0.9, 高度差分占 0.1 */
#define ZSPEED_ACC_WEIGHT (0.9f)
#define ZSPEED_POS_WEIGHT (0.1f)
/* 重力基准的低通系数(未解锁、飞机静止在地面时持续跟踪) */
#define GRAVITY_REF_ALPHA (0.9f)

/* ============================== 姿态PID ============================== */
/* 3 组串级pid: 外环控角度, 内环控角速度 */

/* 俯仰 */
static PID_Struct pitchPID = {.kp = -7.0f, .ki = 0.0f, .kd = 0.0f};  /* 外环 */
static PID_Struct gyroYPID = {.kp = 2.0f, .ki = 0.0f, .kd = 0.1f};   /* 内环 */

/* 横滚 */
static PID_Struct rollPID  = {.kp = -7.0f, .ki = 0.0f, .kd = 0.0f};  /* 外环 */
static PID_Struct gyroXPID = {.kp = -2.0f, .ki = 0.0f, .kd = -0.1f}; /* 内环 */

/* 偏航: 角度是周期量, 误差按最短路径折算, 避免跨 0/360 度时反向猛转 */
static PID_Struct yawPID   = {.kp = -2.2f, .ki = 0.0f, .kd = 0.0f, .isAngle = 1}; /* 外环 */
static PID_Struct gyroZPID = {.kp = -1.5f, .ki = 0.0f, .kd = 0.0f};  /* 内环 */

/* ============================== 定高PID ============================== */
/* 外环控高度, 内环控z轴速度 */
static PID_Struct heightPID = {.kp = -1.4f, .ki = 0.0f, .kd = 0.0f};
static PID_Struct zSpeedPID = {.kp = -1.3f,
                               .ki = 0.0f,
                               .kd = -0.08f,
                               .outMin = -HEIGHT_ASSIST_LIMIT,
                               .outMax = HEIGHT_ASSIST_LIMIT};

/**
 * @description: 飞行模块的启动
 * @return {*}
 */
void App_Flight_Start(void)
{
    /* 1. 初始化motor */
    debug_printfln("motor: init start");
    Inf_Motor_Init();
    debug_printfln("motor: init done");

    /* 2. 初始化MPU6050(内部会做静止校准) */
    debug_printfln("mpu6050: init start");
    Inf_MPU6050_Init();
    debug_printfln("mpu6050: init done");

    /* 3. 初始化VL53L1X */
    debug_printfln("vl53l1x: init start");
    Inf_VL53LX1_Init();
    debug_printfln("vl53l1x: init done");
}

/**
 * @description: 对六轴数据做滤波
 *   角速度: 一阶低通滤波(波动小, 保持相位)
 *   加速度: 卡尔曼滤波(波动大, 容易受振动干扰)
 * @param {GyroAccel_Struct} *gyroAccel
 * @return {*}
 */
void App_Flight_GetGyroAccelWithFilter(GyroAccel_Struct *gyroAccel)
{
    /* 1. 读取原始数据 */
    taskENTER_CRITICAL();
    Inf_MPU6050_ReadGyroAccelCalibrated(gyroAccel);
    taskEXIT_CRITICAL();

    /* 2. 对角速度做一阶低通滤波 */
    static int16_t lastDatas[3] = {0};

    gyroAccel->gyro.gyroX = Com_Filter_LowPass(gyroAccel->gyro.gyroX, lastDatas[0]);
    gyroAccel->gyro.gyroY = Com_Filter_LowPass(gyroAccel->gyro.gyroY, lastDatas[1]);
    gyroAccel->gyro.gyroZ = Com_Filter_LowPass(gyroAccel->gyro.gyroZ, lastDatas[2]);
    lastDatas[0]          = gyroAccel->gyro.gyroX;
    lastDatas[1]          = gyroAccel->gyro.gyroY;
    lastDatas[2]          = gyroAccel->gyro.gyroZ;

    /* 3. 对加速度做卡尔曼滤波 */
    gyroAccel->accel.accelX =
        (int16_t)Com_Filter_Kalman(&kfs[0], (float)gyroAccel->accel.accelX);
    gyroAccel->accel.accelY =
        (int16_t)Com_Filter_Kalman(&kfs[1], (float)gyroAccel->accel.accelY);
    gyroAccel->accel.accelZ =
        (int16_t)Com_Filter_Kalman(&kfs[2], (float)gyroAccel->accel.accelZ);
}

/**
 * @description: 获取欧拉角
 * @return {*}
 */
void App_Flight_GetEulerAngle(GyroAccel_Struct  *gyroAccel,
                              EulerAngle_Struct *eulerAngle,
                              float              dt)
{
    Common_IMU_GetEulerAngle(gyroAccel, eulerAngle, dt);
}

/**
 * @description: 计算姿态pid
 * @param {GyroAccel_Struct} *gyroAccel
 * @param {EulerAngle_Struct} *eulerAngle
 * @param {float} dt 调度周期
 * @return {*}
 */
void App_Flight_PIDPosture(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt)
{
    /* 俯仰 */
    pitchPID.dt      = dt;
    pitchPID.desire  = (joyStick.PIT - STICK_CENTER) * STICK_TO_ANGLE;
    pitchPID.measure = eulerAngle->pitch;

    gyroYPID.dt      = dt;
    gyroYPID.measure = gyroAccel->gyro.gyroY * Gyro_G;

    Com_PID_CascadePID(&pitchPID, &gyroYPID);

    /* 横滚 */
    rollPID.dt      = dt;
    rollPID.desire  = (joyStick.ROL - STICK_CENTER) * STICK_TO_ANGLE;
    rollPID.measure = eulerAngle->roll;

    gyroXPID.dt      = dt;
    gyroXPID.measure = gyroAccel->gyro.gyroX * Gyro_G;

    Com_PID_CascadePID(&rollPID, &gyroXPID);

    /* 偏航 */
    yawPID.dt      = dt;
    yawPID.desire  = (joyStick.YAW - STICK_CENTER) * STICK_TO_ANGLE;
    yawPID.measure = eulerAngle->yaw;

    gyroZPID.dt      = dt;
    gyroZPID.measure = gyroAccel->gyro.gyroZ * Gyro_G;

    Com_PID_CascadePID(&yawPID, &gyroZPID);
}

/**
 * @description: 把姿态pid的结果分配到4个电机上
 *   X字型机架, 各轴对电机的贡献:
 *     横滚 gyroXPID:  左上+左下  vs  右上+右下
 *     俯仰 gyroYPID:  左上+右上  vs  左下+右下
 *     偏航 gyroZPID:  左上+右下  vs  左下+右上
 * @param {Com_Status} isRemoteUnlock
 * @return {*}
 */
void App_Flight_MotorWithPosturePID(Com_Status isRemoteUnlock)
{
    if(isRemoteUnlock != Com_OK) return;

    int16_t speed = (int16_t)(joyStick.THR * THROTTLE_GAIN);

    motorLeftTop.speed     = (int16_t)(speed + gyroXPID.result + gyroYPID.result + gyroZPID.result);
    motorLeftBottom.speed  = (int16_t)(speed + gyroXPID.result - gyroYPID.result - gyroZPID.result);
    motorRightTop.speed    = (int16_t)(speed - gyroXPID.result + gyroYPID.result - gyroZPID.result);
    motorRightBottom.speed = (int16_t)(speed - gyroXPID.result - gyroYPID.result + gyroZPID.result);
}

/**
 * @description: 获取飞机的飞行高度
 * @return {*} 高度: mm
 */
uint16_t App_Flight_GetHeight(void)
{
    static uint16_t lastHeight = 0;
    uint16_t        height     = Inf_VL53LX1_GetHeight();

    /* 测距跳变(反光/遮挡), 或者正在水平飞行(激光斜射到地面)时, 沿用上次结果 */
    if(COM_ABS((int16_t)(height - lastHeight)) > HEIGHT_GLITCH_MM ||
       COM_ABS(joyStick.PIT - STICK_CENTER) > STICK_MOVE_MARGIN ||
       COM_ABS(joyStick.ROL - STICK_CENTER) > STICK_MOVE_MARGIN)
    {
        return lastHeight;
    }

    height     = (uint16_t)Com_Filter_LowPass((int16_t)height, (int16_t)lastHeight);
    lastHeight = height;

    return height;
}

/**
 * @description: 高度pid控制
 * @param {Com_Status} isRemoteUnlocked
 * @param {uint16_t} height
 * @param {float} dt
 * @return {*}
 */
void App_Flight_PIDHeight(Com_Status isRemoteUnlocked, uint16_t height, float dt)
{
    /* 定高状态机:
        状态0: 待机, 检测是否允许进入定高
        状态1: 记录进入定高时的油门和高度
        状态2: 运行串级pid
     */
    static uint8_t  status     = 0;
    static uint16_t thrHold    = 0;
    static uint16_t heightHold = 0;

    /* 静止时的z轴加速度, 作为 1g 重力基准 */
    static float staticAcc = 0.0f;

    switch(status)
    {
        case 0: /* 待机 */
        {
            /* 清空pid历史, 避免上一次定高残留的积分量 */
            Com_PID_Reset(&heightPID);
            Com_PID_Reset(&zSpeedPID);

            /* 未解锁时飞机静止在地面, 持续跟踪重力基准.
               原来只在第一次解锁时采样一次, 之后永不更新,
               若首次解锁时飞机并非水平静止, 这个基准会一直是错的 */
            if(isRemoteUnlocked != Com_OK)
            {
                staticAcc = GRAVITY_REF_ALPHA * staticAcc +
                            (1.0f - GRAVITY_REF_ALPHA) * Common_IMU_GetNormAccZ();
            }

            if(isRemoteUnlocked == Com_OK && isFixHeight == Com_OK)
            {
                status = 1;
            }
            break;
        }

        case 1: /* 记录定高时的油门与高度 */
        {
            thrHold    = joyStick.THR;
            heightHold = height;
            status     = 2;
            break;
        }

        case 2: /* 运行pid */
        {
            /* 油门变化超过阈值, 或者定高标记被取消 -> 退出定高 */
            if(COM_ABS(joyStick.THR - thrHold) > HEIGHT_THR_EXIT_DELTA || isFixHeight == Com_FAIL)
            {
                status               = 0;
                joyStick.isFixHeight = 0;
                isFixHeight          = Com_FAIL;
                break;
            }

            /* 高度环周期较长, 每 HEIGHT_PID_DIV 个控制周期算一次pid */
            static uint8_t cnt = 0;
            cnt++;
            if(cnt < HEIGHT_PID_DIV) return;
            cnt = 0;

            float pidDt = dt * (float)HEIGHT_PID_DIV;

            /* z轴速度: 互补滤波. 加速度积分响应快但会漂, 高度差分不漂但滞后 */
            float zSpeed = ZSPEED_ACC_WEIGHT * (zSpeedPID.measure +
                                                (Common_IMU_GetNormAccZ() - staticAcc) * pidDt) +
                           ZSPEED_POS_WEIGHT * (float)(height - heightPID.measure) / pidDt;

            /* 串级pid: 外环高度, 内环z轴速度 */
            heightPID.desire  = heightHold;
            heightPID.measure = height;
            heightPID.dt      = pidDt;

            zSpeedPID.measure = zSpeed;
            zSpeedPID.dt      = pidDt;

            Com_PID_CascadePID(&heightPID, &zSpeedPID);
            break;
        }

        default:
            break;
    }
}

/**
 * @description: 把定高pid的输出叠加到4个电机上
 * @param {Com_Status} isRemoteUnlocked
 * @return {*}
 */
void App_Flight_MotorWithHeightPID(Com_Status isRemoteUnlocked)
{
    /* 未解锁时不叠加定高输出 */
    if(isRemoteUnlocked != Com_OK) return;

    /* 限幅已经在 zSpeedPID 的 outMin/outMax 里配置好 */
    int16_t zPid = (int16_t)zSpeedPID.result;

    motorLeftTop.speed += zPid;
    motorLeftBottom.speed += zPid;
    motorRightTop.speed += zPid;
    motorRightBottom.speed += zPid;
}

/**
 * @description: 让飞机工作(把4个电机的速度真正写到定时器上)
 * @param {Com_Status} isRemoteUnlock
 * @return {*}
 */
void App_Flight_Work(Com_Status isRemoteUnlock)
{
    if(isRemoteUnlock != Com_OK || joyStick.THR <= THROTTLE_MIN)
    {
        Inf_Motor_StopAll();
        return;
    }

    Inf_Motor_AllMotorsWork();
}
