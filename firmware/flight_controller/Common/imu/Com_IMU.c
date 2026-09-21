#include "Com_IMU.h"
#include "math.h"

/* ============================ 欧拉角解算 ============================ */

#define RAD_TO_DEG (57.2957795f)          /* 弧度 -> 度 */
#define DEG_TO_RAD (3.1415926f / 180.0f)  /* 度 -> 弧度 */

/* 陀螺仪量程 +-2000度/秒 对应 65536/4000 LSB/(度/秒) */
#define GYRO_DEG_PER_LSB (4000.0f / 65536.0f)

/* 偏航角速度小于该值(度/秒)时认为是零漂, 不做积分 */
#define YAW_GYRO_DEADZONE (0.5f)

/* 加速度计对陀螺仪的 PI 补偿系数 */
#define AHRS_KP (0.8f)
#define AHRS_KI (0.0003f)

#define squa(Sq) (((float)Sq) * ((float)Sq)) /* 计算平方 */

const float Gyro_G  = GYRO_DEG_PER_LSB;                     /* 度/秒 */
static const float Gyro_Gr = GYRO_DEG_PER_LSB * DEG_TO_RAD; /* 弧度/秒 */

/* 三维向量 */
typedef struct
{
    float x;
    float y;
    float z;
} Com_Vector3f;

/* 机体Z轴方向上的加速度(倾斜时会包含重力在其他轴上的分量) */
static float normAccz;

/**
 * @description: 快速计算 1/sqrt(number)(平方根倒数)
 *   Cortex-M3 没有硬件浮点单元, 直接调用 sqrtf 开销较大.
 *   这里用经典的"魔数"给出初值, 再做两次牛顿迭代,
 *   相对误差约 5e-6, 对四元数/加速度的归一化来说精度足够.
 *   用 union 做位模式转换而不是指针强转, 避免破坏严格别名规则.
 * @param {float} number 必须为正数
 * @return {*} 1/sqrt(number)
 */
static float Q_rsqrt(float number)
{
    const float threehalfs = 1.5f;
    float       x2         = number * 0.5f;
    union
    {
        float    f;
        uint32_t i;
    } u;

    u.f = number;
    u.i = 0x5f3759dfU - (u.i >> 1); /* 用整数减法猜出接近 1/sqrt(x) 的初值 */
    u.f = u.f * (threehalfs - (x2 * u.f * u.f)); /* 第 1 次牛顿迭代 */
    u.f = u.f * (threehalfs - (x2 * u.f * u.f)); /* 第 2 次牛顿迭代 */

    return u.f;
}

/**
 * @description: 根据六轴数据解算表征姿态的欧拉角
 *   用四元数做姿态递推(一阶龙格库塔), 并用加速度计测得的重力方向
 *   通过 PI 补偿陀螺仪的积分漂移.
 * @param {GyroAccel_Struct} *gyroAccel 六轴原始数据
 * @param {EulerAngle_Struct} *eulerAngle 解算得到的欧拉角(度)
 * @param {float} dt 采样周期(秒)
 * @return {*}
 */
void Common_IMU_GetEulerAngle(GyroAccel_Struct  *gyroAccel,
                              EulerAngle_Struct *eulerAngle,
                              float              dt)
{
    Com_Vector3f Gravity, Acc, Gyro, AccGravity;

    static Com_Vector3f      GyroIntegError = {0.0f, 0.0f, 0.0f};
    static Quaternion_Struct NumQ           = {1.0f, 0.0f, 0.0f, 0.0f};

    float q0_t, q1_t, q2_t, q3_t;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    /* 1. 由当前四元数提取等效旋转矩阵中的重力分量 */
    Gravity.x = 2.0f * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2.0f * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1.0f - 2.0f * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    /* 2. 加速度归一化 */
    NormQuat = Q_rsqrt(squa(gyroAccel->accel.accelX) +
                       squa(gyroAccel->accel.accelY) +
                       squa(gyroAccel->accel.accelZ));
    Acc.x = gyroAccel->accel.accelX * NormQuat;
    Acc.y = gyroAccel->accel.accelY * NormQuat;
    Acc.z = gyroAccel->accel.accelZ * NormQuat;

    /* 3. 测量到的重力方向与估计的重力方向做叉乘, 得到姿态误差 */
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    /* 4. 误差积分, 用于消除陀螺仪的常值零漂 */
    GyroIntegError.x += AccGravity.x * AHRS_KI;
    GyroIntegError.y += AccGravity.y * AHRS_KI;
    GyroIntegError.z += AccGravity.z * AHRS_KI;

    /* 5. 角速度 + 比例补偿 + 积分补偿, 得到修正后的角速度(弧度制) */
    Gyro.x = gyroAccel->gyro.gyroX * Gyro_Gr + AHRS_KP * AccGravity.x + GyroIntegError.x;
    Gyro.y = gyroAccel->gyro.gyroY * Gyro_Gr + AHRS_KP * AccGravity.y + GyroIntegError.y;
    Gyro.z = gyroAccel->gyro.gyroZ * Gyro_Gr + AHRS_KP * AccGravity.z + GyroIntegError.z;

    /* 6. 一阶龙格库塔法更新四元数 */
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    /* 7. 四元数归一化 */
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    /* 8. 机体坐标系下的Z方向向量 */
    float vecxZ = 2.0f * NumQ.q0 * NumQ.q2 - 2.0f * NumQ.q1 * NumQ.q3; /* 矩阵(3,1)项 */
    float vecyZ = 2.0f * NumQ.q2 * NumQ.q3 + 2.0f * NumQ.q0 * NumQ.q1; /* 矩阵(3,2)项 */
    float veczZ = 1.0f - 2.0f * NumQ.q1 * NumQ.q1 - 2.0f * NumQ.q2 * NumQ.q2; /* 矩阵(3,3)项 */

    /* 9. 由旋转矩阵元素反解欧拉角 */
    /* 偏航: 直接对Z轴角速度积分, 不做加速度补偿(加速度计测不到偏航) */
    float yaw_G = gyroAccel->gyro.gyroZ * Gyro_G;
    if(yaw_G > YAW_GYRO_DEADZONE || yaw_G < -YAW_GYRO_DEADZONE)
    {
        eulerAngle->yaw += yaw_G * dt;
    }

    /* 归一化的数值误差会让 vecxZ 略微超出 +-1, asin 会返回 NaN 并污染整条控制链 */
    eulerAngle->pitch = asinf(LIMIT(vecxZ, -1.0f, 1.0f)) * RAD_TO_DEG;
    eulerAngle->roll  = atan2f(vecyZ, veczZ) * RAD_TO_DEG;

    /* 10. 记录Z轴方向上的加速度, 供定高的速度估计使用 */
    normAccz = gyroAccel->accel.accelX * vecxZ +
               gyroAccel->accel.accelY * vecyZ +
               gyroAccel->accel.accelZ * veczZ;
}

/**
 * @description: 获取Z轴方向上的加速度
 * @return {*}
 */
float Common_IMU_GetNormAccZ(void)
{
    return normAccz;
}
