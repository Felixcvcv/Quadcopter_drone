/*
 * fw/imu.js — Com_IMU.c 的移植 (四元数姿态解算 + 加速度计 PI 补偿)
 *
 * 逐行对应固件:
 *   1. 由四元数提取等效旋转矩阵中的重力分量
 *   2. 加速度归一化 (快速平方根倒数)
 *   3. 与估计重力方向做叉乘得到姿态误差
 *   4. 误差积分 (消除陀螺仪常值零漂) 并按 Kp/Ki 补偿到角速度
 *   5. 一阶龙格库塔法更新四元数
 *   6. 归一化
 *   7. 反解欧拉角 (pitch 对 asin 入参做了 ±1 限幅, 否则会出现 NaN)
 *   8. 记录机体 Z 轴方向加速度, 供定高使用
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});
    var cfg = QC.config;

    /* ---- 位模式转换视图: 对应 C 里的 union { float f; uint32_t i } ---- */
    var _f32 = new Float32Array(1);
    var _u32 = new Uint32Array(_f32.buffer);

    /**
     * 快速平方根倒数 1/sqrt(x)
     * 魔数初值 + 两次牛顿迭代 (相对误差约 5e-6).
     * 用 union 而不是指针强转, 避免破坏严格别名规则。
     */
    function Q_rsqrt(number) {
        var threehalfs = 1.5;
        var x2 = number * 0.5;

        _f32[0] = number;
        _u32[0] = (0x5f3759df - (_u32[0] >>> 1)) >>> 0;
        var y = _f32[0];

        y = y * (threehalfs - (x2 * y * y)); /* 第 1 次牛顿迭代 */
        y = y * (threehalfs - (x2 * y * y)); /* 第 2 次牛顿迭代 */
        return y;
    }

    function squa(v) { return v * v; }

    function LIMIT(x, min, max) {
        return (x >= max) ? max : ((x <= min) ? min : x);
    }

    /* ---- 姿态解算内部状态 (对应 Com_IMU.c 里的 static 变量) ---- */
    var state = {
        gyroIntegError: { x: 0, y: 0, z: 0 },
        quat: { q0: 1, q1: 0, q2: 0, q3: 0 },
        normAccz: 0
    };

    /* 复位姿态估计(上电或用户重置仿真时调用) */
    function Reset() {
        state.gyroIntegError.x = state.gyroIntegError.y = state.gyroIntegError.z = 0;
        state.quat.q0 = 1; state.quat.q1 = 0; state.quat.q2 = 0; state.quat.q3 = 0;
        state.normAccz = 0;
    }

    function GetQuaternion() {
        var q = state.quat;
        return { q0: q.q0, q1: q.q1, q2: q.q2, q3: q.q3 };
    }

    /**
     * 对应 Common_IMU_GetEulerAngle(gyroAccel, eulerAngle, dt)
     * gyroAccel 用 int16 原始值 (与固件一样是 ADC 量级的整数)
     */
    function GetEulerAngle(gyroAccel, eulerAngle, dt) {
        var Q = state.quat;
        var G = state.gyroIntegError;

        var Gravity = { x: 0, y: 0, z: 0 };
        var Acc = { x: 0, y: 0, z: 0 };
        var Gyro = { x: 0, y: 0, z: 0 };
        var AccGravity = { x: 0, y: 0, z: 0 };

        var q0 = Q.q0, q1 = Q.q1, q2 = Q.q2, q3 = Q.q3;
        var HalfTime = dt * 0.5;

        /* 1. 由当前四元数提取等效旋转矩阵中的重力分量 */
        Gravity.x = 2 * (q1 * q3 - q0 * q2);
        Gravity.y = 2 * (q0 * q1 + q2 * q3);
        Gravity.z = 1 - 2 * (q1 * q1 + q2 * q2);

        /* 2. 加速度归一化 */
        var NormQuat = Q_rsqrt(squa(gyroAccel.accel.accelX) +
                               squa(gyroAccel.accel.accelY) +
                               squa(gyroAccel.accel.accelZ));
        Acc.x = gyroAccel.accel.accelX * NormQuat;
        Acc.y = gyroAccel.accel.accelY * NormQuat;
        Acc.z = gyroAccel.accel.accelZ * NormQuat;

        /* 3. 测量重力方向与估计重力方向叉乘, 得到姿态误差 */
        AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
        AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
        AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

        /* 4. 误差积分, 用于消除陀螺仪常值零漂 */
        G.x += AccGravity.x * cfg.AHRS_KI;
        G.y += AccGravity.y * cfg.AHRS_KI;
        G.z += AccGravity.z * cfg.AHRS_KI;

        /* 5. 角速度 + 比例补偿 + 积分补偿 (弧度制) */
        Gyro.x = gyroAccel.gyro.gyroX * cfg.GYRO_DEG_PER_LSB * (Math.PI / 180) + cfg.AHRS_KP * AccGravity.x + G.x;
        Gyro.y = gyroAccel.gyro.gyroY * cfg.GYRO_DEG_PER_LSB * (Math.PI / 180) + cfg.AHRS_KP * AccGravity.y + G.y;
        Gyro.z = gyroAccel.gyro.gyroZ * cfg.GYRO_DEG_PER_LSB * (Math.PI / 180) + cfg.AHRS_KP * AccGravity.z + G.z;

        /* 6. 一阶龙格库塔法更新四元数 */
        var q0_t = (-q1 * Gyro.x - q2 * Gyro.y - q3 * Gyro.z) * HalfTime;
        var q1_t = (q0 * Gyro.x - q3 * Gyro.y + q2 * Gyro.z) * HalfTime;
        var q2_t = (q3 * Gyro.x + q0 * Gyro.y - q1 * Gyro.z) * HalfTime;
        var q3_t = (-q2 * Gyro.x + q1 * Gyro.y + q0 * Gyro.z) * HalfTime;

        q0 += q0_t; q1 += q1_t; q2 += q2_t; q3 += q3_t;

        /* 7. 归一化 */
        NormQuat = Q_rsqrt(squa(q0) + squa(q1) + squa(q2) + squa(q3));
        Q.q0 = q0 * NormQuat;
        Q.q1 = q1 * NormQuat;
        Q.q2 = q2 * NormQuat;
        Q.q3 = q3 * NormQuat;
        q0 = Q.q0; q1 = Q.q1; q2 = Q.q2; q3 = Q.q3;

        /* 8. 机体坐标系下的 Z 方向向量 */
        var vecxZ = 2 * q0 * q2 - 2 * q1 * q3;
        var vecyZ = 2 * q2 * q3 + 2 * q0 * q1;
        var veczZ = 1 - 2 * q1 * q1 - 2 * q2 * q2;

        /* 9. 反解欧拉角 */
        var yaw_G = gyroAccel.gyro.gyroZ * cfg.GYRO_DEG_PER_LSB;
        if (yaw_G > cfg.YAW_GYRO_DEADZONE || yaw_G < -cfg.YAW_GYRO_DEADZONE) {
            eulerAngle.yaw += yaw_G * dt;
        }

        /* 归一化的数值误差会让 vecxZ 略微超出 ±1, asin 会返回 NaN */
        eulerAngle.pitch = Math.asin(LIMIT(vecxZ, -1.0, 1.0)) * cfg.RAD_TO_DEG;
        eulerAngle.roll = Math.atan2(vecyZ, veczZ) * cfg.RAD_TO_DEG;

        /* 10. 机体 Z 轴方向加速度 */
        state.normAccz = gyroAccel.accel.accelX * vecxZ +
                         gyroAccel.accel.accelY * vecyZ +
                         gyroAccel.accel.accelZ * veczZ;
    }

    /* 对应 Common_IMU_GetNormAccZ */
    function GetNormAccZ() {
        return state.normAccz;
    }

    QC.imu = {
        GetEulerAngle: GetEulerAngle,
        GetNormAccZ: GetNormAccZ,
        GetQuaternion: GetQuaternion,
        Reset: Reset,
        Q_rsqrt: Q_rsqrt
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
