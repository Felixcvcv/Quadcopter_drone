/*
 * config.js — 仿真器的全部常量
 *
 * 每一项都标注了对应的固件出处。控制相关的常量必须与固件一致,
 * 否则仿真就不能反映真实固件的行为了。
 * 物理参数(质量/转动惯量/推力系数)是仿真器自己引入的: 固件里没有这些,
 * 它们决定了"这台虚拟飞机"好不好飞, 见 docs/05-仿真器.md。
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});

    QC.config = {
        /* ==================== 任务周期 (App_Task.c) ==================== */
        FLIGHT_EXEC_CYCLE: 4,        /* ms, 飞控任务 */
        COMMUNICATION_EXEC_CYCLE: 6, /* ms, 通讯任务 */
        LED_EXEC_CYCLE: 50,          /* ms, 灯控任务 */
        POWER_EXEC_CYCLE: 10000,     /* ms, 电源任务超时 */
        LED_BLINK_FAST: 2,           /* 灯态: 每 N 个灯控周期翻转一次 */
        LED_BLINK_MID: 10,
        LED_BLINK_SLOW: 15,

        /* ==================== 控制参数 (App_Flight.c) ==================== */
        STICK_CENTER: 500,
        STICK_TO_ANGLE: 0.04,        /* 摇杆 ±500 -> 期望角 ±20 度 */
        STICK_MOVE_MARGIN: 100,
        THROTTLE_GAIN: 0.7,
        THROTTLE_MIN: 30,

        HEIGHT_ASSIST_LIMIT: 150.0,
        HEIGHT_THR_EXIT_DELTA: 100,
        HEIGHT_GLITCH_MM: 500,
        HEIGHT_PID_DIV: 5,
        ZSPEED_ACC_WEIGHT: 0.9,
        ZSPEED_POS_WEIGHT: 0.1,
        GRAVITY_REF_ALPHA: 0.9,

        /* 3 组串级姿态 pid + 2 个定高 pid (App_Flight.c) */
        PID_POSTURE: {
            pitch: { kp: -7.0, ki: 0.0, kd: 0.0, isAngle: 0 },
            gyroY: { kp: 2.0, ki: 0.0, kd: 0.1, isAngle: 0 },
            roll: { kp: -7.0, ki: 0.0, kd: 0.0, isAngle: 0 },
            gyroX: { kp: -2.0, ki: 0.0, kd: -0.1, isAngle: 0 },
            /* 偏航是周期量, isAngle=1 让误差走最短路径 */
            yaw: { kp: -2.2, ki: 0.0, kd: 0.0, isAngle: 1 },
            gyroZ: { kp: -1.5, ki: 0.0, kd: 0.0, isAngle: 0 }
        },
        PID_HEIGHT: {
            height: { kp: -1.4, ki: 0.0, kd: 0.0 },
            zSpeed: {
                kp: -1.3, ki: 0.0, kd: -0.08,
                outMin: -150.0, outMax: 150.0
            }
        },

        /* ==================== 滤波器 (Com_Filter.c) ==================== */
        LOWPASS_ALPHA: 0.8,
        KALMAN_Q: 0.001,
        KALMAN_R: 0.543,

        /* ==================== 姿态解算 (Com_IMU.c) ==================== */
        RAD_TO_DEG: 57.2957795,
        GYRO_DEG_PER_LSB: 4000.0 / 65536.0, /* 度/秒 每 LSB */
        YAW_GYRO_DEADZONE: 0.5,             /* 度/秒 */
        AHRS_KP: 0.8,
        AHRS_KI: 0.0003,

        /* ==================== MPU6050 (Inf_MPU6050.c) ==================== */
        MPU_ACCEL_1G_LSB: 16383,   /* ±2g 量程, 1g ≈ 16384 LSB */
        MPU_ACCEL_LSB_PER_G: 16384,
        MPU_GYRO_LSB_PER_DPS: 65536 / 4000, /* 16.4 LSB/(度/秒) */
        MPU_STILL_DELTA_THRESHOLD: 10,
        MPU_STILL_SAMPLE_COUNT: 30,
        MPU_STILL_TIMEOUT: 2000,
        MPU_CALIB_SAMPLE_COUNT: 255,
        MPU_CALIB_SAMPLE_INTERVAL_MS: 3,

        /* ==================== 传感器噪声 (仿真引入) ==================== */
        /* 陀螺仪/加速度计的常值零偏与随机噪声, 用来检验零偏校准与
           AHRS 的积分补偿是否真的有效 */
        SENSOR: {
            gyroBiasDps: { x: 0.9, y: -0.6, z: 1.3 }, /* 常值零偏 度/秒 */
            gyroNoiseDps: 0.35,                        /* 白噪声 度/秒 */
            accelBiasG: 0.012,                         /* 零偏 g */
            accelNoiseG: 0.010                         /* 白噪声 g */
        },

        /* ==================== 2.4G 协议 (Com_Config.h) ==================== */
        FRAME_0: 0x11,
        FRAME_1: 0x22,
        FRAME_2: 0x33,
        RF_HEADER_LEN: 4,
        RF_PAYLOAD_LEN: 10,
        RF_CHECKSUM_LEN: 4,
        RF_FRAME_LEN: 18,
        RF_CHANNEL: 111,

        /* ==================== 解锁/链路 (App_Communication.c) ==================== */
        LOST_FRAME_LIMIT: 200,       /* 200 * 6ms = 1.2s 失联 */
        UNLOCK_HOLD_COUNT: 200,
        AUTO_LOCK_COUNT: 200 * 50,   /* 60s */
        THR_NEAR_MAX: 960,
        THR_NEAR_MIN: 20,

        /* ==================== 遥控器 (remote_controller) ==================== */
        STICK_VALUE_MAX: 1000,
        STICK_VALUE_MID: 500,
        KEY_DEBOUNCE_MS: 30,
        KEY_LONG_PRESS_MS: 500,
        KEY_HOLD_MAX_MS: 1200,
        CALIB_SAMPLE_COUNT: 100,
        CALIB_SAMPLE_INTERVAL: 10,
        TRIM_STEP: 10,

        /* ==================== VL53L1X (Inf_VL53LX1.c) ==================== */
        TOF_TIMING_BUDGET_MS: 20,
        TOF_INTER_MEASUREMENT_MS: 20,
        TOF_MAX_RANGE_MM: 4000,   /* 长距离模式下的有效量程(仿真限制) */
        TOF_NOISE_MM: 6,
        TOF_DROPOUT_PROB: 0.004,  /* 偶发丢包(测距无数据)的概率 */

        /* ==================== 虚拟飞机的物理参数 (仿真引入) ==================== */
        /*
         * 这些不是从固件来的。它们由 tools/tune.mjs 在参数空间里搜出来的,
         * 判据是"让固件里那组 PID 增益飞得又稳又快":
         *   - 三个姿态轴受扰后都在 0.11s 内收敛, 残余角速度 ~0.002 rad/s
         *   - 悬停油门落在 63% 行程(629), 上下都有足够余量
         *   - 满油门推重比 5.17
         * 几个参数的物理含义:
         *   angularDrag        转动阻尼. 这是最关键的参数: 桨叶随机体转动会产生
         *                      与角速度反向的力矩(来自诱导速度的变化), 量级约
         *                      0.066*4*a^2 ≈ 3e-4 N·m/(rad/s). 阻尼太小会让速率
         *                      内环的相位裕度不足, 表现为持续抖动。
         *   motorTimeConstant  空心杯电机+小桨的机械时间常数. 它给速率内环带来
         *                      相位滞后, 太大或太小都会让整定结果变差。
         *   thrustCoef         推力系数, 由"期望悬停转速"反推:
         *                      4*kT*w0^2 = m*g, w0 = 440
         */
        DRONE: {
            mass: 0.055,            /* kg */
            armLength: 0.05,        /* m, 电机到重心的距离(半轴距) */
            inertia: { x: 3.6e-5, y: 3.6e-5, z: 6.5e-5 }, /* kg·m^2 */
            thrustCoef: 6.965e-7,   /* N / (转速单位)^2 */
            torqueCoef: 2.0e-9,     /* N·m / (转速单位)^2, 反扭矩(偏航); 对应
                                       桨的扭矩/推力比约 0.002 m, 与 65mm 桨相符 */
            motorTimeConstant: 0.022, /* s */
            linearDrag: 0.055,      /* N/(m/s) 水平气动阻力 */
            verticalDrag: 0.075,    /* N/(m/s) 垂直阻尼 */
            angularDrag: 1.56e-4,   /* N·m/(rad/s) 转动阻尼 */
            maxSpeedCmd: 1000       /* 固件里的电机限幅上限 */
        },

        /* ==================== IMU 安装方向 (仿真引入) ==================== */
        /*
         * MPU6050 在机身上的贴装方向. 取 (1,1,1) 表示芯片三轴与机体系同向。
         *
         * 为什么可以这样取: 固件的姿态解算用的是标准 Hamilton 四元数 +
         * 标准旋转矩阵, 角度反解公式 (pitch=asin(-R20), roll=atan2(R21,R22))
         * 对任意"绕 Z 转 180°"的贴装都自洽。仿真的电机位置与旋向已经按
         * 固件混控反推确定(见 dynamics.js), 所以这里取同向即可。
         */
        IMU_MOUNT: { x: 1, y: 1, z: 1 },

        /* ==================== 遥控器摇杆极性 (仿真引入) ==================== */
        /*
         * 键盘拨动方向 -> 固件里 joyStick 通道值的极性.
         * 对应真实遥控器上电位器的接线方向(固件里那句
         * `1000 - raw*1000/4095` 就是一个整体反相)。
         * 取值的依据: 让"推杆方向 = 飞机运动方向"符合直觉 ——
         *   俯仰: 推杆向前 -> PIT 增大 -> 机头下压 -> 向前飞
         *   横滚: 推杆向右 -> ROL 减小 -> 向右横滚
         *      (因为本仿真里 swing roll 角为正表示"右翼抬起/向左横滚")
         *   偏航: 推杆向右 -> YAW 增大 -> 机头右转
         */
        STICK_POLARITY: { THR: 1, YAW: 1, PIT: 1, ROL: -1 },

        /* ==================== 场景 ==================== */
        WORLD: {
            size: 700,              /* m, 正方形场地边长 */
            groundLevel: 0,
            startPad: { x: 0, y: 0, z: 0 }
        }
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
