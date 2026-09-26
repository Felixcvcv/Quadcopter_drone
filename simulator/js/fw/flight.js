/*
 * fw/flight.js — App_Flight.c 的移植
 *
 * 逐个函数对应:
 *   App_Flight_GetGyroAccelWithFilter   角速度一阶低通 + 加速度卡尔曼
 *   App_Flight_GetEulerAngle            调用 Com_IMU
 *   App_Flight_PIDPosture               3 组串级姿态 pid
 *   App_Flight_MotorWithPosturePID      X 字型机架分配
 *   App_Flight_GetHeight                激光测距 + 野值剔除
 *   App_Flight_PIDHeight                定高状态机 + 高度/速度串级 pid
 *   App_Flight_MotorWithHeightPID       定高作用量公共叠加
 *   App_Flight_Work                     输出到电机(含未解锁/低油门停转)
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});
    var cfg = QC.config;
    var pidLib = QC.pid;
    var filter = QC.filter;
    var imu = QC.imu;

    /* 对应 Inf_Motor_SetSpeed: 限幅后写入定时器比较寄存器 */
    function MotorSetSpeed(motor) {
        motor.speed = Math.round(pidLib.LIMIT(motor.speed, QC.state.MOTOR_STOP, QC.state.MOTOR_MAX));
    }

    function MotorStopAll(s) {
        s.motors.leftTop.speed = QC.state.MOTOR_STOP;
        s.motors.leftBottom.speed = QC.state.MOTOR_STOP;
        s.motors.rightTop.speed = QC.state.MOTOR_STOP;
        s.motors.rightBottom.speed = QC.state.MOTOR_STOP;
    }

    function MotorAllMotorsWork(s) {
        MotorSetSpeed(s.motors.leftTop);
        MotorSetSpeed(s.motors.leftBottom);
        MotorSetSpeed(s.motors.rightTop);
        MotorSetSpeed(s.motors.rightBottom);
    }

    /* 对应 App_Flight_GetGyroAccelWithFilter */
    function GetGyroAccelWithFilter(s, sensors) {
        var ga = s.gyroAccel;

        /* 1. 读取原始数据(已扣零偏) */
        sensors.readGyroAccelCalibrated(ga);

        /* 2. 角速度一阶低通 */
        ga.gyro.gyroX = filter.LowPass(ga.gyro.gyroX, s.lastGyro[0]);
        ga.gyro.gyroY = filter.LowPass(ga.gyro.gyroY, s.lastGyro[1]);
        ga.gyro.gyroZ = filter.LowPass(ga.gyro.gyroZ, s.lastGyro[2]);
        s.lastGyro[0] = ga.gyro.gyroX;
        s.lastGyro[1] = ga.gyro.gyroY;
        s.lastGyro[2] = ga.gyro.gyroZ;

        /* 3. 加速度逐轴卡尔曼 */
        ga.accel.accelX = Math.round(filter.Kalman(s.kfs[0], ga.accel.accelX));
        ga.accel.accelY = Math.round(filter.Kalman(s.kfs[1], ga.accel.accelY));
        ga.accel.accelZ = Math.round(filter.Kalman(s.kfs[2], ga.accel.accelZ));
    }

    /* 对应 App_Flight_GetEulerAngle */
    function GetEulerAngle(s, dt) {
        imu.GetEulerAngle(s.gyroAccel, s.eulerAngle, dt);
    }

    /* 对应 App_Flight_PIDPosture */
    function PIDPosture(s, dt) {
        var P = s.pids;
        var ga = s.gyroAccel;
        var ea = s.eulerAngle;

        /* 俯仰 */
        P.pitch.dt = dt;
        P.pitch.desire = (s.joyStick.PIT - cfg.STICK_CENTER) * cfg.STICK_TO_ANGLE;
        P.pitch.measure = ea.pitch;
        P.gyroY.dt = dt;
        P.gyroY.measure = ga.gyro.gyroY * cfg.GYRO_DEG_PER_LSB;
        pidLib.CascadePID(P.pitch, P.gyroY);

        /* 横滚 */
        P.roll.dt = dt;
        P.roll.desire = (s.joyStick.ROL - cfg.STICK_CENTER) * cfg.STICK_TO_ANGLE;
        P.roll.measure = ea.roll;
        P.gyroX.dt = dt;
        P.gyroX.measure = ga.gyro.gyroX * cfg.GYRO_DEG_PER_LSB;
        pidLib.CascadePID(P.roll, P.gyroX);

        /* 偏航 */
        P.yaw.dt = dt;
        P.yaw.desire = (s.joyStick.YAW - cfg.STICK_CENTER) * cfg.STICK_TO_ANGLE;
        P.yaw.measure = ea.yaw;
        P.gyroZ.dt = dt;
        P.gyroZ.measure = ga.gyro.gyroZ * cfg.GYRO_DEG_PER_LSB;
        pidLib.CascadePID(P.yaw, P.gyroZ);

        s.debug.rollOut = P.gyroX.result;
        s.debug.pitchOut = P.gyroY.result;
        s.debug.yawOut = P.gyroZ.result;
    }

    /* 对应 App_Flight_MotorWithPosturePID */
    function MotorWithPosturePID(s) {
        if (s.isRemoteUnlocked !== 0) return; /* Com_OK == 0 */

        var speed = s.joyStick.THR * cfg.THROTTLE_GAIN;
        var rx = s.pids.gyroX.result, ry = s.pids.gyroY.result, rz = s.pids.gyroZ.result;

        s.motors.leftTop.speed = speed + rx + ry + rz;
        s.motors.leftBottom.speed = speed + rx - ry - rz;
        s.motors.rightTop.speed = speed - rx + ry - rz;
        s.motors.rightBottom.speed = speed - rx - ry + rz;
    }

    /* 对应 App_Flight_GetHeight */
    function GetHeight(s, sensors) {
        var height = sensors.tofRead();
        s.debug.heightRaw = height;

        /* 测距跳变 / 正在水平打杆 时沿用上次结果 */
        if (Math.abs(height - s.lastHeight) > cfg.HEIGHT_GLITCH_MM ||
            Math.abs(s.joyStick.PIT - cfg.STICK_CENTER) > cfg.STICK_MOVE_MARGIN ||
            Math.abs(s.joyStick.ROL - cfg.STICK_CENTER) > cfg.STICK_MOVE_MARGIN) {
            s.debug.heightFiltered = s.lastHeight;
            return s.lastHeight;
        }

        height = filter.LowPass(height, s.lastHeight);
        s.lastHeight = height;
        s.debug.heightFiltered = height;
        return height;
    }

    /* 对应 App_Flight_PIDHeight */
    function PIDHeight(s, height, dt) {
        var A = s.alt;
        var P = s.pids;

        switch (A.status) {
            case 0: /* 待机 */
                pidLib.Reset(P.height);
                pidLib.Reset(P.zSpeed);

                /* 未解锁时飞机静止在地面, 持续跟踪重力基准 */
                if (s.isRemoteUnlocked !== 0) {
                    A.staticAcc = cfg.GRAVITY_REF_ALPHA * A.staticAcc +
                                  (1.0 - cfg.GRAVITY_REF_ALPHA) * imu.GetNormAccZ();
                }

                if (s.isRemoteUnlocked === 0 && s.isFixHeight === 0) {
                    A.status = 1;
                }
                break;

            case 1: /* 记录进入定高时的油门与高度 */
                A.thrHold = s.joyStick.THR;
                A.heightHold = height;
                A.status = 2;
                break;

            case 2: /* 运行 pid */
                if (Math.abs(s.joyStick.THR - A.thrHold) > cfg.HEIGHT_THR_EXIT_DELTA ||
                    s.isFixHeight === 2) {
                    A.status = 0;
                    s.joyStick.isFixHeight = 0;
                    s.isFixHeight = 2;
                    break;
                }

                A.cnt++;
                if (A.cnt < cfg.HEIGHT_PID_DIV) break;
                A.cnt = 0;

                var pidDt = dt * cfg.HEIGHT_PID_DIV;

                /* z 轴速度: 互补滤波 */
                var zSpeed = cfg.ZSPEED_ACC_WEIGHT *
                             (P.zSpeed.measure + (imu.GetNormAccZ() - A.staticAcc) * pidDt) +
                             cfg.ZSPEED_POS_WEIGHT * (height - P.height.measure) / pidDt;
                s.debug.zSpeedEstimate = zSpeed;

                P.height.desire = A.heightHold;
                P.height.measure = height;
                P.height.dt = pidDt;

                P.zSpeed.measure = zSpeed;
                P.zSpeed.dt = pidDt;

                pidLib.CascadePID(P.height, P.zSpeed);
                s.debug.zPidOut = P.zSpeed.result;
                break;

            default:
                break;
        }
    }

    /* 对应 App_Flight_MotorWithHeightPID */
    function MotorWithHeightPID(s) {
        if (s.isRemoteUnlocked !== 0) return;

        var zPid = Math.round(s.pids.zSpeed.result);
        s.motors.leftTop.speed += zPid;
        s.motors.leftBottom.speed += zPid;
        s.motors.rightTop.speed += zPid;
        s.motors.rightBottom.speed += zPid;
    }

    /* 对应 App_Flight_Work */
    function Work(s) {
        if (s.isRemoteUnlocked !== 0 || s.joyStick.THR <= cfg.THROTTLE_MIN) {
            MotorStopAll(s);
            MotorAllMotorsWork(s);
            return;
        }
        MotorAllMotorsWork(s);
    }

    /* ---- 一次飞控任务的完整流程 (对应 flightTask 的循环体) ---- */
    function FlightTaskStep(s, sensors, dt) {
        GetGyroAccelWithFilter(s, sensors);
        GetEulerAngle(s, dt);
        PIDPosture(s, dt);
        MotorWithPosturePID(s);
        var height = GetHeight(s, sensors);
        PIDHeight(s, height, dt);
        MotorWithHeightPID(s);
        Work(s);
    }

    QC.flight = {
        FlightTaskStep: FlightTaskStep,
        MotorSetSpeed: MotorSetSpeed,
        MotorStopAll: MotorStopAll,
        MotorAllMotorsWork: MotorAllMotorsWork,
        GetHeight: GetHeight
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
