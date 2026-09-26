/*
 * fw/state.js — Com_Config.c 里的全局状态
 *
 * 固件把这些变量放在全局, 由各个模块共享(App_Flight / App_Communication /
 * App_Task / Inf_*). 仿真器用同一个结构体保存一份, 让移植过来的代码
 * 写法与固件保持一致。
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});
    var pid = QC.pid;
    var filter = QC.filter;
    var cfg = QC.config;

    /* LED 位置 (对应 Com_Location) */
    var LOC = { LEFT_TOP: 0, LEFT_BOTTOM: 1, RIGHT_TOP: 2, RIGHT_BOTTOM: 3 };
    /* LED 状态: 0 常亮, 1 常灭, >=2 闪烁翻转周期 */
    var LED_ON = 0, LED_OFF = 1;
    /* 电机转速范围 (对应 MOTOR_STOP / MOTOR_MAX) */
    var MOTOR_STOP = 0, MOTOR_MAX = 1000;

    function makeDroneState() {
        /* 4 个 LED; pinOn 表示引脚电平(仿真里 LED 是低电平点亮) */
        var leds = {
            firstUnlock: 1,
            leftTop: { location: LOC.LEFT_TOP, status: LED_OFF, pinOn: false },
            leftBottom: { location: LOC.LEFT_BOTTOM, status: LED_OFF, pinOn: false },
            rightTop: { location: LOC.RIGHT_TOP, status: LED_OFF, pinOn: false },
            rightBottom: { location: LOC.RIGHT_BOTTOM, status: LED_OFF, pinOn: false }
        };

        /* 4 个电机 */
        var motors = {
            leftTop: { location: LOC.LEFT_TOP, speed: MOTOR_STOP },
            leftBottom: { location: LOC.LEFT_BOTTOM, speed: MOTOR_STOP },
            rightTop: { location: LOC.RIGHT_TOP, speed: MOTOR_STOP },
            rightBottom: { location: LOC.RIGHT_BOTTOM, speed: MOTOR_STOP }
        };

        return {
            LOC: LOC,
            LED_ON: LED_ON,
            LED_OFF: LED_OFF,
            MOTOR_STOP: MOTOR_STOP,
            MOTOR_MAX: MOTOR_MAX,

            leds: leds,
            motors: motors,

            /* 灯控任务的翻转计数 (App_Task.c 灯控任务里的 cnt) */
            ledCnt: 0,

            /* 电源状态: 收到关机命令后变为 false */
            powerOn: true,
            powerNotify: false,

            /* 坠机标志(仿真引入, 固件里对应"电机停转") */
            crashed: false,

            /* 飞控板状态 (Com_Status: 0=OK, 1=TIMEOUT, 2=FAIL) */
            isRemoteConnected: 2,
            isRemoteUnlocked: 2,
            isFixHeight: 2,

            /* 摇杆数据 */
            joyStick: {
                THR: 0, PIT: 500, ROL: 500, YAW: 500,
                isPowerDown: 0, isFixHeight: 0
            },

            /* 六轴数据与欧拉角 */
            gyroAccel: {
                gyro: { gyroX: 0, gyroY: 0, gyroZ: 0 },
                accel: { accelX: 0, accelY: 0, accelZ: 0 }
            },
            eulerAngle: { pitch: 0, roll: 0, yaw: 0 },

            /* 8 个 PID (App_Flight.c 里的 static PID_Struct) */
            pids: {
                pitch: pid.PID_Struct(cfg.PID_POSTURE.pitch),
                gyroY: pid.PID_Struct(cfg.PID_POSTURE.gyroY),
                roll: pid.PID_Struct(cfg.PID_POSTURE.roll),
                gyroX: pid.PID_Struct(cfg.PID_POSTURE.gyroX),
                yaw: pid.PID_Struct(cfg.PID_POSTURE.yaw),
                gyroZ: pid.PID_Struct(cfg.PID_POSTURE.gyroZ),
                height: pid.PID_Struct(cfg.PID_HEIGHT.height),
                zSpeed: pid.PID_Struct(cfg.PID_HEIGHT.zSpeed)
            },

            /* 三轴加速度的卡尔曼滤波器 (Com_Filter.c 的 kfs[3]) */
            kfs: [filter.KalmanFilter(), filter.KalmanFilter(), filter.KalmanFilter()],

            /* 定高状态机的持久变量 (App_Flight_PIDHeight 的 static) */
            alt: {
                status: 0,
                thrHold: 0,
                heightHold: 0,
                staticAcc: 0,
                cnt: 0
            },

            /* 姿态滤波的上一拍数据 (App_Flight_GetGyroAccelWithFilter 的 static) */
            lastGyro: [0, 0, 0],

            /* 定高测距的上一拍值 (App_Flight_GetHeight 的 static) */
            lastHeight: 0,

            /* 调试/显示用的内部量 */
            debug: {
                zSpeedEstimate: 0,
                heightRaw: 0,
                heightFiltered: 0,
                rollOut: 0,
                pitchOut: 0,
                yawOut: 0,
                zPidOut: 0
            },

            /* 通讯状态机的持久变量 (App_Communication.c 的 static) */
            comm: {
                lostCnt: cfg.LOST_FRAME_LIMIT, /* 初值取上限, 避免刚上电就显示已连接 */
                remoteStatus: 0,               /* THR_FREE */
                thrMaxDuration: 0,
                thrMinDuration: 0,
                lowDuration: 0
            }
        };
    }

    /* 遥控器的状态: 摇杆/零偏/按键 (对应 remote_controller 的 Com_Config.c) */
    function makeRemoteState() {
        return {
            joyStick: {
                THR: 0, PIT: 500, ROL: 500, YAW: 500,
                isPowerDown: 0, isFixHeight: 0
            },
            joyStickBias: { THR: 0, PIT: 0, ROL: 0, YAW: 0 },

            /* 4 路 ADC 原始值(12位), 由键盘或鼠标拖动产生的行程量反推 */
            adc: [0, 0, 0, 0],

            /* 按键: 记录按下起始时刻, 用来区分短按/长按 */
            key: { pressedAt: {}, lastEvent: 0 },

            /* 电源键脉冲状态 */
            powerKey: { level: 1, pendingUntil: 0 },

            /* 累计发送帧数与单次触发的命令 */
            txCount: 0
        };
    }

    /* 对应 Com_PID_Reset 的批量复位: 重新解锁/重新进入定高前清空状态 */
    function ResetControllers(s) {
        pid.Reset(s.pids.height);
        pid.Reset(s.pids.zSpeed);
    }

    QC.state = {
        /* 这几个常量也要导出: fw/flight.js 里按固件的写法直接用
           QC.state.MOTOR_STOP 之类的名字 */
        LOC: LOC,
        LED_ON: LED_ON,
        LED_OFF: LED_OFF,
        MOTOR_STOP: MOTOR_STOP,
        MOTOR_MAX: MOTOR_MAX,

        makeDroneState: makeDroneState,
        makeRemoteState: makeRemoteState,
        ResetControllers: ResetControllers
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
