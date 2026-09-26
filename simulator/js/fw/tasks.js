/*
 * fw/tasks.js — App_Task.c 的任务调度
 *
 * 固件用 FreeRTOS 的 vTaskDelayUntil 让每个任务按固定周期执行; 仿真器同样按
 * 固定周期调用各任务的函数体。**控制周期严格保持 4ms**, 因为 PID 里的 dt 就是它,
 * 一旦周期变了, 固件里那组增益的行为也就变了。
 *
 * 任务表 (与 App_Task.c 对应):
 *   power_task   优先级 9   事件 / 10s 超时
 *   flight_task  优先级 8   4ms
 *   comm_task    优先级 8   6ms
 *   led_task     优先级 2   50ms
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});
    var cfg = QC.config;
    var comm = QC.comm;
    var flight = QC.flight;

    var LED_KEYS = ['leftTop', 'leftBottom', 'rightTop', 'rightBottom'];

    /* LED 灯效 (App_Task.c 的 ledTask) */
    function ledTaskStep(s) {
        var L = s.leds;

        if (s.isFixHeight === comm.Com_OK) {
            L.leftTop.status = L.leftBottom.status = cfg.LED_BLINK_FAST;
            L.rightTop.status = L.rightBottom.status = cfg.LED_BLINK_FAST;
        } else if (s.isRemoteUnlocked === comm.Com_OK) {
            if (s.leds.firstUnlock === 1) {
                L.leftTop.status = L.leftBottom.status = s.LED_OFF;
                L.rightTop.status = L.rightBottom.status = s.LED_OFF;
                s.leds.firstUnlock = 0;
            } else {
                L.leftTop.status = L.leftBottom.status = cfg.LED_BLINK_MID;
                L.rightTop.status = L.rightBottom.status = cfg.LED_BLINK_MID;
            }
        } else if (s.isRemoteConnected === comm.Com_OK) {
            s.leds.firstUnlock = 1;
            L.leftTop.status = L.rightTop.status = s.LED_OFF;
            L.leftBottom.status = L.rightBottom.status = cfg.LED_BLINK_SLOW;
        } else {
            L.leftTop.status = L.rightTop.status = cfg.LED_BLINK_SLOW;
            L.leftBottom.status = L.rightBottom.status = s.LED_OFF;
        }
    }

    /* 对应 Inf_LED_SetStatus: status<=1 直接置电平, >=2 翻转 */
    function ledApply(s) {
        s.ledCnt = (s.ledCnt || 0) + 1;
        for (var i = 0; i < 4; i++) {
            var led = s.leds[LED_KEYS[i]];
            if (led.status <= s.LED_OFF) {
                led.pinOn = (led.status === s.LED_ON);
            } else if (s.ledCnt % led.status === 0) {
                led.pinOn = !led.pinOn;
            }
        }
    }

    /* 电源任务: 等关机通知, 超时则补开机脉冲 (App_Task.c 的 powerTask) */
    function powerTaskStep(s) {
        if (s.powerNotify) {
            s.powerNotify = false;
            s.powerOn = false;
            s.debug.powerEvents = (s.debug.powerEvents || 0) + 1;
        }
        /* 超时分支: 固件里是给 IP5305T 补一个短按脉冲防止自动关机 */
        s.debug.keepAlive = true;
    }

    function makeScheduler(dtMsPerTick) {
        return {
            accum: { flight: 0, comm: 0, led: 0, power: 0 },
            stats: {
                flightRuns: 0, commRuns: 0, ledRuns: 0,
                framesSent: 0, framesLost: 0, framesCorrupt: 0, framesOk: 0
            },
            dtMsPerTick: dtMsPerTick || 1
        };
    }

    /**
     * 按各自周期触发任务
     * @param {number} dtMs 本次推进的模拟毫秒
     * @param {object} drone 飞控状态
     * @param {object} remote 遥控器状态
     * @param {object} sensors 虚拟传感器
     * @param {object} dronePos 无人机的真实位置(用于链路的距离衰减)
     * @param {object} remotePos 遥控器的位置
     * @param {function} rng 随机数
     */
    function advance(sc, dtMs, drone, remote, sensors, dronePos, remotePos, rng) {
        sc.accum.flight += dtMs;
        sc.accum.comm += dtMs;
        sc.accum.led += dtMs;
        sc.accum.power += dtMs;

        /* 传感器的时间基准跟随仿真时钟 (固件里是 SysTick) */
        sensors.advanceTime(dtMs);

        /* ---- flight_task: 4ms ---- */
        while (sc.accum.flight >= cfg.FLIGHT_EXEC_CYCLE) {
            sc.accum.flight -= cfg.FLIGHT_EXEC_CYCLE;

            if (drone.powerOn === false || drone.crashed) {
                /* 关机或坠机: 电机停转 */
                flight.MotorStopAll(drone);
                flight.MotorAllMotorsWork(drone);
            } else {
                flight.FlightTaskStep(drone, sensors, cfg.FLIGHT_EXEC_CYCLE / 1000);

                /*
                 * 数值发散保护 (仿真特有的看门狗)
                 *
                 * 快速平方根倒数在输入为 Infinity 时会算出 Inf*0 = NaN, 而 JS 的
                 * 双精度不像固件里的 float32 那样在 3.4e38 处饱和。真实固件遇到
                 * 同样的病态输入(例如在倾斜状态下做了加速度计校准, 于是姿态环
                 * 一直追一个不存在的倾角)会表现为姿态乱掉、直接摔机; 仿真里显式
                 * 检出并复位, 同时在 HUD 上提示。
                 */
                var ea = drone.eulerAngle;
                if (!isFinite(ea.pitch) || !isFinite(ea.roll) || !isFinite(ea.yaw)) {
                    QC.imu.Reset();
                    ea.pitch = 0; ea.roll = 0; ea.yaw = 0;
                    drone.attitudeDiverged = (drone.attitudeDiverged || 0) + 1;
                }
            }
            sc.stats.flightRuns++;
        }

        /* ---- comm_task: 6ms (遥控器发帧 + 飞控收帧) ---- */
        while (sc.accum.comm >= cfg.COMMUNICATION_EXEC_CYCLE) {
            sc.accum.comm -= cfg.COMMUNICATION_EXEC_CYCLE;

            var frame = comm.BuildFrame(remote);
            sc.stats.framesSent++;

            var dx = dronePos.x - remotePos.x;
            var dy = dronePos.y - remotePos.y;
            var dz = dronePos.z - remotePos.z;
            var dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            /* 低空且距离远时被楼体遮挡的概率更高 */
            var occluded = (dronePos.z < 25) && (dist > 70);

            var rx = comm.Transmit(sc.link, frame, dist, occluded, rng);
            var isReceiveData;

            if (rx === null) {
                sc.stats.framesLost++;
                isReceiveData = comm.Com_FAIL;
            } else {
                isReceiveData = comm.ReceiveFrame(drone, rx);
                if (isReceiveData === comm.Com_OK) sc.stats.framesOk++;
                else sc.stats.framesCorrupt++;
            }

            drone.isRemoteConnected = comm.CheckConnection(drone, isReceiveData);
            drone.isRemoteUnlocked = comm.RemoteUnlock(drone, drone.isRemoteConnected);

            if (isReceiveData === comm.Com_OK && drone.joyStick.isPowerDown) {
                drone.joyStick.isPowerDown = 0;
                drone.powerNotify = true;
            }
            sc.stats.commRuns++;
        }

        /* ---- led_task: 50ms ---- */
        while (sc.accum.led >= cfg.LED_EXEC_CYCLE) {
            sc.accum.led -= cfg.LED_EXEC_CYCLE;
            ledTaskStep(drone);
            ledApply(drone);
            sc.stats.ledRuns++;
        }

        /* ---- power_task: 10s 超时 ---- */
        while (sc.accum.power >= cfg.POWER_EXEC_CYCLE) {
            sc.accum.power -= cfg.POWER_EXEC_CYCLE;
            powerTaskStep(drone);
        }
    }

    QC.tasks = {
        makeScheduler: makeScheduler,
        advance: advance,
        ledTaskStep: ledTaskStep,
        ledApply: ledApply,
        powerTaskStep: powerTaskStep,
        LED_KEYS: LED_KEYS
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
