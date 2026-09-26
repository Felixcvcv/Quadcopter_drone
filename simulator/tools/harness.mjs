/*
 * tools/harness.mjs — Node 无头测试台
 *
 * 把浏览器里那些普通 <script> 加载的模块用 vm 跑进同一个全局环境,
 * 于是可以在 Node 里直接跑固件算法 + 物理仿真, 做数值验证。
 * 浏览器和测试用的是**同一份代码**, 不存在"测试版"和"发布版"漂移。
 */
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const JS_DIR = path.join(HERE, '..', 'js');

/* 加载顺序 = index.html 里的 <script> 顺序 */
export const LOAD_ORDER = [
    'config.js',
    'sim/math.js',
    'world/layout.js',
    'fw/pid.js',
    'fw/filter.js',
    'fw/imu.js',
    'fw/state.js',
    'fw/flight.js',
    'fw/comm.js',
    'fw/tasks.js',
    'sim/dynamics.js',
    'sim/sensors.js',
    'ui/input.js'
];

let loaded = false;

export function loadModules() {
    /*
     * 只加载一次。
     * 这些模块是"普通 <script> + 全局命名空间"的写法, config.js 每次执行都会
     * 用新对象覆盖 QC.config。如果重复加载, 外部对 QC.config 的修改(例如整定
     * 脚本改物理参数)就会被悄悄丢掉。
     */
    if (loaded) return globalThis.QC;
    for (const rel of LOAD_ORDER) {
        const file = path.join(JS_DIR, rel);
        const code = fs.readFileSync(file, 'utf8');
        vm.runInThisContext(code, { filename: rel });
    }
    loaded = true;
    return globalThis.QC;
}

/**
 * 搭一个完整的仿真台: 飞控状态 + 遥控器状态 + 传感器 + 物理 + 调度器
 * @param {object} opts { seed, startZ, initialTiltPitchDeg, initialTiltRollDeg, calibrate }
 */
export function makeRig(opts = {}) {
    const QC = loadModules();
    const cfg = QC.config;

    const drone = QC.state.makeDroneState();
    const remote = QC.state.makeRemoteState();
    const dyn = QC.dynamics.makeDynamics();

    /* 让遥控器"站"在起降场旁边, 链路才有距离 */
    const remotePos = { x: 6, y: -6, z: 1.6 };

    QC.dynamics.reset(dyn, 0, 0, opts.startZ === undefined ? 0 : opts.startZ);
    if (opts.initialTiltPitchDeg || opts.initialTiltRollDeg) {
        dyn.quat = QC.math.qFromEuler(
            (opts.initialTiltPitchDeg || 0) * Math.PI / 180,
            (opts.initialTiltRollDeg || 0) * Math.PI / 180,
            0);
    }

    /* 复位姿态解算器的 static 状态。
       Com_IMU.c 里的四元数与积分补偿量是函数内 static 的, 不清零的话
       两次实验之间会互相污染(整定脚本里就会表现为结果不可复现)。 */
    QC.imu.Reset();

    const sensors = QC.sensors.makeSensors(opts.seed === undefined ? 12345 : opts.seed,
        QC.layout, () => ({
            position: dyn.position,
            velocity: dyn.velocity,
            quat: dyn.quat,
            omega: dyn.omega,
            accel: dyn.accelWorld
        }));

    if (opts.calibrate !== false) {
        sensors.calibrate();
    }

    const sc = QC.tasks.makeScheduler(0.5);
    sc.link = QC.comm.LinkModel();

    const rng = QC.math.Mulberry32(opts.linkSeed === undefined ? 999 : opts.linkSeed);

    /* 摇杆的直接输入(0~1000, 已经是"固件里的通道值"): 测试用, 绕开键盘 */
    const sticks = { THR: 0, YAW: 500, PIT: 500, ROL: 500 };

    /*
     * useInput=true 时改走真实的输入链路:
     *   input.axes -> config.STICK_POLARITY 极性映射 -> remote.joyStick -> 空口 -> 飞控
     * 用来回归"推杆方向是否等于飞行方向"这类问题。
     */
    const useInput = !!opts.useInput;
    const input = useInput ? QC.input.makeInput() : null;

    function applySticks() {
        if (useInput) {
            input.update(0.5);
            input.applyToRemote(remote, 0.5);
            return;
        }
        remote.joyStick.THR = sticks.THR;
        remote.joyStick.YAW = sticks.YAW;
        remote.joyStick.PIT = sticks.PIT;
        remote.joyStick.ROL = sticks.ROL;
    }

    let simMs = 0;

    /** 物理推进 dtMs 毫秒(内部 0.5ms 子步, 任务按各自周期触发) */
    function step(dtMs) {
        const sub = 0.5;
        let remain = dtMs;
        while (remain > 1e-9) {
            const dt = Math.min(sub, remain);
            remain -= dt;
            simMs += dt;

            applySticks();
            QC.tasks.advance(sc, dt, drone, remote, sensors, dyn.position, remotePos, rng);

            QC.dynamics.readMotorCommands(dyn, drone.motors);
            QC.dynamics.step(dyn, dt / 1000, QC.layout);

            /* 撞到楼/桥塔 -> 坠机 */
            const hit = QC.layout.hitTest(dyn.position.x, dyn.position.y, dyn.position.z);
            if (hit && !dyn.crashed) {
                dyn.crashed = true;
                dyn.crashReason = '撞上' + hit.name;
                drone.crashed = true;
            }
        }
    }

    /** 解锁: 油门最高保持 1.2s -> 最低保持 1.2s (与固件状态机一致) */
    function arm() {
        sticks.THR = 1000;
        step(1300);
        sticks.THR = 0;
        step(1300);
        return drone.isRemoteUnlocked === QC.comm.Com_OK;
    }

    /** 悬停所需的油门通道值(固件里电机基础转速 = THR * THROTTLE_GAIN) */
    function hoverThrottle() {
        return QC.dynamics.hoverSpeed() / cfg.THROTTLE_GAIN;
    }

    return {
        QC, cfg, drone, remote, dyn, sensors, sc, sticks, remotePos, input,
        step, arm, hoverThrottle,
        get simMs() { return simMs; },
        get unlocked() { return drone.isRemoteUnlocked === QC.comm.Com_OK; },
        get connected() { return drone.isRemoteConnected === QC.comm.Com_OK; },
        /* 真实欧拉角(由物理四元数算出), 用于与解算结果对照 */
        trueAttitude() {
            const R = QC.math.qToMatrix(dyn.quat);
            return {
                /* 与固件同一套反解公式, 便于直接比对 */
                pitchFw: Math.asin(QC.math.clamp(-R[2][0], -1, 1)) * cfg.RAD_TO_DEG,
                rollFw: Math.atan2(R[2][1], R[2][2]) * cfg.RAD_TO_DEG,
                rollStd: Math.atan2(R[2][1], R[2][2]) * cfg.RAD_TO_DEG
            };
        }
    };
}
