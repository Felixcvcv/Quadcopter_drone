/*
 * tools/selftest.mjs — 无头自检
 *
 * 验证的是"固件算法 + 物理模型"整体行为, 而不是某个函数返回值:
 *   1. 快速平方根倒数的精度与固件注释一致
 *   2. 遥测帧的校验和能挡住被改坏的帧
 *   3. AHRS 的姿态估计能从带零偏的原始数据收敛到真值
 *   4. 三个姿态串级环都是负反馈(受扰动后能回到水平)
 *   5. 解锁状态机需要完整的"油门高-保持-油门低-保持"序列
 *   6. 断链会触发自动上锁并退出定高
 *   7. 定高环能把飞机稳定在目标高度附近
 *   8. 源码里引用的配置常量都真实存在(引用不存在的常量只会算出 NaN, 不会报错)
 *   9. 电机输出始终被限幅在 0~1000
 *  10. 推杆方向 = 屏幕上的飞行方向(走真实输入链路: 摇杆 -> 极性 -> 空口 -> 飞控)
 *
 * 运行: node simulator/tools/selftest.mjs
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { makeRig, loadModules, LOAD_ORDER } from './harness.mjs';

const QC = loadModules();

let pass = 0, fail = 0;
const failures = [];

function check(name, ok, detail) {
    if (ok) {
        pass++;
        console.log(`  \u2713 ${name}` + (detail ? `   ${detail}` : ''));
    } else {
        fail++;
        failures.push(name + (detail ? `  (${detail})` : ''));
        console.log(`  \u2717 ${name}` + (detail ? `   ${detail}` : ''));
    }
}

function section(t) { console.log(`\n=== ${t} ===`); }

/* ------------------------------------------------------------------ */
section('1. 快速平方根倒数');

{
    let worst = 0;
    for (const x of [0.25, 1, 2, 9.81, 16384, 12345.6, 1e6]) {
        const got = QC.imu.Q_rsqrt(x);
        const want = 1 / Math.sqrt(x);
        worst = Math.max(worst, Math.abs(got - want) / want);
    }
    check('相对误差 < 1e-4', worst < 1e-4, `最差 ${worst.toExponential(2)}`);
    /*
     * 与固件的对照: 固件注释说"两次牛顿迭代后相对误差约 5e-6"。
     * 这里顺带记录 1 次迭代的误差, 说明为什么值得多迭代一次。
     */
    const rng = QC.math.Mulberry32(7);
    let w1 = 0, w2 = 0;
    const f32 = new Float32Array(1), u32 = new Uint32Array(f32.buffer);
    for (let i = 0; i < 2000; i++) {
        const x = 0.5 + rng() * 20000;
        f32[0] = x; u32[0] = (0x5f3759df - (u32[0] >>> 1)) >>> 0;
        let y = f32[0];
        const x2 = x * 0.5;
        const one = y * (1.5 - x2 * y * y);
        const two = one * (1.5 - x2 * one * one);
        const want = 1 / Math.sqrt(x);
        w1 = Math.max(w1, Math.abs(one - want) / want);
        w2 = Math.max(w2, Math.abs(two - want) / want);
    }
    check('两次迭代明显优于一次', w2 < w1 / 50,
        `1次 ${w1.toExponential(2)} -> 2次 ${w2.toExponential(2)}`);
}

/* ------------------------------------------------------------------ */
section('2. 2.4G 帧的三级校验');

{
    const remote = QC.state.makeRemoteState();
    remote.joyStick.THR = 512;
    remote.joyStick.YAW = 500;
    remote.joyStick.PIT = 620;
    remote.joyStick.ROL = 380;
    remote.joyStick.isPowerDown = 1;
    remote.joyStick.isFixHeight = 1;

    const frame = QC.comm.BuildFrame(remote);
    check('帧长 18 字节', frame.length === 18, `实际 ${frame.length}`);
    check('帧头 0x11 0x22 0x33', frame[0] === 0x11 && frame[1] === 0x22 && frame[2] === 0x33);
    check('载荷长度字段 = 10', frame[3] === 10, `实际 ${frame[3]}`);
    check('单次触发字段已清零',
        remote.joyStick.isPowerDown === 0 && remote.joyStick.isFixHeight === 0);

    const drone = QC.state.makeDroneState();
    check('原帧能被接收', QC.comm.ReceiveFrame(drone, frame) === QC.comm.Com_OK);
    check('解析出的油门正确', drone.joyStick.THR === 512, `实际 ${drone.joyStick.THR}`);
    check('解析出的俯仰正确', drone.joyStick.PIT === 620, `实际 ${drone.joyStick.PIT}`);
    check('定高切换被置位(收到1取反)', drone.joyStick.isFixHeight === 1);
    check('定高状态同步为 Com_OK', drone.isFixHeight === QC.comm.Com_OK);

    /* 帧头被破坏 */
    let bad = frame.slice(); bad[1] = 0x99;
    check('坏帧头被拒', QC.comm.ReceiveFrame(QC.state.makeDroneState(), bad) === QC.comm.Com_FAIL);

    /* 载荷长度被破坏 */
    bad = frame.slice(); bad[3] = 9;
    check('错长度被拒', QC.comm.ReceiveFrame(QC.state.makeDroneState(), bad) === QC.comm.Com_FAIL);

    /* 载荷单比特翻转 */
    let rejected = 0;
    for (let bit = 0; bit < 8; bit++) {
        bad = frame.slice();
        bad[7] ^= (1 << bit);
        if (QC.comm.ReceiveFrame(QC.state.makeDroneState(), bad) === QC.comm.Com_FAIL) rejected++;
    }
    check('载荷单比特翻转全部被校验和拦住', rejected === 8, `8 个里拦住 ${rejected} 个`);
}

/* ------------------------------------------------------------------ */
section('3. AHRS 姿态估计收敛 (带陀螺仪零偏)');

{
    /*
     * 让飞机保持一个固定倾角, 看解算结果是否收敛到真值。
     *
     * 注意顺序: 固件的加速度计零偏校准**要求飞机水平静止**(见
     * Inf_MPU6050_Calibrate 的注释), 所以必须先在校平状态下完成标定,
     * 再把飞机摆到倾斜姿态。反过来的话, 零偏会把倾角当成零点。
     */
    const rig = makeRig({ calibrate: true });
    /* 标定完成后再摆成 12 度俯仰 / -8 度横滚 */
    rig.dyn.quat = QC.math.qFromEuler(12 * Math.PI / 180, -8 * Math.PI / 180, 0);
    QC.imu.Reset();
    rig.dyn.omega.x = rig.dyn.omega.y = rig.dyn.omega.z = 0;
    rig.dyn.accelWorld.x = rig.dyn.accelWorld.y = rig.dyn.accelWorld.z = 0;

    const holdQuat = rig.dyn.quat;
    for (let i = 0; i < 4000; i++) {
        /* 强行保持姿态静止, 只让传感器与解算器工作 */
        rig.dyn.quat = holdQuat;
        rig.dyn.omega.x = rig.dyn.omega.y = rig.dyn.omega.z = 0;
        rig.dyn.accelWorld.x = rig.dyn.accelWorld.y = rig.dyn.accelWorld.z = 0;

        QC.flight.FlightTaskStep(rig.drone, rig.sensors, 0.004);
        rig.dyn.quat = holdQuat;
    }

    const truth = rig.trueAttitude();
    const est = rig.drone.eulerAngle;
    const dp = Math.abs(est.pitch - truth.pitchFw);
    const dr = Math.abs(est.roll - truth.rollFw);

    console.log(`      真实 pitch=${truth.pitchFw.toFixed(2)}  估计=${est.pitch.toFixed(2)}` +
        `   真实 roll=${truth.rollFw.toFixed(2)}  估计=${est.roll.toFixed(2)}`);
    check('俯仰角估计误差 < 2 度', dp < 2, `误差 ${dp.toFixed(2)} 度`);
    check('横滚角估计误差 < 2 度', dr < 2, `误差 ${dr.toFixed(2)} 度`);
}

/* ------------------------------------------------------------------ */
section('4. 三个姿态环都是负反馈 (受扰后回水平)');

{
    const axes = [
        { name: '俯仰', set: (r, v) => { r.dyn.omega.y = v; } },
        { name: '横滚', set: (r, v) => { r.dyn.omega.x = v; } },
        { name: '偏航', set: (r, v) => { r.dyn.omega.z = v; } }
    ];

    for (const ax of axes) {
        const rig = makeRig({ startZ: 0, calibrate: true });
        const hover = rig.hoverThrottle();

        /* 让它先飞到空中并稳定 */
        const armed = rig.arm();
        if (!armed) { check(`${ax.name}: 解锁成功`, false); continue; }
        rig.sticks.THR = hover;
        rig.step(1500);

        /* 施加一个角速度扰动 */
        ax.set(rig, 4.0);
        rig.step(60);

        const peak = ax.name === '偏航'
            ? Math.abs(rig.dyn.omega.z)
            : Math.abs(ax.name === '俯仰' ? rig.dyn.omega.y : rig.dyn.omega.x);

        /* 观察 3 秒后是否收敛 */
        rig.step(3000);
        const resid = ax.name === '偏航'
            ? Math.abs(rig.dyn.omega.z)
            : Math.abs(ax.name === '俯仰' ? rig.dyn.omega.y : rig.dyn.omega.x);

        check(`${ax.name}环受扰后收敛`, resid < 0.5,
            `峰值 ${peak.toFixed(2)} -> 残余 ${resid.toFixed(3)} rad/s`);
    }
}

/* ------------------------------------------------------------------ */
section('5. 解锁状态机');

{
    const rig = makeRig({ calibrate: false });
    check('上电时未解锁', !rig.unlocked);
    check('上电时未连接', !rig.connected, '需要收到数据后才算连接');

    /* 只把油门推到最高, 但保持时间不够 */
    rig.sticks.THR = 1000;
    rig.step(600);
    rig.sticks.THR = 0;
    rig.step(1200);
    check('油门高/低各保持不足 1.2s 时不解锁', !rig.unlocked);

    /* 完整序列 */
    const ok = rig.arm();
    check('完整序列后解锁成功', ok);
    check('解锁后链路为已连接', rig.connected);

    /* 解锁后油门停在最低 60s 会自动上锁 */
    rig.sticks.THR = 0;
    rig.step(61000);
    check('油门停在最低 60s 后自动上锁', !rig.unlocked);
}

/* ------------------------------------------------------------------ */
section('6. 断链保护');

{
    const rig = makeRig({ startZ: 0, calibrate: true });
    const hover = rig.hoverThrottle();
    rig.arm();
    rig.sticks.THR = hover;
    rig.step(1200);
    check('飞行中处于解锁状态', rig.unlocked);

    /* 打开干扰, 模拟链路中断 */
    rig.sc.link.jammed = true;
    rig.step(200);
    check('断链 0.2s 内仍判为已连接(1.2s 门限)', rig.connected);

    rig.step(1400);
    check('断链超过 1.2s 判为失联', !rig.connected);
    check('失联后自动上锁', !rig.unlocked);
    check('失联后退出定高', rig.drone.isFixHeight !== QC.comm.Com_OK);

    rig.sc.link.jammed = false;
    rig.step(200);
    check('干扰解除后恢复连接', rig.connected);
}

/* ------------------------------------------------------------------ */
section('7. 定高环');

{
    const rig = makeRig({ startZ: 0, calibrate: true });
    const hover = rig.hoverThrottle();
    rig.arm();
    /* 先用略高于悬停的油门爬到空中, 再做定高 */
    rig.sticks.THR = hover + 120;
    rig.step(3500);
    const z0 = rig.dyn.position.z;
    check('先爬升到空中', z0 > 5, `高度 ${z0.toFixed(2)} m`);

    /* 把油门收回到悬停附近再打开定高(推杆不动时定高才有意义) */
    rig.sticks.THR = hover;
    rig.step(300);

    /* 打开定高 */
    rig.remote.joyStick.isFixHeight = 1;
    rig.step(400);
    /* 定高的目标是"进入时的激光测距值", 所以用测距值比对 */
    const targetMm = rig.drone.alt.heightHold;
    const tofAtHold = rig.sensors.state.tof.lastMm;
    console.log(`      进入定高: 世界高度 ${rig.dyn.position.z.toFixed(2)} m, ` +
        `测距 ${tofAtHold} mm, 目标 ${targetMm} mm`);

    rig.step(4000);
    const tof1 = rig.sensors.state.tof.lastMm;
    const err = Math.abs(tof1 - targetMm) / 1000;

    check('定高后高度漂移 < 1.5 m', err < 1.5,
        `4 秒后测距 ${tof1} mm, 偏差 ${err.toFixed(2)} m`);

    /* 施加一个向下扰动, 看是否能拉回来 */
    rig.dyn.velocity.z -= 2.5;
    rig.step(500);
    const tofAfterKick = rig.sensors.state.tof.lastMm;
    rig.step(3000);
    const tof2 = rig.sensors.state.tof.lastMm;
    const err2 = Math.abs(tof2 - targetMm) / 1000;
    check('受向下扰动后仍能拉回目标高度 < 2 m', err2 < 2.0,
        `扰动后最低 ${tofAfterKick} mm -> 3 秒后 ${tof2} mm, 偏差 ${err2.toFixed(2)} m`);

    /* 油门大幅变化应退出定高 */
    rig.sticks.THR = hover + 120;
    rig.step(100);
    check('油门突变后退出定高', rig.drone.alt.status === 0,
        `状态机 status=${rig.drone.alt.status}`);
}

/* ------------------------------------------------------------------ */
section('8. 未解锁时电机必须停转');

{
    const rig = makeRig({ calibrate: false });
    rig.sticks.THR = 700;   /* 油门给足, 但不解锁 */
    rig.step(500);
    const m = rig.drone.motors;
    const allZero = m.leftTop.speed === 0 && m.leftBottom.speed === 0 &&
        m.rightTop.speed === 0 && m.rightBottom.speed === 0;
    check('未解锁时 4 个电机全为 0', allZero,
        `转速 ${m.leftTop.speed}/${m.leftBottom.speed}/${m.rightTop.speed}/${m.rightBottom.speed}`);
    check('未解锁时飞机没有离地', rig.dyn.position.z < 0.1, `z=${rig.dyn.position.z.toFixed(3)}`);
}

/* ------------------------------------------------------------------ */
section('9. 源码里引用的配置常量都真实存在');

{
    /*
     * 这条检查是拿教训换来的:
     * 曾经在 config.js 里漏加 MOTOR_MAX, 而 main.js / hud.js 里照常写
     * `cfg.MOTOR_MAX`, 于是 `0 / undefined = NaN` —— 螺旋桨的旋转角被写成 NaN
     * 后再也没转回来(桨叶一直不显示), HUD 的电机条也永远停在 0 宽。
     * 这类"引用了不存在的常量"不报错, 只会算出 NaN, 所以必须静态查出来。
     */
    const JS_DIR = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'js');
    const cfgNames = new Set(Object.keys(QC.config));
    const stateNames = new Set(Object.keys(QC.state));
    const problems = [];
    let scanned = 0;

    const RE_CFG = new RegExp('\bcfg\.([A-Za-z_][A-Za-z0-9_]*)', 'g');
    const RE_QCFG = new RegExp('\bQC\.config\.([A-Za-z_][A-Za-z0-9_]*)', 'g');
    const RE_QSTATE = new RegExp('\bQC\.state\.([A-Za-z_][A-Za-z0-9_]*)', 'g');

    function scan(dir) {
        for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
            const f = path.join(dir, e.name);
            if (e.isDirectory()) { scan(f); continue; }
            if (!f.endsWith('.js')) continue;
            scanned++;
            const src = fs.readFileSync(f, 'utf8');
            const rel = path.relative(JS_DIR, f);
            const look = (re, names, what) => {
                re.lastIndex = 0;
                let m;
                while ((m = re.exec(src)) !== null) {
                    if (!names.has(m[1])) problems.push(rel + ': ' + what + '.' + m[1] + ' 不存在');
                }
            };
            look(RE_CFG, cfgNames, 'config');
            look(RE_QCFG, cfgNames, 'config');
            look(RE_QSTATE, stateNames, 'state');
        }
    }
    scan(JS_DIR);

    check('没有引用不存在的配置常量', problems.length === 0,
        problems.length ? problems.slice(0, 6).join(' | ') : '扫描了 ' + scanned + ' 个模块');
    check('config.MOTOR_MAX = 1000', QC.config.MOTOR_MAX === 1000,
        '实际 ' + QC.config.MOTOR_MAX);
    check('state.MOTOR_MAX 与 config 一致',
        QC.state.MOTOR_MAX === QC.config.MOTOR_MAX &&
        QC.state.MOTOR_STOP === QC.config.MOTOR_STOP);
}

/* ------------------------------------------------------------------ */
section('10. 电机输出始终被限幅在 0~1000');

{
    const rig = makeRig({ calibrate: true });
    const hover = rig.hoverThrottle();
    rig.arm();
    rig.sticks.THR = hover + 260;
    rig.step(3000);
    rig.remote.joyStick.isFixHeight = 1;
    rig.step(500);
    rig.dyn.velocity.z = -6;        /* 猛地下沉, 定高环会把 zPid 打到限幅 */
    rig.step(2000);
    rig.sticks.THR = hover - 300;   /* 再猛收油门 */

    let minS = 1e9, maxS = -1e9;
    const mk = ['leftTop', 'leftBottom', 'rightTop', 'rightBottom'];
    for (let i = 0; i < 400; i++) {
        rig.step(5);
        for (const k of mk) {
            const v = rig.drone.motors[k].speed;
            minS = Math.min(minS, v);
            maxS = Math.max(maxS, v);
        }
    }
    check('所有电机值都在 0~1000 之间', minS >= 0 && maxS <= 1000,
        '实测范围 ' + minS + ' ~ ' + maxS);
}

/* ------------------------------------------------------------------ */
section('11. 推杆方向 = 屏幕上的飞行方向');

{
    /*
     * 这条是用户实测反馈换来的: 曾经按左键飞机在屏幕上往右飞。
     * 根因不在映射, 而在坐标手性 —— 轴向标成 (X=北, Y=东, Z=上) 在 Z 朝上时
     * 是左手系, 而 three.js 与物理用标准右手代数, 画面因此左右镜像;
     * 俯仰正好落在镜像对称轴上所以正常, 横滚与偏航都反了。
     *
     * 结论: 相机在机尾后方时, 屏幕右方向对应世界 -Y。所以断言写成:
     *   推杆向右 -> 位移的世界 Y 分量 < 0(即屏幕向右飞)
     *   推杆向前 -> 位移的世界 X 分量 > 0(即朝屏幕里飞)
     *   偏航向右 -> 偏航角估计减小(机头转向世界 -Y = 屏幕右)
     *
     * 这里走的是完整输入链路: input.axes -> config.STICK_POLARITY -> 18字节帧
     * -> 飞控解析 -> 姿态环 -> 物理, 所以极性配错一定会被抓到。
     */
    function flyWith(axes, ms) {
        const rig = makeRig({ calibrate: true, useInput: true });
        /*
         * 注意: useInput 时通道值是 input.axes 经极性映射来的, rig.sticks 不起作用,
         * 所以解锁序列也必须走输入链路(油门极性为 +1, 与方向无关)。
         */
        const I = rig.input;
        I.pinAxis('THR', 1000); rig.step(1300);
        I.pinAxis('THR', 0); rig.step(1300);
        const armed = rig.drone.isRemoteUnlocked === 0;

        /* 摆到原点、水平、朝 +X, 然后改用输入链路推杆 */
        rig.dyn.position.x = 0; rig.dyn.position.y = 0; rig.dyn.position.z = 30;
        rig.dyn.velocity.x = rig.dyn.velocity.y = rig.dyn.velocity.z = 0;
        rig.dyn.quat = rig.QC.math.qFromEuler(0, 0, 0);
        rig.dyn.omega.x = rig.dyn.omega.y = rig.dyn.omega.z = 0;
        rig.step(400);

        const p0 = { x: rig.dyn.position.x, y: rig.dyn.position.y, z: rig.dyn.position.z };
        const yaw0 = rig.drone.eulerAngle.yaw;
        Object.keys(axes).forEach(k => rig.input.pinAxis(k, axes[k]));
        rig.step(ms);
        const yaw1 = rig.drone.eulerAngle.yaw;
        const chan = { PIT: rig.remote.joyStick.PIT, ROL: rig.remote.joyStick.ROL,
                       YAW: rig.remote.joyStick.YAW };
        return {
            armed, chan,
            dx: rig.dyn.position.x - p0.x,
            dy: rig.dyn.position.y - p0.y,
            dYaw: yaw1 - yaw0,
            crashed: rig.dyn.crashed
        };
    }

    const right = flyWith({ THR: 600, ROL: 1000 }, 2500);
    check('摇杆向右 -> 屏幕向右飞(世界 Y 为负)', right.armed && right.dy < -3,
        `解锁=${right.armed}, 通道 ROL=${right.chan.ROL}, 位移 Y=${right.dy.toFixed(2)}m`);

    const left = flyWith({ THR: 600, ROL: 0 }, 2500);
    check('摇杆向左 -> 屏幕向左飞(世界 Y 为正)', left.armed && left.dy > 3,
        `解锁=${left.armed}, 通道 ROL=${left.chan.ROL}, 位移 Y=${left.dy.toFixed(2)}m`);

    const fwd = flyWith({ THR: 600, PIT: 1000 }, 2500);
    check('摇杆向前 -> 朝屏幕里飞(世界 X 为正)', fwd.armed && fwd.dx > 3,
        `解锁=${fwd.armed}, 通道 PIT=${fwd.chan.PIT}, 位移 X=${fwd.dx.toFixed(2)}m`);

    const back = flyWith({ THR: 600, PIT: 0 }, 2500);
    check('摇杆向后 -> 朝相机方向退(世界 X 为负)', back.armed && back.dx < -3,
        `解锁=${back.armed}, 通道 PIT=${back.chan.PIT}, 位移 X=${back.dx.toFixed(2)}m`);

    const yawR = flyWith({ THR: 600, YAW: 1000 }, 1800);
    check('偏航向右 -> 机头转向屏幕右', yawR.armed && yawR.dYaw < -8,
        `解锁=${yawR.armed}, 通道 YAW=${yawR.chan.YAW}, 偏航变化 ${yawR.dYaw.toFixed(1)}°`);

    const yawL = flyWith({ THR: 600, YAW: 0 }, 1800);
    check('偏航向左 -> 机头转向屏幕左', yawL.armed && yawL.dYaw > 8,
        `解锁=${yawL.armed}, 通道 YAW=${yawL.chan.YAW}, 偏航变化 ${yawL.dYaw.toFixed(1)}°`);

    /* 顺带确认三个姿态轴都没有发散、没有撞地 */
    check('方向测试期间没有坠机/发散',
        ![right, left, fwd, back, yawR, yawL].some(r => r.crashed));
}

/* ------------------------------------------------------------------ */
console.log(`\n${'='.repeat(56)}`);
console.log(`自检结果: ${pass} 通过, ${fail} 失败`);
if (fail) {
    console.log('失败项:');
    for (const f of failures) console.log('  - ' + f);
    process.exit(1);
}
console.log('全部通过');
