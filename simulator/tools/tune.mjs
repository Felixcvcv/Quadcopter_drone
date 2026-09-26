/*
 * tools/tune.mjs — 物理参数整定 (开发时用)
 *
 * 为什么需要它:
 *   固件里那组 PID 增益是按真实飞机整定的, 而仿真器的物理参数(质量/转动惯量/
 *   推力系数/电机时间常数/转动阻尼)是仿真器自己引入的。如果这些参数取得不合
 *   适, 同一组增益在仿真里就会振荡甚至发散 —— 那是模型的问题, 不是固件的问题。
 *   这个脚本在物理参数空间里搜一组"让固件增益表现良好"的取值。
 *
 * 评价指标(越小越好): 三个轴各施加一次角速度扰动, 取最差的一个:
 *   - 收敛时间(角速度回到 0.12rad/s 且角度回到 2.5 度以内)
 *   - 扰动峰值超调
 *   - 收敛后的残余角速度与残余角度
 *   - 悬停时的高度漂移
 *   发散 / 撞地 / NaN 直接给极高惩罚分。
 *
 * 用法: node simulator/tools/tune.mjs            粗搜 + 细化
 *       node simulator/tools/tune.mjs --quick    只粗搜几组
 */
import { makeRig, loadModules } from './harness.mjs';

const QC = loadModules();
const D = QC.config.DRONE;
const BASE = JSON.parse(JSON.stringify(D));

const AXES = [
    { key: 'roll', set: (r, v) => { r.dyn.omega.x = v; }, rate: r => r.dyn.omega.x, angle: r => r.drone.eulerAngle.roll },
    { key: 'pitch', set: (r, v) => { r.dyn.omega.y = v; }, rate: r => r.dyn.omega.y, angle: r => r.drone.eulerAngle.pitch },
    { key: 'yaw', set: (r, v) => { r.dyn.omega.z = v; }, rate: r => r.dyn.omega.z, angle: r => r.drone.eulerAngle.yaw }
];

/** 用一组物理参数跑一次"悬停 + 三轴扰动", 返回最差轴的指标 */
function evaluate(cand) {
    Object.assign(D, BASE, cand);
    D.inertia = {
        x: BASE.inertia.x * cand.inertiaScale,
        y: BASE.inertia.y * cand.inertiaScale,
        z: BASE.inertia.z * cand.inertiaScale
    };

    const out = { worst: null, axes: {} };

    for (const ax of AXES) {
        const rig = makeRig({ calibrate: true });
        const hover = rig.hoverThrottle();
        if (!(hover > 100 && hover < 950)) return { score: 1e6, note: 'hover out of range', hover };

        if (!rig.arm()) return { score: 1e6, note: 'arm failed' };
        rig.sticks.THR = hover;
        rig.step(1200);

        const zBefore = rig.dyn.position.z;
        ax.set(rig, 3.0);

        let peakAngle = 0, settle = -1;
        const t0 = rig.simMs;
        for (let i = 0; i < 800; i++) {          /* 最多 4 秒 */
            rig.step(5);
            const rate = Math.abs(ax.rate(rig));
            const angle = Math.abs(ax.angle(rig));
            if (!isFinite(rate) || !isFinite(rig.dyn.position.z)) {
                return { score: 1e6, note: ax.key + ' NaN' };
            }
            peakAngle = Math.max(peakAngle, angle);
            if (settle < 0 && rate < 0.12 && angle < 2.5) settle = (rig.simMs - t0) / 1000;
        }

        const rateEnd = Math.abs(ax.rate(rig));
        const angleEnd = Math.abs(ax.angle(rig));
        const dz = Math.abs(rig.dyn.position.z - zBefore);
        if (rig.dyn.crashed) return { score: 1e6, note: ax.key + ' crashed' };

        let score = (settle < 0) ? 60 : settle;
        score += Math.min(peakAngle, 200) / 40;
        score += rateEnd * 20;
        score += angleEnd * 2;
        score += Math.max(0, dz - 1.0) * 4;

        /*
         * 可用性约束: 悬停油门要落在遥控器中段附近, 否则一只手杆几乎没有
         * 爬升余量, 演示时很难飞。真实的微型四轴也大多悬停在 50%~70% 行程。
         */
        const hoverThr = hover;
        if (hoverThr < 430) score += (430 - hoverThr) * 0.02;
        if (hoverThr > 700) score += (hoverThr - 700) * 0.02;

        out.axes[ax.key] = { score, settle, peakAngle, rateEnd, angleEnd, dz, hover };
        if (!out.worst || score > out.worst.score) out.worst = { key: ax.key, ...out.axes[ax.key] };
    }

    out.score = out.worst.score;
    return out;
}

function search(grid, label) {
    let best = null;
    let tried = 0;
    for (const angularDrag of grid.angularDrag) {
        for (const motorTimeConstant of grid.motorTimeConstant) {
            for (const hoverSpeedTarget of grid.hoverSpeedTarget) {
                for (const inertiaScale of grid.inertiaScale) {
                    const kT = (BASE.mass * 9.80665) / (4 * hoverSpeedTarget * hoverSpeedTarget);
                    const r = evaluate({ angularDrag, motorTimeConstant, inertiaScale,
                                         thrustCoef: kT, hoverSpeedTarget });
                    tried++;
                    if (!best || r.score < best.r.score) {
                        best = { cand: { angularDrag, motorTimeConstant, inertiaScale,
                                         thrustCoef: kT, hoverSpeedTarget }, r };
                        const w = r.worst;
                        console.log(`  [${label}] score=${r.score.toFixed(2)}  ` +
                            `drag=${angularDrag.toExponential(1)} tau=${motorTimeConstant} ` +
                            `hover=${hoverSpeedTarget} Iscale=${inertiaScale}` +
                            (w ? `  最差轴=${w.key} settle=${w.settle.toFixed(2)}s peak=${w.peakAngle.toFixed(1)}deg` : `  ${r.note}`));
                    }
                }
            }
        }
    }
    console.log(`  试了 ${tried} 组`);
    return best;
}

/* ---------------- 第一阶段: 粗搜 ---------------- */
console.log('第一阶段: 粗搜');
const coarse = search({
    angularDrag: [1.2e-4, 1.8e-4, 2.4e-4, 3.2e-4, 4.2e-4],
    motorTimeConstant: [0.016, 0.022, 0.03, 0.042],
    hoverSpeedTarget: [300, 340, 390, 440],
    inertiaScale: [1.2, 1.6, 2.0, 2.6]
}, 'coarse');

/* ---------------- 第二阶段: 局部细化 ---------------- */
const c = coarse.cand;
console.log('\n第二阶段: 在最优解附近细化');
const fine = search({
    angularDrag: [c.angularDrag * 0.6, c.angularDrag * 0.8, c.angularDrag, c.angularDrag * 1.3, c.angularDrag * 1.7],
    motorTimeConstant: [0.012, 0.016, c.motorTimeConstant, c.motorTimeConstant * 1.4, c.motorTimeConstant * 1.9],
    hoverSpeedTarget: [c.hoverSpeedTarget * 0.82, c.hoverSpeedTarget * 0.92, c.hoverSpeedTarget,
                       c.hoverSpeedTarget * 1.12, c.hoverSpeedTarget * 1.25],
    inertiaScale: [c.inertiaScale * 0.75, c.inertiaScale * 0.9, c.inertiaScale,
                   c.inertiaScale * 1.15, c.inertiaScale * 1.35]
}, 'fine');

/* ---------------- 结果 ---------------- */
const win = fine.r.score <= coarse.r.score ? fine : coarse;
const base = BASE;
const winHoverSpeed = Math.sqrt((base.mass * 9.80665) / (4 * win.cand.thrustCoef));
console.log('\n================ 最终参数 ================');
console.log(`angularDrag        = ${win.cand.angularDrag.toExponential(2)}   (原 ${base.angularDrag.toExponential(2)})`);
console.log(`motorTimeConstant  = ${win.cand.motorTimeConstant}            (原 ${base.motorTimeConstant})`);
console.log(`thrustCoef         = ${win.cand.thrustCoef.toExponential(4)}  (原 ${base.thrustCoef.toExponential(4)})`);
console.log(`  -> 悬停转速 ${winHoverSpeed.toFixed(0)} (对应油门 ${(winHoverSpeed / QC.config.THROTTLE_GAIN).toFixed(0)})`);
console.log(`  -> 满油门推重比 ${(4 * win.cand.thrustCoef * 1e6 / (base.mass * 9.80665)).toFixed(2)}`);
console.log(`inertia.x/y        = ${(base.inertia.x * win.cand.inertiaScale).toExponential(3)} = 基准 x ${win.cand.inertiaScale}`);
console.log(`inertia.z          = ${(base.inertia.z * win.cand.inertiaScale).toExponential(3)}`);
console.log('\n各轴表现:');
for (const k of ['roll', 'pitch', 'yaw']) {
    const a = win.r.axes[k];
    if (!a) continue;
    console.log(`  ${k.padEnd(6)} 收敛 ${a.settle.toFixed(2)}s  峰值 ${a.peakAngle.toFixed(1)}deg  ` +
        `残余速率 ${a.rateEnd.toFixed(4)}rad/s  残余角度 ${a.angleEnd.toFixed(3)}deg  高度漂移 ${a.dz.toFixed(2)}m`);
}
