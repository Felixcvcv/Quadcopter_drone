/*
 * fw/pid.js — Com_PID.c 的移植
 *
 * 与固件保持一致的地方:
 *   - 误差定义为 (测量值 - 期望值), 所以比例系数为负表示"方向正确"
 *   - isAngle 时误差折算到 (-180, 180], 走最短角度路径
 *   - 积分限幅 / 输出限幅: 上限 <= 下限时不启用
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});

    function PID_Struct(cfg) {
        cfg = cfg || {};
        return {
            kp: cfg.kp || 0,
            ki: cfg.ki || 0,
            kd: cfg.kd || 0,

            dt: 0,
            integral: 0,
            lastError: 0,
            desire: 0,
            measure: 0,
            result: 0,

            outMin: (cfg.outMin === undefined) ? 0 : cfg.outMin,
            outMax: (cfg.outMax === undefined) ? 0 : cfg.outMax,
            integralMin: (cfg.integralMin === undefined) ? 0 : cfg.integralMin,
            integralMax: (cfg.integralMax === undefined) ? 0 : cfg.integralMax,

            isAngle: cfg.isAngle ? 1 : 0
        };
    }

    /* 取 x 在 [min, max] 之间的限幅值 (对应 Com_Config.h 的 LIMIT 宏) */
    function LIMIT(x, min, max) {
        return (x >= max) ? max : ((x <= min) ? min : x);
    }

    /* 对应 Com_PID.c 的 Com_PID_WrapAngle180 */
    function WrapAngle180(deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }

    /* 对应 Com_PID_Reset */
    function Reset(pid) {
        pid.integral = 0.0;
        pid.lastError = 0.0;
        pid.result = 0.0;
    }

    /* 对应 Com_PID_ComputePID */
    function ComputePID(pid) {
        /* 0. 误差 */
        var error = pid.measure - pid.desire;
        if (pid.isAngle) {
            error = WrapAngle180(error);
        }

        /* 1. 比例项 */
        var pV = pid.kp * error;

        /* 2. 积分项(带限幅, 抑制积分饱和) */
        pid.integral += pid.ki * error * pid.dt;
        if (pid.integralMax > pid.integralMin) {
            pid.integral = LIMIT(pid.integral, pid.integralMin, pid.integralMax);
        }

        /* 3. 微分项 */
        var dV = pid.kd * (error - pid.lastError) / pid.dt;
        pid.lastError = error;

        /* 4. 输出 */
        pid.result = pV + pid.integral + dV;

        /* 5. 输出限幅 */
        if (pid.outMax > pid.outMin) {
            pid.result = LIMIT(pid.result, pid.outMin, pid.outMax);
        }
    }

    /* 对应 Com_PID_CascadePID: 外环输出作为内环期望值 */
    function CascadePID(out, inner) {
        ComputePID(out);
        inner.desire = out.result;
        ComputePID(inner);
    }

    QC.pid = {
        PID_Struct: PID_Struct,
        LIMIT: LIMIT,
        WrapAngle180: WrapAngle180,
        Reset: Reset,
        ComputePID: ComputePID,
        CascadePID: CascadePID
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
