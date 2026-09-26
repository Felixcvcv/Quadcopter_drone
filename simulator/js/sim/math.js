/*
 * sim/math.js — 四元数与向量工具
 *
 * 约定 (与固件一致):
 *   世界系: X 北, Y 东, Z 上   (docx: "正北, 正东, 正上方分别表示X, Y, Z轴")
 *   机体系: X 前, Y 右, Z 上
 *   四元数: Hamilton 约定, q = (q0, q1, q2, q3) = (w, x, y, z), 表示"机体系 -> 世界系"
 *   旋转矩阵 R 用标准公式, 与 Com_IMU.c 里的解算完全对应
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});

    /* ---- 四元数 ---- */
    function qIdentity() { return { q0: 1, q1: 0, q2: 0, q3: 0 }; }

    function qNormalize(q) {
        var n = Math.sqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
        if (n < 1e-12) return qIdentity();
        return { q0: q.q0 / n, q1: q.q1 / n, q2: q.q2 / n, q3: q.q3 / n };
    }

    /* 用机体角速度(rad/s)积分四元数: dq = 0.5 * q ⊗ (0, ω) * dt */
    function qIntegrate(q, w, dt) {
        var q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
        var h = 0.5 * dt;
        return qNormalize({
            q0: q0 + (-q1 * w.x - q2 * w.y - q3 * w.z) * h,
            q1: q1 + (q0 * w.x - q3 * w.y + q2 * w.z) * h,
            q2: q2 + (q3 * w.x + q0 * w.y - q1 * w.z) * h,
            q3: q3 + (-q2 * w.x + q1 * w.y + q0 * w.z) * h
        });
    }

    /* 机体 -> 世界 旋转矩阵 (行主序, 与 Com_IMU.c 的旋转矩阵一致) */
    function qToMatrix(q) {
        var q0 = q.q0, q1 = q.q1, q2 = q.q2, q3 = q.q3;
        return [
            [1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)]
        ];
    }

    /* 世界系向量 转 机体系: v_body = R^T * v_world */
    function worldToBody(R, v) {
        return {
            x: R[0][0] * v.x + R[1][0] * v.y + R[2][0] * v.z,
            y: R[0][1] * v.x + R[1][1] * v.y + R[2][1] * v.z,
            z: R[0][2] * v.x + R[1][2] * v.y + R[2][2] * v.z
        };
    }

    /* 机体系向量 转 世界系: v_world = R * v_body */
    function bodyToWorld(R, v) {
        return {
            x: R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z,
            y: R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z,
            z: R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z
        };
    }

    /* 由欧拉角构造四元数 (仅用于初始化, 例: 让飞机带一点初始倾角) */
    function qFromEuler(pitch, roll, yaw) {
        var cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
        var cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
        var cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
        return qNormalize({
            q0: cy * cp * cr + sy * sp * sr,
            q1: cy * cp * sr - sy * sp * cr,
            q2: sy * cp * sr + cy * sp * cr,
            q3: sy * cp * cr - cy * sp * sr
        });
    }

    function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

    /* 线性同余伪随机数, 让仿真可复现 */
    function Mulberry32(seed) {
        var a = seed >>> 0;
        return function () {
            a = (a + 0x6D2B79F5) >>> 0;
            var t = a;
            t = Math.imul(t ^ (t >>> 15), t | 1);
            t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
            return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
        };
    }

    /* 近似高斯噪声 (12 个均匀分布求和, 比 Box-Muller 便宜) */
    function makeGaussian(rng) {
        return function () {
            var s = 0;
            for (var i = 0; i < 12; i++) s += rng();
            return s - 6;
        };
    }

    QC.math = {
        qIdentity: qIdentity,
        qNormalize: qNormalize,
        qIntegrate: qIntegrate,
        qToMatrix: qToMatrix,
        qFromEuler: qFromEuler,
        worldToBody: worldToBody,
        bodyToWorld: bodyToWorld,
        clamp: clamp,
        Mulberry32: Mulberry32,
        makeGaussian: makeGaussian
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
