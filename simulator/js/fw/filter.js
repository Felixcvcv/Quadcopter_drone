/*
 * fw/filter.js — Com_Filter.c 的移植
 *
 * 注意: 固件跑在没有硬件浮点的 Cortex-M3 上, 两个滤波器都刻意用单精度。
 * 这里保持同样的算式, 包括一阶低通输出取整到 int16 的行为
 * (它决定了滤波器在低幅度下的"粘滞"特性)。
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});

    /**
     * 一阶低通: result = ALPHA * last + (1 - ALPHA) * new
     * 对应 Com_Filter_LowPass(int16_t newData, int16_t lastData)
     */
    function LowPass(newData, lastData, alpha) {
        if (alpha === undefined) alpha = QC.config.LOWPASS_ALPHA;
        /* 固件返回 int16_t, 这里用 |0 截断为整数以保持一致 */
        return ((alpha * lastData + (1.0 - alpha) * newData) | 0);
    }

    /**
     * 一维卡尔曼滤波器 (对应 KalmanFilter_Struct + Com_Filter_Kalman)
     * 预测: nowP = lastP + Q
     * 更新: gain = nowP / (nowP + R); estimate += gain*(input-estimate)
     *       lastP = (1-gain)*nowP
     */
    function KalmanFilter(q, r) {
        var cfg = QC.config;
        return {
            lastP: 0,
            nowP: 0,
            estimate: 0,
            gain: 0,
            Q: (q === undefined) ? cfg.KALMAN_Q : q,
            R: (r === undefined) ? cfg.KALMAN_R : r
        };
    }

    function Kalman(kf, input) {
        kf.nowP = kf.lastP + kf.Q;
        kf.gain = kf.nowP / (kf.nowP + kf.R);
        kf.estimate = kf.estimate + kf.gain * (input - kf.estimate);
        kf.lastP = (1.0 - kf.gain) * kf.nowP;
        return kf.estimate;
    }

    QC.filter = {
        LowPass: LowPass,
        KalmanFilter: KalmanFilter,
        Kalman: Kalman
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
