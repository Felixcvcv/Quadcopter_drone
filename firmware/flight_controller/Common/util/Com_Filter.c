#include "Com_Filter.h"

/**
 * @description: 一阶低通滤波
 *   result = LOWPASS_ALPHA * lastData + (1 - LOWPASS_ALPHA) * newData
 *   用 float 而不是 double: STM32F103(Cortex-M3)没有硬件浮点单元,
 *   单精度由软件浮点库直接支持, 双精度还要再模拟一层, 代价高很多
 * @param {int16_t} newData  本次采样
 * @param {int16_t} lastData 上次的滤波结果
 * @return {*} 本次滤波结果
 */
int16_t Com_Filter_LowPass(int16_t newData, int16_t lastData)
{
    return (int16_t)(LOWPASS_ALPHA * (float)lastData + (1.0f - LOWPASS_ALPHA) * (float)newData);
}

/* 三轴加速度各用一个滤波器: Q 为过程噪声方差, R 为测量噪声方差 */
KalmanFilter_Struct kfs[3] = {
    {.Q = 0.001f, .R = 0.543f},
    {.Q = 0.001f, .R = 0.543f},
    {.Q = 0.001f, .R = 0.543f}};

/**
 * @description: 一维卡尔曼滤波
 *   预测: nowP = lastP + Q
 *   更新: gain = nowP / (nowP + R)
 *         estimate += gain * (input - estimate)
 *         lastP = (1 - gain) * nowP
 * @param {KalmanFilter_Struct} *kf
 * @param {float} input 本次测量值
 * @return {*} 滤波后的估计值
 */
float Com_Filter_Kalman(KalmanFilter_Struct *kf, float input)
{
    kf->nowP     = kf->lastP + kf->Q;
    kf->gain     = kf->nowP / (kf->nowP + kf->R);
    kf->estimate = kf->estimate + kf->gain * (input - kf->estimate);
    kf->lastP    = (1.0f - kf->gain) * kf->nowP;

    return kf->estimate;
}
