#ifndef __COM_FILTER_H
#define __COM_FILTER_H
#include "Com_Debug.h"
#include "stdint.h"

/* 一阶低通滤波系数: 结果 = LOWPASS_ALPHA * 上次结果 + (1 - LOWPASS_ALPHA) * 本次采样
   系数越大越平滑、但相位滞后越明显 */
#define LOWPASS_ALPHA (0.8f)

/* 一维卡尔曼滤波器结构体 */
typedef struct
{
    float lastP;    /* 上一时刻的状态方差 */
    float nowP;     /* 当前时刻的状态方差 */
    float estimate; /* 状态估计值, 即滤波输出 */
    float gain;     /* 卡尔曼增益, 决定更相信预测还是更相信测量 */
    float Q;        /* 过程噪声方差, 反映模型不确定性 */
    float R;        /* 测量噪声方差, 反映测量不确定性 */
} KalmanFilter_Struct;

extern KalmanFilter_Struct kfs[3];

/* 一阶低通滤波(整型接口, 用于角速度原始数据) */
int16_t Com_Filter_LowPass(int16_t newData, int16_t lastData);

/* 一维卡尔曼滤波 */
float Com_Filter_Kalman(KalmanFilter_Struct *kf, float input);

#endif
