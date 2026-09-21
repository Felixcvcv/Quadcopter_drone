#include "Com_PID.h"

/**
 * @description: 把角度误差折算到 (-180, 180], 使控制器始终沿最短路径回到期望值
 *   偏航角是周期量, 若直接相减, 期望值 0° / 测量值 359° 会被当成 -359° 的大误差,
 *   导致偏航反向猛转一圈.
 * @param {float} deg 原始误差(度)
 * @return {*} 折算后的误差(度)
 */
static float Com_PID_WrapAngle180(float deg)
{
    while(deg > 180.0f)
    {
        deg -= 360.0f;
    }
    while(deg <= -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

/**
 * @description: 清空pid的历史状态(积分、上次误差、输出)
 *   重新进入闭环(例如重新解锁、重新开启定高)之前调用,
 *   避免上一次运行残留的积分量导致输出突跳
 * @param {PID_Struct} *pid
 * @return {*}
 */
void Com_PID_Reset(PID_Struct *pid)
{
    pid->integral  = 0.0f;
    pid->lastError = 0.0f;
    pid->result    = 0.0f;
}

/**
 * @description: 计算pid
 * @param {PID_Struct} *pid
 * @return {*}
 */
void Com_PID_ComputePID(PID_Struct *pid)
{
    /* 0. 计算误差 */
    float error = pid->measure - pid->desire;
    if(pid->isAngle)
    {
        error = Com_PID_WrapAngle180(error);
    }

    /* 1. 比例项 */
    float pV = pid->kp * error;

    /* 2. 积分项: 带限幅, 抑制积分饱和 */
    pid->integral += pid->ki * error * pid->dt;
    if(pid->integralMax > pid->integralMin)
    {
        pid->integral = LIMIT(pid->integral, pid->integralMin, pid->integralMax);
    }

    /* 3. 微分项 */
    float dV       = pid->kd * (error - pid->lastError) / pid->dt;
    pid->lastError = error;

    /* 4. 计算结果 */
    pid->result = pV + pid->integral + dV;

    /* 5. 输出限幅 */
    if(pid->outMax > pid->outMin)
    {
        pid->result = LIMIT(pid->result, pid->outMin, pid->outMax);
    }
}

/**
 * @description: 串级pid
 * @param {PID_Struct} *out 外环
 * @param {PID_Struct} *in 内环
 * @return {*}
 */
void Com_PID_CascadePID(PID_Struct *out, PID_Struct *in)
{
    /* 1. 计算外环 */
    Com_PID_ComputePID(out);
    /* 2. 把外环的输出作为内环的期望值 */
    in->desire = out->result;
    /* 3. 计算内环 */
    Com_PID_ComputePID(in);
}
