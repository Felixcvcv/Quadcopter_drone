#ifndef __COM_PID_H
#define __COM_PID_H
#include "Com_Debug.h"
#include "Com_Config.h"

/* 计算一次pid: 误差 = 测量值 - 期望值, 结果放在 pid->result */
void Com_PID_ComputePID(PID_Struct *pid);

/* 串级pid: 外环的输出作为内环的期望值 */
void Com_PID_CascadePID(PID_Struct *out, PID_Struct *in);

/* 清空积分/上次误差/输出, 重新进入闭环前调用 */
void Com_PID_Reset(PID_Struct *pid);

#endif
