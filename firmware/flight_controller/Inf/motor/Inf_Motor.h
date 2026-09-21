#ifndef __INF_MOTOR_H
#define __INF_MOTOR_H
#include "Com_Debug.h"
#include "tim.h"
#include "Com_Config.h"

/* 启动4路PWM */
void Inf_Motor_Init(void);

/* 设置指定电机的转速(内部会限幅到 0 ~ MOTOR_MAX) */
void Inf_Motor_SetSpeed(Motor_Struct *motor);

/* 把4个电机的当前转速写到定时器 */
void Inf_Motor_AllMotorsWork(void);

/* 4个电机全部停转 */
void Inf_Motor_StopAll(void);

#endif
