#include "Inf_Motor.h"

/* 电机 -> 定时器通道的对应关系
   空心杯电机用的是普通PWM, 4个电机各占一个定时器的一个通道 */
void Inf_Motor_Init(void)
{
    /* left-top: tim3-c1, left-bottom: tim4-c4, right-top: tim2-c2, right-bottom: tim1-c3 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    Inf_Motor_StopAll();
}

/**
 * @description: 设置指定电机的转速
 * @param {Motor_Struct} *motor
 * @return {*}
 */
void Inf_Motor_SetSpeed(Motor_Struct *motor)
{
    /* 限幅, 防止占空比越界 */
    motor->speed = (int16_t)LIMIT(motor->speed, MOTOR_STOP, MOTOR_MAX);

    switch(motor->location)
    {
        case LEFT_TOP:
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (uint32_t)motor->speed);
            break;

        case LEFT_BOTTOM:
            __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, (uint32_t)motor->speed);
            break;

        case RIGHT_TOP:
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, (uint32_t)motor->speed);
            break;

        case RIGHT_BOTTOM:
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, (uint32_t)motor->speed);
            break;

        default:
            break;
    }
}

/**
 * @description: 把4个电机当前的转速写到定时器
 * @return {*}
 */
void Inf_Motor_AllMotorsWork(void)
{
    Inf_Motor_SetSpeed(&motorLeftTop);
    Inf_Motor_SetSpeed(&motorLeftBottom);
    Inf_Motor_SetSpeed(&motorRightTop);
    Inf_Motor_SetSpeed(&motorRightBottom);
}

/**
 * @description: 4个电机全部停转(未解锁、油门过低时调用)
 * @return {*}
 */
void Inf_Motor_StopAll(void)
{
    motorLeftTop.speed     = MOTOR_STOP;
    motorLeftBottom.speed  = MOTOR_STOP;
    motorRightTop.speed    = MOTOR_STOP;
    motorRightBottom.speed = MOTOR_STOP;

    Inf_Motor_AllMotorsWork();
}
