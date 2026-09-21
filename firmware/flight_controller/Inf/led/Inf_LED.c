#include "Inf_LED.h"

/* 4个LED的引脚映射: 机头方向朝前时, 左上/左下/右上/右下各一个 */
void Inf_LED_SetStatus(LedStruct *led)
{
    if(led->status <= LED_OFF)
    {
        /* 常亮/常灭: 引脚低电平点亮 */
        GPIO_PinState state = (led->status == LED_ON) ? GPIO_PIN_RESET : GPIO_PIN_SET;

        switch(led->location)
        {
            case LEFT_TOP:
                HAL_GPIO_WritePin(LED_LEFT_TOP_GPIO_Port, LED_LEFT_TOP_Pin, state);
                break;

            case LEFT_BOTTOM:
                HAL_GPIO_WritePin(LED_LEFT_BOTTOM_GPIO_Port, LED_LEFT_BOTTOM_Pin, state);
                break;

            case RIGHT_TOP:
                HAL_GPIO_WritePin(LED_RIGHT_TOP_GPIO_Port, LED_RIGHT_TOP_Pin, state);
                break;

            case RIGHT_BOTTOM:
                HAL_GPIO_WritePin(LED_RIGHT_BOTTOM_GPIO_Port, LED_RIGHT_BOTTOM_Pin, state);
                break;

            default:
                break;
        }
    }
    else
    {
        /* 闪烁: 这里只翻转电平, 闪烁周期由灯控任务决定何时调用 */
        switch(led->location)
        {
            case LEFT_TOP:
                HAL_GPIO_TogglePin(LED_LEFT_TOP_GPIO_Port, LED_LEFT_TOP_Pin);
                break;

            case LEFT_BOTTOM:
                HAL_GPIO_TogglePin(LED_LEFT_BOTTOM_GPIO_Port, LED_LEFT_BOTTOM_Pin);
                break;

            case RIGHT_TOP:
                HAL_GPIO_TogglePin(LED_RIGHT_TOP_GPIO_Port, LED_RIGHT_TOP_Pin);
                break;

            case RIGHT_BOTTOM:
                HAL_GPIO_TogglePin(LED_RIGHT_BOTTOM_GPIO_Port, LED_RIGHT_BOTTOM_Pin);
                break;

            default:
                break;
        }
    }
}
