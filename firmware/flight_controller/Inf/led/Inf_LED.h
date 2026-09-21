#ifndef __INF_LED_H
#define __INF_LED_H
#include "Com_Debug.h"
#include "gpio.h"
#include "Com_Config.h"

/* 按 led->status 更新一个LED:
   status <= 1 直接设置亮/灭, status >= 2 翻转一次(闪烁周期由调用者控制) */
void Inf_LED_SetStatus(LedStruct *led);

#endif
