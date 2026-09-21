#include "Inf_IP5305T.h"

/* 短按电源键: 拉低 -> 保持 -> 拉高.
   IP5305T 靠这个脉冲判断"开机/关机"操作 */
#define POWER_KEY_PRESS_MS (100)
/* 两次短按之间的间隔 */
#define POWER_KEY_GAP_MS (500)

/**
 * @description: 按一下电源键
 * @return {*}
 */
static void Inf_IP5305T_ShortPress(void)
{
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_RESET);

    vTaskDelay(POWER_KEY_PRESS_MS);

    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_SET);
}

/**
 * @description: 开机
 *   IP5305T 在无负载一段时间后会自动关机, 需要定期调用本函数补一次脉冲
 * @return {*}
 */
void Inf_IP5305T_Open(void)
{
    Inf_IP5305T_ShortPress();
}

/**
 * @description: 关机(连续两次短按)
 * @return {*}
 */
void Inf_IP5305T_Close(void)
{
    Inf_IP5305T_ShortPress();
    vTaskDelay(POWER_KEY_GAP_MS);
    Inf_IP5305T_ShortPress();
}
