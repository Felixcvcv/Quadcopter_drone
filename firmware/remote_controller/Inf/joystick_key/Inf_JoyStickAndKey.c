#include "Inf_JoyStickAndKey.h"

/* 按键参数 */
#define KEY_DEBOUNCE_MS   (30)   /* 消抖时间 */
#define KEY_POLL_MS       (20)   /* 等待抬起时的轮询间隔 */
#define KEY_LONG_PRESS_MS (500)  /* 按下超过该时长算长按 */
#define KEY_HOLD_MAX_MS   (1200) /* 等待抬起的最长时间, 超时视为按键卡住 */

/* 4路摇杆ADC的DMA缓冲区 */
static uint16_t joystickAdcBuff[4] = {0};

/* 面板上的6个按键 */
typedef enum
{
    BTN_LEFT = 0,
    BTN_RIGHT,
    BTN_UP,
    BTN_DOWN,
    BTN_LEFT_TOP,
    BTN_RIGHT_TOP,
    BTN_COUNT
} Com_Button;

#define READ_LEFT      HAL_GPIO_ReadPin(KEY_LEFT_GPIO_Port, KEY_LEFT_Pin)
#define READ_RIGHT     HAL_GPIO_ReadPin(KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin)
#define READ_UP        HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin)
#define READ_DOWN      HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin)
#define READ_LEFT_TOP  HAL_GPIO_ReadPin(KEY_LEFT_TOP_GPIO_Port, KEY_LEFT_TOP_Pin)
#define READ_RIGHT_TOP HAL_GPIO_ReadPin(KEY_RIGHT_TOP_GPIO_Port, KEY_RIGHT_TOP_Pin)

/**
 * @description: 读按键电平, 按下返回1
 * @param {Com_Button} btn
 * @return {*}
 */
static uint8_t Inf_JoyStickAndKey_IsPressed(Com_Button btn)
{
    uint8_t pressed = 0;

    switch(btn)
    {
        case BTN_LEFT:
            pressed = (READ_LEFT == 0) ? 1 : 0;
            break;

        case BTN_RIGHT:
            pressed = (READ_RIGHT == 0) ? 1 : 0;
            break;

        case BTN_UP:
            pressed = (READ_UP == 0) ? 1 : 0;
            break;

        case BTN_DOWN:
            pressed = (READ_DOWN == 0) ? 1 : 0;
            break;

        case BTN_LEFT_TOP:
            pressed = (READ_LEFT_TOP == 0) ? 1 : 0;
            break;

        case BTN_RIGHT_TOP:
            pressed = (READ_RIGHT_TOP == 0) ? 1 : 0;
            break;

        default:
            break;
    }

    return pressed;
}

/**
 * @description: 等待按键抬起, 返回按键按下的时长(ms)
 *   带超时上限: 原来这里是无界的 while(键还按着); 一旦按键卡住,
 *   按键任务就会永久占着CPU不返回
 * @param {Com_Button} btn
 * @param {uint16_t} maxMs 最长等待时间
 * @return {*} 按下时长(ms); 等于 maxMs 表示超时(没等到抬起)
 */
static uint16_t Inf_JoyStickAndKey_WaitReleaseMs(Com_Button btn, uint16_t maxMs)
{
    uint16_t heldMs = 0;

    while(Inf_JoyStickAndKey_IsPressed(btn) && heldMs < maxMs)
    {
        vTaskDelay(KEY_POLL_MS);
        heldMs += KEY_POLL_MS;
    }

    return heldMs;
}

/**
 * @description: 摇杆初始化: 启动ADC的DMA采集
 * @return {*}
 */
void Inf_JoyStickAndKey_Init(void)
{
    debug_printfln("joystick/key: init start");

    /* 1. ADC 校准 */
    HAL_ADCEx_Calibration_Start(&hadc1);

    /* 2. 启动4通道DMA采集, 转换结果由DMA直接写入缓冲区 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)joystickAdcBuff, 4);

    debug_printfln("joystick/key: init done");
}

/**
 * @description: 扫描摇杆数据(原始ADC值)
 * @return {*}
 */
void Inf_JoyStickAndKey_JoyStickScan(void)
{
    joyStick.THR = (int16_t)joystickAdcBuff[0];
    joyStick.YAW = (int16_t)joystickAdcBuff[1];
    joyStick.PIT = (int16_t)joystickAdcBuff[2];
    joyStick.ROL = (int16_t)joystickAdcBuff[3];
}

/**
 * @description: 扫描按键
 *   依次判断: 消抖 -> 定位到具体按键 -> 等到抬起.
 *   只有左侧两个键区分长按/短按.
 * @return {*} 按下的键; 没有按键或按键卡住时返回 KEY_NONE
 */
Com_Key Inf_JoyStickAndKey_KeyScan(void)
{
    Com_Button btn = BTN_COUNT;

    /* 1. 先看有没有按键, 没有就直接返回 */
    for(uint8_t i = 0; i < BTN_COUNT; i++)
    {
        if(Inf_JoyStickAndKey_IsPressed((Com_Button)i))
        {
            btn = (Com_Button)i;
            break;
        }
    }
    if(btn == BTN_COUNT) return KEY_NONE;

    /* 2. 消抖 */
    vTaskDelay(KEY_DEBOUNCE_MS);
    if(!Inf_JoyStickAndKey_IsPressed(btn)) return KEY_NONE;

    /* 3. 等抬起, 同时测出按下的时长 */
    uint16_t heldMs = Inf_JoyStickAndKey_WaitReleaseMs(btn, KEY_HOLD_MAX_MS);
    if(heldMs >= KEY_HOLD_MAX_MS) return KEY_NONE; /* 超时, 当作卡住 */

    /* 4. 左上/右上区分长按与短按 */
    switch(btn)
    {
        case BTN_LEFT:
            return KEY_LEFT;

        case BTN_RIGHT:
            return KEY_RIGHT;

        case BTN_UP:
            return KEY_UP;

        case BTN_DOWN:
            return KEY_DOWN;

        case BTN_LEFT_TOP:
            return (heldMs > KEY_LONG_PRESS_MS) ? KEY_LEFT_TOP_LONG : KEY_LEFT_TOP;

        case BTN_RIGHT_TOP:
            return (heldMs > KEY_LONG_PRESS_MS) ? KEY_RIGHT_TOP_LONG : KEY_RIGHT_TOP;

        default:
            break;
    }

    return KEY_NONE;
}
