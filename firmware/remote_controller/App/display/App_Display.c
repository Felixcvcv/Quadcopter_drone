#include "App_Display.h"

/* 进度条: 字库中从 BAR_SEG_BASE 开始连续放着 BAR_SEGMENTS+1 个字符,
   序号越大越"满", 最后一个是实心方块 */
#define BAR_SEG_BASE  (12)
#define BAR_SEGMENTS  (12)
/* 每 BAR_STEP_DIV 个行程量对应进度条的一段.
   注意: 这个比例在行程量约 500 以上就会画满, 与原来的显示效果一致 */
#define BAR_STEP_DIV  (41)

void App_Display_Start(void)
{
    OLED_Init();
}

/**
 * @description: 在指定位置画一段进度条
 * @param {uint8_t} x 横坐标
 * @param {uint8_t} y 纵坐标
 * @param {int16_t} segment 段数, 会被限幅到 [0, BAR_SEGMENTS]
 * @return {*}
 */
static void App_Display_DrawBar(uint8_t x, uint8_t y, int16_t segment)
{
    if(segment < 0) segment = 0;
    if(segment > BAR_SEGMENTS) segment = BAR_SEGMENTS;

    OLED_Show_CH(x, y, (uint8_t)(BAR_SEG_BASE + segment), 12, 1);
}

/**
 * @description: 刷新一屏内容
 *   第1行: 射频通道
 *   第3/4行: 4个摇杆通道, 每个通道画两条: 左边是绝对位置, 右边是相对中位的偏移
 * @return {*}
 */
void App_Display_Show(void)
{
    /* 第一行: 射频信道 */
    OLED_Show_CH(Line1_Begin + 00, Y0, 0, 12, 1);
    OLED_Show_CH(Line1_Begin + 12, Y0, 1, 12, 1);
    OLED_Show_CH(Line1_Begin + 24, Y0, 2, 12, 1);
    OLED_Show_CH(Line1_Begin + 36, Y0, 3, 12, 1);
    OLED_Show_CH(Line1_Begin + 48, Y0, 4, 12, 1);
    OLED_Show_CH(Line1_Begin + 60, Y0, 5, 12, 1);

    OLED_ShowNumber(2, Y0, CH, 3, 12);

    /* 第三、四行: 摇杆数据 */
    /* 油门 */
    OLED_ShowString(Line3_Begin + 00, Y2, "THR:", 12, 1);
    App_Display_DrawBar(Line4_Begin + 6, Y2, joyStick.THR / BAR_STEP_DIV);
    App_Display_DrawBar(Line4_Begin + 18, Y2, (joyStick.THR - STICK_VALUE_MID) / BAR_STEP_DIV);

    /* 横滚 */
    OLED_ShowString(Line3_Begin + 64, Y2, "ROL:", 12, 1);
    App_Display_DrawBar(Line4_Begin + 70, Y2, joyStick.ROL / BAR_STEP_DIV);
    App_Display_DrawBar(Line4_Begin + 82, Y2, (joyStick.ROL - STICK_VALUE_MID) / BAR_STEP_DIV);

    /* 偏航 */
    OLED_ShowString(Line3_Begin + 00, Y3, "YAW:", 12, 1);
    App_Display_DrawBar(Line4_Begin + 6, Y3, joyStick.YAW / BAR_STEP_DIV);
    App_Display_DrawBar(Line4_Begin + 18, Y3, (joyStick.YAW - STICK_VALUE_MID) / BAR_STEP_DIV);

    /* 俯仰 */
    OLED_ShowString(Line3_Begin + 64, Y3, "PIT:", 12, 1);
    App_Display_DrawBar(Line4_Begin + 70, Y3, joyStick.PIT / BAR_STEP_DIV);
    App_Display_DrawBar(Line4_Begin + 82, Y3, (joyStick.PIT - STICK_VALUE_MID) / BAR_STEP_DIV);

    /* 一次性刷到屏上 */
    OLED_Refresh_Gram();
}
