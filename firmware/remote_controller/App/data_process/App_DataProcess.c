#include "App_DataProcess.h"

/* 摇杆校准: 采样次数与间隔(100 * 10ms = 1s) */
#define CALIB_SAMPLE_COUNT     (100)
#define CALIB_SAMPLE_INTERVAL  (10)
/* 微调按钮每次调整的步长 */
#define TRIM_STEP (10)

/**
 * @description: 数据处理模块的启动
 * @return {*}
 */
void App_DataProcess_Start(void)
{
    Inf_JoyStickAndKey_Init();
}

/**
 * @description: 把ADC原始值转换成 0~1000 的行程量并统一极性
 *   ADC 原始值范围 0~4095, 且推杆方向与行程量的方向相反:
 *       [4095, 0] => [0, 1000]
 *       1000 - raw * 1000 / 4095
 * @return {*}
 */
static void App_DataProcess_JoyStickPolarityAndRange(void)
{
    joyStick.THR = 1000 - joyStick.THR * 1000 / 4095;
    joyStick.ROL = 1000 - joyStick.ROL * 1000 / 4095;
    joyStick.PIT = 1000 - joyStick.PIT * 1000 / 4095;
    joyStick.YAW = 1000 - joyStick.YAW * 1000 / 4095;
}

/**
 * @description: 扣除校准得到的零偏, 并限幅到有效行程
 * @return {*}
 */
static void App_DataProcess_JoyStickWithBias(void)
{
    joyStick.THR -= joyStickBias.THR;
    joyStick.PIT -= joyStickBias.PIT;
    joyStick.ROL -= joyStickBias.ROL;
    joyStick.YAW -= joyStickBias.YAW;

    joyStick.THR = (int16_t)LIMIT(joyStick.THR, 0, STICK_VALUE_MAX);
    joyStick.PIT = (int16_t)LIMIT(joyStick.PIT, 0, STICK_VALUE_MAX);
    joyStick.ROL = (int16_t)LIMIT(joyStick.ROL, 0, STICK_VALUE_MAX);
    joyStick.YAW = (int16_t)LIMIT(joyStick.YAW, 0, STICK_VALUE_MAX);
}

/**
 * @description: 扫描并处理一次摇杆数据, 取一次一致的数据快照
 *   临界区只保护"扫描 + 换算"这几条赋值, 不能把带延时的操作放进来
 * @return {*}
 */
static void App_DataProcess_ReadJoyStick(void)
{
    taskENTER_CRITICAL();
    /* 1. 扫描摇杆 */
    Inf_JoyStickAndKey_JoyStickScan();
    /* 2. 极性和范围处理 */
    App_DataProcess_JoyStickPolarityAndRange();
    /* 3. 扣除零偏 */
    App_DataProcess_JoyStickWithBias();
    taskEXIT_CRITICAL();
}

/**
 * @description: 摇杆数据处理任务的主体
 * @return {*}
 */
void App_DataProcess_JoyStickDataProcess(void)
{
    App_DataProcess_ReadJoyStick();
}

/**
 * @description: 重新计算摇杆零偏(中位自校准)
 *   采样若干次求平均: 油门的目标值是行程量0, 其余三个通道的目标值是行程量中位.
 *   注意: 累加要用 int32_t. 若用 JoyStick_Struct 里的 int16_t 直接累加,
 *   100 次之后必然溢出(最大 100 * 1000 = 100000 > 32767).
 * @return {*}
 */
static void App_DataProcess_JoyStickCalcBias(void)
{
    int32_t sumThr = 0;
    int32_t sumPit = 0;
    int32_t sumRol = 0;
    int32_t sumYaw = 0;

    debug_printfln("joystick: calibrating, keep sticks centered ...");

    for(uint8_t i = 0; i < CALIB_SAMPLE_COUNT; i++)
    {
        Inf_JoyStickAndKey_JoyStickScan();
        App_DataProcess_JoyStickPolarityAndRange();

        sumThr += joyStick.THR - 0;                /* 油门以 0 为目标 */
        sumPit += joyStick.PIT - STICK_VALUE_MID;  /* 其余通道以中位为目标 */
        sumRol += joyStick.ROL - STICK_VALUE_MID;
        sumYaw += joyStick.YAW - STICK_VALUE_MID;

        /* 这里必须能真正让出CPU: 之前这个循环被包在 taskENTER_CRITICAL() 里,
           vTaskDelay 依赖 SysTick 中断, 而临界区会关掉中断, 结果是永久卡死 */
        vTaskDelay(CALIB_SAMPLE_INTERVAL);
    }

    joyStickBias.THR = (int16_t)(sumThr / CALIB_SAMPLE_COUNT);
    joyStickBias.PIT = (int16_t)(sumPit / CALIB_SAMPLE_COUNT);
    joyStickBias.ROL = (int16_t)(sumRol / CALIB_SAMPLE_COUNT);
    joyStickBias.YAW = (int16_t)(sumYaw / CALIB_SAMPLE_COUNT);

    debug_printfln("joystick: bias thr=%d pit=%d rol=%d yaw=%d",
                   joyStickBias.THR,
                   joyStickBias.PIT,
                   joyStickBias.ROL,
                   joyStickBias.YAW);
}

/**
 * @description: 按键处理
 * @return {*}
 */
void App_DataProcess_KeyDataProcess(void)
{
    Com_Key key = Inf_JoyStickAndKey_KeyScan();

    switch(key)
    {
        case KEY_RIGHT_TOP_LONG:
        {
            /* 长按: 重新校准摇杆零偏 */
            App_DataProcess_JoyStickCalcBias();
            break;
        }

        case KEY_LEFT_TOP:
        {
            /* 关机 */
            joyStick.isPowerDown = 1;
            break;
        }

        case KEY_RIGHT_TOP:
        {
            /* 切换定高: 接收方收到1之后对定高状态取反 */
            joyStick.isFixHeight = 1;
            break;
        }

        /* 以下4个键做零偏微调 */
        case KEY_LEFT:
        {
            joyStickBias.ROL += TRIM_STEP;
            break;
        }

        case KEY_RIGHT:
        {
            joyStickBias.ROL -= TRIM_STEP;
            break;
        }

        case KEY_UP:
        {
            joyStickBias.PIT -= TRIM_STEP;
            break;
        }

        case KEY_DOWN:
        {
            joyStickBias.PIT += TRIM_STEP;
            break;
        }

        default:
            break;
    }
}
