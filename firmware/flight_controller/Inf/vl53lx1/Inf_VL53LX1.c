#include "Inf_VL53LX1.h"

/* VL53L1X 的 7 位 I2C 地址 */
#define VL53L1X_DEV_ADDR (0x52)

/* 测距时序: 单次测量 20ms, 测量间隔 20ms (间隔不能小于单次测量时间) */
#define VL53L1X_TIMING_BUDGET_MS       (20)
#define VL53L1X_INTER_MEASUREMENT_MS   (20)
/* 距离模式: 1 = 短距离(精度高), 2 = 长距离(量程大) */
#define VL53L1X_DISTANCE_MODE_LONG     (2)
/* 芯片上电复位的保持时间 */
#define VL53L1X_RESET_HOLD_MS (500)

/**
 * @description: 初始化激光测距芯片
 * @return {*}
 */
void Inf_VL53LX1_Init(void)
{
    /* 1. 硬复位: 拉低SHUT -> 保持 -> 拉高 */
    HAL_GPIO_WritePin(VL53LX1_SHUT_GPIO_Port, VL53LX1_SHUT_Pin, GPIO_PIN_RESET);
    HAL_Delay(VL53L1X_RESET_HOLD_MS);
    HAL_GPIO_WritePin(VL53LX1_SHUT_GPIO_Port, VL53LX1_SHUT_Pin, GPIO_PIN_SET);

    /* 2. 芯片初始化(载入ST的默认配置) */
    VL53L1X_SensorInit(VL53L1X_DEV_ADDR);

    /* 3. 距离模式 */
    VL53L1X_SetDistanceMode(VL53L1X_DEV_ADDR, VL53L1X_DISTANCE_MODE_LONG);

    /* 4. 单次测量时间 */
    VL53L1X_SetTimingBudgetInMs(VL53L1X_DEV_ADDR, VL53L1X_TIMING_BUDGET_MS);

    /* 5. 测量间隔 */
    VL53L1X_SetInterMeasurementInMs(VL53L1X_DEV_ADDR, VL53L1X_INTER_MEASUREMENT_MS);

    /* 6. 启动连续测量 */
    VL53L1X_StartRanging(VL53L1X_DEV_ADDR);

    /* 7. 读一次芯片ID, 便于确认通信正常 */
    uint16_t sensorID = 0;
    VL53L1X_GetSensorId(VL53L1X_DEV_ADDR, &sensorID);
    debug_printfln("vl53l1x: sensor id 0x%x", sensorID);
}

/**
 * @description: 返回测到的高度
 *   没有新数据时返回上一次的结果, 保证调用者拿到的始终是有效值
 * @return {*} 高度: mm
 */
uint16_t Inf_VL53LX1_GetHeight(void)
{
    static uint16_t height   = 0;
    uint8_t         isDataReady = 0;

    /* 检测本次测距是否完成 */
    VL53L1X_CheckForDataReady(VL53L1X_DEV_ADDR, &isDataReady);
    if(isDataReady)
    {
        VL53L1X_ClearInterrupt(VL53L1X_DEV_ADDR);
        VL53L1X_GetDistance(VL53L1X_DEV_ADDR, &height);
    }

    return height;
}
