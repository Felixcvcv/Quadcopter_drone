#include "outputdata.h"
#include "usart.h"

/* 4 个待观察的通道 */
float OutData[4] = {0};

/* 帧格式: 4 个通道各 2 字节(小端) + 2 字节 CRC16 = 10 字节,
   与原来上位机(VOFA+ / J-Scope 一类)的解析脚本保持一致 */
#define SCOPE_CHANNEL_COUNT (4U)
#define SCOPE_DATA_LEN      (SCOPE_CHANNEL_COUNT * 2U)
#define SCOPE_FRAME_LEN     (SCOPE_DATA_LEN + 2U)

/**
 * @description: 计算CRC16(Modbus多项式 0xA001, 初值 0xFFFF)
 * @param {uint8_t} *buf 数据首地址
 * @param {uint8_t} len 参与计算的字节数
 * @return {*} CRC16
 */
static uint16_t Com_Scope_CRC16(const uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;

    for(uint8_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x01U)
            {
                crc = (crc >> 1) ^ 0xA001U;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

/**
 * @description: 把 OutData[0..3] 按 10 字节的帧发给上位机
 * @return {*}
 */
void OutPut_Data(void)
{
    uint8_t  databuf[SCOPE_FRAME_LEN] = {0};
    uint32_t temp;

    /* 1. 每个通道取低16位, 小端排列 */
    for(uint8_t i = 0; i < SCOPE_CHANNEL_COUNT; i++)
    {
        temp = (uint32_t)(int32_t)OutData[i];

        databuf[i * 2 + 0] = (uint8_t)(temp & 0xFFU);
        databuf[i * 2 + 1] = (uint8_t)((temp >> 8) & 0xFFU);
    }

    /* 2. 追加 CRC16, 低字节在前 */
    uint16_t crc = Com_Scope_CRC16(databuf, SCOPE_DATA_LEN);

    databuf[SCOPE_DATA_LEN + 0] = (uint8_t)(crc & 0xFFU);
    databuf[SCOPE_DATA_LEN + 1] = (uint8_t)((crc >> 8) & 0xFFU);

    /* 3. 发送 */
    HAL_UART_Transmit(&huart2, databuf, SCOPE_FRAME_LEN, 1000);
}
