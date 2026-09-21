#include "App_Communication.h"

/**
 * @description: 启动通讯模块
 * @return {*}
 */
void App_Communication_Start(void)
{
    debug_printfln("comm: init start");

    debug_printfln("comm: si24r1 self-test start");
    while(Inf_Si24R1_Check() == 1)
    {
        HAL_Delay(500);
    }
    debug_printfln("comm: si24r1 self-test done");

    debug_printfln("comm: si24r1 -> TX mode");
    Inf_Si24R1_TXMode();

    debug_printfln("comm: init done");
}

/**
 * @description: 计算累加校验和
 * @param {uint8_t} *buf 数据首地址
 * @param {uint8_t} len 参与计算的字节数
 * @return {*} 累加和
 */
static uint32_t App_Communication_Checksum(const uint8_t *buf, uint8_t len)
{
    uint32_t sum = 0;

    for(uint8_t i = 0; i < len; i++)
    {
        sum += buf[i];
    }

    return sum;
}

/**
 * @description: 通过2.4g把摇杆数据发出去
 *   帧格式见 Com_Config.h 的协议说明; 载荷顺序必须与飞控板的
 *   App_Communication_ReceiveJoyStickData 一一对应, 改一边就要改另一边.
 * @return {*}
 */
void App_Communication_SendJoyStickData(void)
{
    uint8_t index = 0;

    /* 1. 帧头 */
    TX_BUFF[index++] = FRAME_0;
    TX_BUFF[index++] = FRAME_1;
    TX_BUFF[index++] = FRAME_2;

    /* 2. 载荷长度, 内容填完后回填 */
    TX_BUFF[index++] = 0;

    /* 3. 4个摇杆通道, 大端 */
    uint16_t thr = (uint16_t)joyStick.THR;
    uint16_t yaw = (uint16_t)joyStick.YAW;
    uint16_t pit = (uint16_t)joyStick.PIT;
    uint16_t rol = (uint16_t)joyStick.ROL;

    TX_BUFF[index++] = (uint8_t)(thr >> 8);
    TX_BUFF[index++] = (uint8_t)(thr & 0xFFU);

    TX_BUFF[index++] = (uint8_t)(yaw >> 8);
    TX_BUFF[index++] = (uint8_t)(yaw & 0xFFU);

    TX_BUFF[index++] = (uint8_t)(pit >> 8);
    TX_BUFF[index++] = (uint8_t)(pit & 0xFFU);

    TX_BUFF[index++] = (uint8_t)(rol >> 8);
    TX_BUFF[index++] = (uint8_t)(rol & 0xFFU);

    /* 4. 关机命令与定高切换: 都是"发一次就清零"的单次命令 */
    TX_BUFF[index++]     = joyStick.isPowerDown;
    joyStick.isPowerDown = 0;

    TX_BUFF[index++]     = joyStick.isFixHeight;
    joyStick.isFixHeight = 0;

    /* 5. 回填载荷长度 */
    TX_BUFF[3] = (uint8_t)(index - RF_HEADER_LEN);

    /* 6. 累加校验和, 大端追加在末尾 */
    uint32_t sum = App_Communication_Checksum(TX_BUFF, index);

    TX_BUFF[index++] = (uint8_t)((sum >> 24) & 0xFFU);
    TX_BUFF[index++] = (uint8_t)((sum >> 16) & 0xFFU);
    TX_BUFF[index++] = (uint8_t)((sum >> 8) & 0xFFU);
    TX_BUFF[index++] = (uint8_t)(sum & 0xFFU);

    /* 7. 发送 */
    taskENTER_CRITICAL();
    Inf_Si24R1_TxPacket(TX_BUFF);
    taskEXIT_CRITICAL();
}
