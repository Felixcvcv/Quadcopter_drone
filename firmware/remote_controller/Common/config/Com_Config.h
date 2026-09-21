#ifndef __COM_CONFIG_H
#define __COM_CONFIG_H

#include "stdint.h"
#include "stdio.h"

typedef enum
{
    Com_OK = 0,
    Com_TIMEOUT,
    Com_FAIL
} Com_Status;

/* 取 x 在 [min, max] 之间的限幅值. 注意: 参数不应带副作用(可能被求值两次) */
#define LIMIT(x, min, max) (((x) >= (max)) ? (max) : (((x) <= (min)) ? (min) : (x)))

/* 取绝对值. 不用标准库的 abs(): 它在 <stdlib.h> 里, 本工程并未显式包含,
   而且对浮点实参会被隐式截断, 不如自己写清楚 */
#define COM_ABS(x) (((x) >= 0) ? (x) : -(x))

/* 按键 */
typedef enum
{
    KEY_NONE = 0,
    KEY_LEFT,
    KEY_UP,
    KEY_DOWN,
    KEY_RIGHT,
    KEY_LEFT_TOP,
    KEY_RIGHT_TOP,
    KEY_LEFT_TOP_LONG,
    KEY_RIGHT_TOP_LONG
} Com_Key;

typedef struct
{
    int16_t THR; /* 油门 */
    int16_t PIT; /* 俯仰 */
    int16_t ROL; /* 横滚 */
    int16_t YAW; /* 偏航 */

    uint8_t isPowerDown; /* 是否关机: 1:关机 0:不关机 */
    uint8_t isFixHeight; /* 是否翻转定高的状态 */
} JoyStick_Struct;

/* ======================== 2.4G 应用层协议 ========================
 * 一帧的字节布局(共 RF_FRAME_LEN 字节):
 *   [0..2]   帧头 FRAME_0 / FRAME_1 / FRAME_2
 *   [3]      有效载荷长度 = RF_PAYLOAD_LEN
 *   [4..13]  有效载荷
 *   [14..17] 累加校验和, 大端
 * =============================================================== */
#define FRAME_0 (0x11)
#define FRAME_1 (0x22)
#define FRAME_2 (0x33)

#define RF_HEADER_LEN   (4U)                                   /* 帧头(3) + 长度(1) */
#define RF_PAYLOAD_LEN  (10U)                                  /* 有效载荷字节数 */
#define RF_CHECKSUM_LEN (4U)                                   /* 校验和字节数 */
#define RF_FRAME_LEN    (RF_HEADER_LEN + RF_PAYLOAD_LEN + RF_CHECKSUM_LEN) /* 整帧 = 18 */

/* 摇杆 ADC 原始值范围(12位)与归一化后的范围 */
#define STICK_ADC_MAX   (4095)
#define STICK_ADC_MID   (2048)
#define STICK_VALUE_MAX (1000)
#define STICK_VALUE_MID (500)

extern JoyStick_Struct joyStick;
extern JoyStick_Struct joyStickBias;

void Com_Config_PrintJoyStick(uint8_t *pre);
void Com_Config_PrintJoyStickBias(uint8_t *pre);

#endif
