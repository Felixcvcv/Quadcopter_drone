#ifndef __INF_SI24R1_H
#define __INF_SI24R1_H
#include "Com_Debug.h"
#include "spi.h"
#include "Com_Config.h"

#define CH 111 /* 射频通道(0~125), 收发双方必须一致 */

#define SI24R1_CS_H HAL_GPIO_WritePin(Si24R1_CS_GPIO_Port, Si24R1_CS_Pin, GPIO_PIN_SET)
#define SI24R1_CS_L HAL_GPIO_WritePin(Si24R1_CS_GPIO_Port, Si24R1_CS_Pin, GPIO_PIN_RESET)
#define SI24R1_EN_H HAL_GPIO_WritePin(Si24R1_CE_GPIO_Port, Si24R1_CE_Pin, GPIO_PIN_SET)
#define SI24R1_EN_L HAL_GPIO_WritePin(Si24R1_CE_GPIO_Port, Si24R1_CE_Pin, GPIO_PIN_RESET)

/*********************************** 收发数据宽度 ***********************************/
#define TX_ADR_WIDTH 5 /* 地址宽度: 5字节 */
#define RX_ADR_WIDTH 5

/* 有效数据宽度与上层协议帧长保持一致(见 Com_Config.h), 避免两边各写一个数字对不上 */
#define TX_PLOAD_WIDTH RF_FRAME_LEN
#define RX_PLOAD_WIDTH RF_FRAME_LEN

/*********************************** 寄存器操作指令 *********************************/
#define SPI_READ_REG 0x00  /* 读配置寄存器, 低5位为寄存器地址 */
#define SPI_WRITE_REG 0x20 /* 写配置寄存器, 低5位为寄存器地址 */
#define R_RX_PL_WID 0x60   /* 读取收到的数据字节数 */
#define RD_RX_PLOAD 0x61   /* 读RX有效数据, 1~32字节 */
#define WR_TX_PLOAD 0xA0   /* 写TX有效数据, 1~32字节 */
#define FLUSH_TX 0xE1      /* 清除TX FIFO, 发射模式下用 */
#define FLUSH_RX 0xE2      /* 清除RX FIFO, 接收模式下用 */
#define REUSE_TX_PL 0xE3   /* 重新使用上一包数据 */
#define W_ACK_PAYLOAD 0xA8 /* 通过ACK把数据发回去 */
#define NOP 0xFF           /* 空操作, 可用来读状态寄存器 */

/*********************************** 寄存器地址 *************************************/
#define CONFIG 0x00     /* 配置寄存器: bit0 1接收/0发送, bit1 中断使能, bit2 CRC模式, bit3 CRC使能 */
#define EN_AA 0x01      /* 自动应答使能, bit0~5 对应通道0~5 */
#define EN_RXADDR 0x02  /* 接收地址允许, bit0~5 对应通道0~5 */
#define SETUP_AW 0x03   /* 地址宽度: 00=3字节 01=4字节 02=5字节 */
#define SETUP_RETR 0x04 /* 自动重发: bit3:0 重发次数, bit7:4 重发延时 250*x+86us */
#define RF_CH 0x05      /* 射频通道, bit6:0 工作通道频率 */
#define RF_SETUP 0x06   /* 速率与功率: bit3 速率(0:1Mbps 1:2Mbps), bit2:1 发射功率 */
#define STATUS 0x07     /* 状态寄存器: bit0 TX FIFO满, bit3:1 接收数据通道号 */
#define MAX_TX 0x10     /* 状态: bit4 达到最大重发次数 */
#define TX_OK 0x20      /* 状态: bit5 发送完成 */
#define RX_OK 0x40      /* 状态: bit6 收到数据 */

#define OBSERVE_TX 0x08  /* 发送检测: bit7:4 丢包计数, bit3:0 重发计数 */
#define CD 0x09          /* 载波检测 */
#define RX_ADDR_P0 0x0A  /* 通道0接收地址, 最长5字节, 低字节在前 */
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10     /* 发送地址(低字节在前) */
#define RX_PW_P0 0x11    /* 通道0有效数据宽度(1~32字节), 0为非法 */
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17 /* bit0 RX FIFO空, bit1 RX FIFO满, bit4 TX FIFO空, bit5 TX FIFO满 */
#define DYNPD 0x1C       /* 动态负载长度使能 */
#define FEATURE 0x1D     /* bit1 使能ACK负载, bit2 使能动态负载长度 */

extern uint8_t TX_BUFF[TX_PLOAD_WIDTH];
extern uint8_t RX_BUFF[RX_PLOAD_WIDTH];

uint8_t Inf_Si24R1_WriteReg(uint8_t reg, uint8_t data);
uint8_t Inf_Si24R1_ReadReg(uint8_t reg);
uint8_t Inf_Si24R1_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t Inf_Si24R1_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);

/* 清空发送/接收FIFO */
void Inf_Si24R1_FlushTx(void);
void Inf_Si24R1_FlushRx(void);

/* 配置成发送/接收模式 */
void Inf_Si24R1_TXMode(void);
void Inf_Si24R1_RXMode(void);

/* 发送/接收一个数据包, 返回 0 表示成功 */
uint8_t Inf_Si24R1_TxPacket(uint8_t *txBuf);
uint8_t Inf_Si24R1_RxPacket(uint8_t *rxBuf);

/* 芯片自检: 写读地址寄存器比对, 返回 0 表示正常 */
uint8_t Inf_Si24R1_Check(void);

#endif
