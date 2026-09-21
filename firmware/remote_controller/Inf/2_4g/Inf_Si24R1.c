#include "Inf_Si24R1.h"

/* 收发地址, 双方必须一致 */
static uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x0A, 0x01, 0x07, 0x0E, 0x01};
static uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x0A, 0x01, 0x07, 0x0E, 0x01};

uint8_t TX_BUFF[TX_PLOAD_WIDTH];
uint8_t RX_BUFF[RX_PLOAD_WIDTH];

/* 等待发送结果的最长轮询次数: 射频异常时不至于让调用者(可能处于临界区)死等 */
#define TX_POLL_LIMIT (20000U)

/**
 * @description: 写寄存器
 * @param {uint8_t} reg 寄存器地址
 * @param {uint8_t} data 要写入的字节
 * @return {*} 状态寄存器的值
 */
uint8_t Inf_Si24R1_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t status = 0;

    SI24R1_CS_L;
    status = Driver_SPI_SwapByte(reg);
    Driver_SPI_SwapByte(data);
    SI24R1_CS_H;

    return status;
}

/**
 * @description: 读寄存器
 * @param {uint8_t} reg 要读取的寄存器
 * @return {*} 读取到的值
 */
uint8_t Inf_Si24R1_ReadReg(uint8_t reg)
{
    uint8_t res = 0;

    SI24R1_CS_L;
    Driver_SPI_SwapByte(reg);
    res = Driver_SPI_SwapByte(0); /* 参数随意, 目的是产生时钟把数据移进来 */
    SI24R1_CS_H;

    return res;
}

/**
 * @description: 向指定寄存器写多个字节
 * @param {uint8_t} reg 写入的寄存器地址
 * @param {uint8_t*} pBuf 数据指针
 * @param {uint8_t} len 数据字节数
 * @return {*} 状态寄存器的值
 */
uint8_t Inf_Si24R1_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status = 0;

    SI24R1_CS_L;
    status = Driver_SPI_SwapByte(reg);
    for(uint8_t i = 0; i < len; i++)
    {
        Driver_SPI_SwapByte(*pBuf++);
    }
    SI24R1_CS_H;

    return status;
}

/**
 * @description: 从指定寄存器读多个字节
 * @param {uint8_t} reg 要读取的寄存器地址
 * @param {uint8_t *} pBuf 接收缓冲
 * @param {uint8_t} len 接收的字节数
 * @return {*}
 */
uint8_t Inf_Si24R1_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    SI24R1_CS_L;
    Driver_SPI_SwapByte(reg);
    for(uint8_t i = 0; i < len; i++)
    {
        *pBuf++ = Driver_SPI_SwapByte(0);
    }
    SI24R1_CS_H;

    return 0;
}

/**
 * @description: 清空TX FIFO(FLUSH_TX 是单字节命令, 后面不需要跟数据)
 * @return {*}
 */
void Inf_Si24R1_FlushTx(void)
{
    SI24R1_CS_L;
    Driver_SPI_SwapByte(FLUSH_TX);
    SI24R1_CS_H;
}

/**
 * @description: 清空RX FIFO
 * @return {*}
 */
void Inf_Si24R1_FlushRx(void)
{
    SI24R1_CS_L;
    Driver_SPI_SwapByte(FLUSH_RX);
    SI24R1_CS_H;
}

/**
 * @description: 发送模式初始化
 * @return {*}
 */
void Inf_Si24R1_TXMode(void)
{
    /* 1. 进入待机模式 */
    SI24R1_EN_L;

    /* 2. 配置发送地址/接收通道0地址(用于接收ACK)/自动应答/重发/通道/速率功率/CRC */
    Inf_Si24R1_WriteBuf(SPI_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
    Inf_Si24R1_WriteBuf(SPI_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + EN_AA, 0x01);      /* 使能通道0自动应答 */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + EN_RXADDR, 0x01);  /* 使能接收通道0 */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + SETUP_RETR, 0x0a); /* 重发延时250us+86us, 重发10次 */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + RF_CH, CH);        /* 射频通道 */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + RF_SETUP, 0x0f);   /* 2Mbps, 7dBm */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + CONFIG, 0x0e);     /* 16位CRC使能, 上电, 发送模式 */

    SI24R1_EN_H;
}

/**
 * @description: 接收模式初始化
 *   与发送模式的区别:
 *     1. 不需要设置发送地址
 *     2. 需要设置接收通道0的负载长度
 *     3. CONFIG 的 bit0 = 1 表示接收模式
 * @return {*}
 */
void Inf_Si24R1_RXMode(void)
{
    SI24R1_EN_L;

    Inf_Si24R1_WriteBuf(SPI_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + EN_AA, 0x01);            /* 使能通道0自动应答 */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + EN_RXADDR, 0x01);        /* 使能接收通道0 */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + RF_CH, CH);              /* 射频通道 */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); /* 与发送端相同的数据宽度 */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + RF_SETUP, 0x0f);         /* 2Mbps, 7dBm */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + CONFIG, 0x0f);           /* 16位CRC使能, 上电, 接收模式 */

    SI24R1_EN_H;
}

/**
 * @description: 发送一个数据包
 * @param {uint8_t} *txBuf 待发送的数据
 * @return {*} 0: 发送成功  1: 未成功(达到最大重发次数或超时)
 */
uint8_t Inf_Si24R1_TxPacket(uint8_t *txBuf)
{
    /* 1. 把数据写入TX FIFO */
    SI24R1_EN_L;
    Inf_Si24R1_WriteBuf(WR_TX_PLOAD, txBuf, TX_PLOAD_WIDTH);
    SI24R1_EN_H; /* 拉高CE启动发射 */

    /* 2. 轮询状态寄存器, 等待"发送完成"或"达到最大重发次数".
       加超时上限: 万一射频异常, 不能让调用者一直死等(调用点可能处于临界区) */
    uint8_t  state   = 0;
    uint32_t timeout = TX_POLL_LIMIT;

    while(!(state & (TX_OK | MAX_TX)) && timeout)
    {
        state = Inf_Si24R1_ReadReg(SPI_READ_REG + STATUS);
        timeout--;
    }

    if(!(state & (TX_OK | MAX_TX))) /* 超时: 清FIFO后报错 */
    {
        Inf_Si24R1_FlushTx();
        return 1;
    }

    /* 3. 写1清除中断标志位 */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + STATUS, state);

    /* 4. 达到最大重发次数时要主动清TX FIFO, 否则无法继续发送 */
    if(state & MAX_TX)
    {
        if(state & 0x01) /* bit0: TX FIFO满 */
        {
            Inf_Si24R1_FlushTx();
        }
        return 1;
    }

    return 0;
}

/**
 * @description: 接收一个数据包
 * @param {uint8_t} *rxBuf 读到数据的缓冲区
 * @return {*} 0: 成功读到数据  1: 未读到数据
 */
uint8_t Inf_Si24R1_RxPacket(uint8_t *rxBuf)
{
    /* 1. 读状态寄存器判断是否收到数据 */
    uint8_t state = Inf_Si24R1_ReadReg(SPI_READ_REG + STATUS);

    /* 2. 清除接收中断标志 */
    Inf_Si24R1_WriteReg(SPI_WRITE_REG + STATUS, state);

    if(!(state & RX_OK))
    {
        return 1;
    }

    /* 3. 从RX FIFO读出数据并清空 */
    Inf_Si24R1_ReadBuf(RD_RX_PLOAD, rxBuf, RX_PLOAD_WIDTH);
    Inf_Si24R1_FlushRx();

    return 0;
}

/**
 * @description: 自检: 往地址寄存器写一组数据再读回来比对
 * @return {*} 0: 正常  1: 异常
 */
uint8_t Inf_Si24R1_Check(void)
{
    const uint8_t buff_w[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t       buff_r[5] = {0};

    Inf_Si24R1_WriteBuf(SPI_WRITE_REG + TX_ADDR, (uint8_t *)buff_w, 5);
    Inf_Si24R1_ReadBuf(SPI_READ_REG + TX_ADDR, buff_r, 5);

    for(uint8_t i = 0; i < 5; i++)
    {
        if(buff_r[i] != buff_w[i])
        {
            return 1;
        }
    }

    return 0;
}
