#ifndef __OUTPUTDATA_H
#define __OUTPUTDATA_H

/* 上位机波形工具(VOFA+ / J-Scope 一类)的帧格式:
   4 个 32 位整数(小端) + 2 字节 CRC16(小端) = 10 字节.
   把要观察的通道写进 OutData[0..3], 再调用 OutPut_Data() 即可. */

extern float OutData[4];

void OutPut_Data(void);

#endif
