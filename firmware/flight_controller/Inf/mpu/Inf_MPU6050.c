#include "Inf_MPU6050.h"

/* ---- 量程与滤波配置 ----
   陀螺仪 ±2000度/秒, 加速度计 ±2g, 数字低通滤波 1(1kHz 输出速率)
   采样率 = 陀螺仪输出频率 / (1 + SMPLRT_DIV) = 1000 / 2 = 500Hz */
#define MPU_GYRO_FS_SEL_2000DPS (3 << 3) /* MPU_GYRO_CFG_REG */
#define MPU_ACCEL_FS_SEL_2G     (0)      /* MPU_ACCEL_CFG_REG */
#define MPU_DLPF_CFG            (1)      /* MPU_CFG_REG */
#define MPU_SMPLRT_DIV          (1)      /* MPU_SAMPLE_RATE_REG */
#define MPU_CLKSEL_PLL_XGYRO    (1)      /* MPU_PWR_MGMT1_REG: 时钟源选X轴陀螺PLL */

/* ±2g 量程下加速度计灵敏度为 16384 LSB/g, 标定时用它把静止时的读数折算成 1g */
#define MPU_ACCEL_1G_LSB (16383)

/* 静止判定: 相邻两次角速度采样之差不超过该值(LSB)就认为足够安静 */
#define MPU_STILL_DELTA_THRESHOLD (10)
/* 静止判定需要连续满足多少次(每次间隔3ms) */
#define MPU_STILL_SAMPLE_COUNT (30)
/* 静止判定/校准的超时保护, 避免飞机一直在动导致开机流程卡死 */
#define MPU_STILL_TIMEOUT (2000U)
/* 求平均值时的采样次数 */
#define MPU_CALIB_SAMPLE_COUNT (255U)
#define MPU_CALIB_SAMPLE_INTERVAL_MS (3)

static GyroAccel_Struct offsetGyroAccel;

static void Inf_MPU6050_Calibrate(void);

/**
 * @description: 向指定的寄存器写入一个字节
 * @param {uint8_t} reg
 * @param {uint8_t} byte
 * @return {*}
 */
static void Inf_MPU6050_WriteReg(uint8_t reg, uint8_t byte)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_W, reg, I2C_MEMADD_SIZE_8BIT, &byte, 1, 2000);
}

/**
 * @description: 从指定的寄存器开始读多个字节
 * @param {uint8_t} startReg
 * @param {uint8_t} *bytes
 * @param {uint8_t} len
 * @return {*}
 */
static void Inf_MPU6050_ReadRegs(uint8_t startReg, uint8_t *bytes, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_R, startReg, I2C_MEMADD_SIZE_8BIT, bytes, len, 2000);
}

/**
 * @description: 读取角速度(原始值)
 * @param {Gyro_Struct} *gyro
 * @return {*}
 */
static void Inf_MPU6050_ReadGyro(Gyro_Struct *gyro)
{
    uint8_t data[6] = {0};

    Inf_MPU6050_ReadRegs(MPU_GYRO_XOUTH_REG, data, 6);

    gyro->gyroX = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroY = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroZ = (int16_t)((data[4] << 8) | data[5]);
}

/**
 * @description: 读取加速度(原始值)
 * @param {Accel_Struct} *accel
 * @return {*}
 */
static void Inf_MPU6050_ReadAccel(Accel_Struct *accel)
{
    uint8_t data[6] = {0};

    Inf_MPU6050_ReadRegs(MPU_ACCEL_XOUTH_REG, data, 6);

    accel->accelX = (int16_t)((data[0] << 8) | data[1]);
    accel->accelY = (int16_t)((data[2] << 8) | data[3]);
    accel->accelZ = (int16_t)((data[4] << 8) | data[5]);
}

/**
 * @description: 等待飞机静止
 *   连续 MPU_STILL_SAMPLE_COUNT 次采样中, 相邻两次的角速度差都在阈值内,
 *   就认为飞机已经静止. 带超时保护, 超时后直接用当前数据校准.
 * @return {*}
 */
static void Inf_MPU6050_WaitUntilStill(void)
{
    GyroAccel_Struct current;
    GyroAccel_Struct last;
    uint8_t          stillCount = 0;
    uint32_t         timeout    = MPU_STILL_TIMEOUT;

    debug_printfln("mpu6050: waiting for standstill");

    /* 先取一次作为比较基准, 否则首次比较会用到未初始化的数据 */
    Inf_MPU6050_ReadGyroAccel(&last);

    while(stillCount < MPU_STILL_SAMPLE_COUNT && timeout)
    {
        Inf_MPU6050_ReadGyroAccel(&current);

        if(COM_ABS(current.gyro.gyroX - last.gyro.gyroX) <= MPU_STILL_DELTA_THRESHOLD &&
           COM_ABS(current.gyro.gyroY - last.gyro.gyroY) <= MPU_STILL_DELTA_THRESHOLD &&
           COM_ABS(current.gyro.gyroZ - last.gyro.gyroZ) <= MPU_STILL_DELTA_THRESHOLD)
        {
            stillCount++;
        }
        else
        {
            stillCount = 0; /* 只要有抖动就重新计数 */
        }

        last = current;
        timeout--;
        HAL_Delay(MPU_CALIB_SAMPLE_INTERVAL_MS);
    }

    if(timeout == 0)
    {
        debug_printfln("mpu6050: standstill timeout, calibrate anyway");
    }
    else
    {
        debug_printfln("mpu6050: standstill detected");
    }
}

/**
 * @description: 在飞机水平静止的状态下校准六轴数据
 *   先确认静止, 再多点采样求平均得到零偏:
 *   角速度的期望静止值都是0; 加速度Z轴的期望静止值是 1g.
 * @return {*}
 */
static void Inf_MPU6050_Calibrate(void)
{
    GyroAccel_Struct sample;
    int32_t          sumBuff[6] = {0};

    /* 1. 确认飞机处于静止 */
    Inf_MPU6050_WaitUntilStill();

    /* 2. 多次采样求平均.
       注意: 这里用局部变量接收采样, 不能写进全局的 gyroAccel,
       否则会把校准过程中的中间值暴露给正在运行的控制任务 */
    debug_printfln("mpu6050: calibration start");
    for(uint16_t i = 0; i < MPU_CALIB_SAMPLE_COUNT; i++)
    {
        Inf_MPU6050_ReadGyroAccel(&sample);

        sumBuff[0] += sample.gyro.gyroX;
        sumBuff[1] += sample.gyro.gyroY;
        sumBuff[2] += sample.gyro.gyroZ;
        sumBuff[3] += sample.accel.accelX;
        sumBuff[4] += sample.accel.accelY;
        sumBuff[5] += sample.accel.accelZ - MPU_ACCEL_1G_LSB;

        HAL_Delay(MPU_CALIB_SAMPLE_INTERVAL_MS);
    }

    offsetGyroAccel.gyro.gyroX   = (int16_t)(sumBuff[0] / (int32_t)MPU_CALIB_SAMPLE_COUNT);
    offsetGyroAccel.gyro.gyroY   = (int16_t)(sumBuff[1] / (int32_t)MPU_CALIB_SAMPLE_COUNT);
    offsetGyroAccel.gyro.gyroZ   = (int16_t)(sumBuff[2] / (int32_t)MPU_CALIB_SAMPLE_COUNT);
    offsetGyroAccel.accel.accelX = (int16_t)(sumBuff[3] / (int32_t)MPU_CALIB_SAMPLE_COUNT);
    offsetGyroAccel.accel.accelY = (int16_t)(sumBuff[4] / (int32_t)MPU_CALIB_SAMPLE_COUNT);
    offsetGyroAccel.accel.accelZ = (int16_t)(sumBuff[5] / (int32_t)MPU_CALIB_SAMPLE_COUNT);

    debug_printfln("mpu6050: calibration done");
}

/**
 * @description: MPU6050 初始化
 * @return {*}
 */
void Inf_MPU6050_Init(void)
{
    /* 1. 复位: 复位 -> 等待200ms -> 唤醒 */
    Inf_MPU6050_WriteReg(MPU_PWR_MGMT1_REG, 1 << 7);
    HAL_Delay(200);
    Inf_MPU6050_WriteReg(MPU_PWR_MGMT1_REG, 0);

    /* 2. 设置量程 */
    Inf_MPU6050_WriteReg(MPU_GYRO_CFG_REG, MPU_GYRO_FS_SEL_2000DPS); /* 角速度 ±2000度/秒 */
    Inf_MPU6050_WriteReg(MPU_ACCEL_CFG_REG, MPU_ACCEL_FS_SEL_2G);    /* 加速度 ±2g */

    /* 3. 关闭中断、关闭第二IIC接口、禁止FIFO */
    Inf_MPU6050_WriteReg(MPU_INT_EN_REG, 0);
    Inf_MPU6050_WriteReg(MPU_USER_CTRL_REG, 0);
    Inf_MPU6050_WriteReg(MPU_FIFO_EN_REG, 0);

    /* 4. 采样率与低通滤波
       采样频率 = 陀螺仪输出频率 / (1 + SMPLRT_DIV) = 1000 / (1 + 1) = 500Hz */
    Inf_MPU6050_WriteReg(MPU_SAMPLE_RATE_REG, MPU_SMPLRT_DIV);
    Inf_MPU6050_WriteReg(MPU_CFG_REG, MPU_DLPF_CFG);

    /* 5. 配置系统时钟源 */
    Inf_MPU6050_WriteReg(MPU_PWR_MGMT1_REG, MPU_CLKSEL_PLL_XGYRO);

    /* 6. 使能角速度与加速度传感器(退出待机) */
    Inf_MPU6050_WriteReg(MPU_PWR_MGMT2_REG, 0);

    /* 7. 校准零偏 */
    Inf_MPU6050_Calibrate();
}

/**
 * @description: 读取角速度和加速度(原始值)
 * @param {GyroAccel_Struct} *gyroAccel
 * @return {*}
 */
void Inf_MPU6050_ReadGyroAccel(GyroAccel_Struct *gyroAccel)
{
    Inf_MPU6050_ReadGyro(&gyroAccel->gyro);
    Inf_MPU6050_ReadAccel(&gyroAccel->accel);
}

/**
 * @description: 读取角速度和加速度, 并扣除校准得到的零偏
 * @param {GyroAccel_Struct} *gyroAccel
 * @return {*}
 */
void Inf_MPU6050_ReadGyroAccelCalibrated(GyroAccel_Struct *gyroAccel)
{
    Inf_MPU6050_ReadGyroAccel(gyroAccel);

    gyroAccel->gyro.gyroX -= offsetGyroAccel.gyro.gyroX;
    gyroAccel->gyro.gyroY -= offsetGyroAccel.gyro.gyroY;
    gyroAccel->gyro.gyroZ -= offsetGyroAccel.gyro.gyroZ;
    gyroAccel->accel.accelX -= offsetGyroAccel.accel.accelX;
    gyroAccel->accel.accelY -= offsetGyroAccel.accel.accelY;
    gyroAccel->accel.accelZ -= offsetGyroAccel.accel.accelZ;
}
