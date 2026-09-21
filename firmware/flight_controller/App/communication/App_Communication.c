#include "App_Communication.h"

/* 失联判定: 连续这么多次没收到数据(通讯任务周期 6ms) => 6 * 200 = 1.2s */
#define LOST_FRAME_LIMIT (200)
/* 解锁动作: 油门推到最大/最小附近保持的次数 => 200 * 6ms = 1.2s */
#define UNLOCK_HOLD_COUNT (200)
/* 解锁状态下油门长时间处于最低 => 自动上锁, 200 * 50 * 6ms = 60s */
#define AUTO_LOCK_COUNT (UNLOCK_HOLD_COUNT * 50)
/* 油门"最大/最小附近"的判定阈值 */
#define THR_NEAR_MAX (960)
#define THR_NEAR_MIN (20)

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

    debug_printfln("comm: si24r1 -> RX mode");
    Inf_Si24R1_RXMode();

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
 * @description: 接收并解析摇杆数据
 *   2.4G 在受干扰或丢包时可能收到错帧, 因此必须逐个校验
 *   帧头 / 载荷长度 / 校验和, 三者都通过了才认为数据有效.
 * @return {*} Com_Status 是否收到有效数据: Com_OK 有效 否则无效
 */
Com_Status App_Communication_ReceiveJoyStickData(void)
{
    /* 1. 接收数据, 没有收到直接返回 */
    taskENTER_CRITICAL();
    uint8_t r = Inf_Si24R1_RxPacket(RX_BUFF);
    taskEXIT_CRITICAL();
    if(r == 1) return Com_FAIL;

    /* 2. 校验帧头(确认是自己的遥控器发的) */
    if(RX_BUFF[0] != FRAME_0 || RX_BUFF[1] != FRAME_1 || RX_BUFF[2] != FRAME_2)
    {
        return Com_FAIL;
    }

    /* 3. 校验载荷长度 */
    if(RX_BUFF[3] != RF_PAYLOAD_LEN) return Com_FAIL;

    /* 4. 校验累加和(帧头 + 长度 + 载荷 的累加值, 大端4字节) */
    uint32_t sum      = App_Communication_Checksum(RX_BUFF, RF_HEADER_LEN + RF_PAYLOAD_LEN);
    uint32_t frameSum = ((uint32_t)RX_BUFF[RF_HEADER_LEN + RF_PAYLOAD_LEN] << 24) |
                        ((uint32_t)RX_BUFF[RF_HEADER_LEN + RF_PAYLOAD_LEN + 1] << 16) |
                        ((uint32_t)RX_BUFF[RF_HEADER_LEN + RF_PAYLOAD_LEN + 2] << 8) |
                        ((uint32_t)RX_BUFF[RF_HEADER_LEN + RF_PAYLOAD_LEN + 3]);
    if(sum != frameSum) return Com_FAIL;

    /* 5. 解析载荷, 与发送端 App_Communication_SendJoyStickData 的顺序一一对应 */
    const uint8_t *p = &RX_BUFF[RF_HEADER_LEN];

    joyStick.THR = (int16_t)(((uint16_t)p[0] << 8) | p[1]);
    joyStick.YAW = (int16_t)(((uint16_t)p[2] << 8) | p[3]);
    joyStick.PIT = (int16_t)(((uint16_t)p[4] << 8) | p[5]);
    joyStick.ROL = (int16_t)(((uint16_t)p[6] << 8) | p[7]);

    joyStick.isPowerDown = p[8];

    if(p[9]) /* 收到1就翻转一次定高状态(按键是单次触发) */
    {
        joyStick.isFixHeight = !joyStick.isFixHeight;

        isFixHeight = (joyStick.isFixHeight == 1) ? Com_OK : Com_FAIL;
    }

    return Com_OK;
}

/**
 * @description: 检测遥控器的连接情况
 *   连接成功: 只要收到一条数据就算连接成功
 *   失联:     连续 LOST_FRAME_LIMIT 次没有收到数据
 * @param {Com_Status} isReceiveData 本次是否收到数据
 * @return {*} 是否连接成功
 */
Com_Status App_Communication_CheckConnection(Com_Status isReceiveData)
{
    /* 记录连续收不到数据的次数; 初值取上限, 避免刚上电还没收到数据就显示"已连接" */
    static uint16_t cnt = LOST_FRAME_LIMIT;

    /* 1. 收到数据 -> 连接成功 */
    if(isReceiveData == Com_OK)
    {
        cnt = 0;
        return Com_OK;
    }

    /* 2. 没收到 -> 计时 */
    if(cnt < LOST_FRAME_LIMIT)
    {
        cnt++;
    }

    return (cnt >= LOST_FRAME_LIMIT) ? Com_FAIL : Com_OK;
}

/**
 * @description: 遥控器的解锁/上锁
 *   把油门推到最高并保持 1.2s, 再拉到最低并保持 1.2s, 解锁成功.
 *
 *   自由状态
 *       油门升到最大附近(>=960) -> 进入"最大值附近"状态
 *   最大值附近
 *       保持 LOST_FRAME_LIMIT 次 -> 离开最大值状态
 *       中途掉下来 -> 回到自由状态
 *   离开最大值
 *       油门降到最低附近(<=20) -> 进入"最小值附近"状态
 *   最小值附近
 *       保持 LOST_FRAME_LIMIT 次 -> 解锁成功
 *       中途升上去 -> 回到自由状态
 *   已解锁
 *       油门长时间(60s)保持在最低 -> 自动上锁, 回到自由状态
 *
 * @param {Com_Status} isRemoteConnected 是否连接成功
 * @return {*} 是否解锁成功 Com_OK 解锁成功 其他 解锁失败
 */
Com_Status App_Communication_RemoteUnlock(Com_Status isRemoteConnected)
{
    /* 油门在最大值/最小值附近的持续时间 */
    static uint16_t thrMaxDuration = 0;
    static uint16_t thrMinDuration = 0;
    /* 油门所处的解锁状态 */
    static Com_RemoteStatus remoteStatus = THR_FREE;

    /* 1. 失联状态下直接上锁, 并退出定高 */
    if(isRemoteConnected != Com_OK)
    {
        remoteStatus         = THR_FREE;
        thrMaxDuration       = 0;
        thrMinDuration       = 0;
        joyStick.isFixHeight = 0;
        isFixHeight          = Com_FAIL;
        return Com_FAIL;
    }

    switch(remoteStatus)
    {
        case THR_FREE:
        {
            /* 自由状态: 持续时间从0开始计数 */
            thrMaxDuration = 0;
            thrMinDuration = 0;
            if(joyStick.THR >= THR_NEAR_MAX)
            {
                remoteStatus = THR_MAX;
            }
            break;
        }

        case THR_MAX:
        {
            if(joyStick.THR >= THR_NEAR_MAX)
            {
                if(++thrMaxDuration >= UNLOCK_HOLD_COUNT)
                {
                    remoteStatus = THR_MAX_LEAVE;
                }
            }
            else
            {
                /* 在最大值附近没坚持住 */
                remoteStatus = THR_FREE;
            }
            break;
        }

        case THR_MAX_LEAVE:
        {
            if(joyStick.THR <= THR_NEAR_MIN)
            {
                remoteStatus = THR_MIN;
            }
            break;
        }

        case THR_MIN:
        {
            if(joyStick.THR <= THR_NEAR_MIN)
            {
                if(++thrMinDuration >= UNLOCK_HOLD_COUNT)
                {
                    remoteStatus = THR_UNLOCK;
                }
            }
            else
            {
                remoteStatus = THR_FREE;
            }
            break;
        }

        case THR_UNLOCK:
        {
            /* 已解锁: 若油门长时间停在最低, 自动上锁 */
            static uint32_t lowDuration = 0;

            if(joyStick.THR <= THR_NEAR_MIN)
            {
                if(++lowDuration >= AUTO_LOCK_COUNT)
                {
                    remoteStatus = THR_FREE;
                    lowDuration  = 0;

                    /* 上锁的同时退出定高 */
                    joyStick.isFixHeight = 0;
                    isFixHeight          = Com_FAIL;

                    return Com_FAIL;
                }
            }
            else
            {
                lowDuration = 0;
            }

            return Com_OK;
        }

        default:
            break;
    }

    /* 除"已解锁"外, 其余状态都视为未解锁; 未解锁时不允许定高 */
    joyStick.isFixHeight = 0;
    isFixHeight          = Com_FAIL;

    return Com_FAIL;
}
