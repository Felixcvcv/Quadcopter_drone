#include "App_Task.h"

#define FW_NAME    "Quadcopter Flight Controller"
#define FW_VERSION "V1.0.0"

/* 任务名长度上限是 FreeRTOSConfig.h 里的 configMAX_TASK_NAME_LEN(16, 含结束符),
   超长会被内核截断, 所以这里都控制在 15 个字符以内 */
/* 0. 启动任务 */
static void startTask(void *args);
#define START_TASK_NAME     "start_task"
#define START_TASK_STACK    (128) /* 单位: 字(32位架构下 1 字 = 4 字节) */
#define START_TASK_PRIORITY (10)
static TaskHandle_t startTaskHandle;

/* 1. 电源管理任务 */
static void powerTask(void *args);
#define POWER_TASK_NAME     "power_task"
#define POWER_TASK_STACK    (128)
#define POWER_TASK_PRIORITY (9)
static TaskHandle_t powerTaskHandle;
/* 在这么长时间内没收到关机命令, 就补一次开机脉冲防止IP5305T自动关机 */
#define POWER_EXEC_CYCLE (10 * 1000)

/* 2. 灯控任务 */
static void ledTask(void *args);
#define LED_TASK_NAME     "led_task"
#define LED_TASK_STACK    (128)
#define LED_TASK_PRIORITY (2)
static TaskHandle_t ledTaskHandle;
#define LED_EXEC_CYCLE 50

/* 3. 飞控任务: 姿态与定高的控制周期 */
static void flightTask(void *args);
#define FLIGHT_TASK_NAME     "flight_task"
#define FLIGHT_TASK_STACK    (256)
#define FLIGHT_TASK_PRIORITY (8)
static TaskHandle_t flightTaskHandle;
#define FLIGHT_EXEC_CYCLE 4

/* 4. 通讯任务: 接收遥控器数据、判断连接与解锁 */
static void communicationTask(void *args);
#define COMMUNICATION_TASK_NAME     "comm_task"
#define COMMUNICATION_TASK_STACK    (256)
#define COMMUNICATION_TASK_PRIORITY (8)
static TaskHandle_t communicationTaskHandle;
#define COMMUNICATION_EXEC_CYCLE 6

/* LED 闪烁周期: status >= 2 时表示"每多少个灯控周期翻转一次", 灯控周期为 50ms */
#define LED_BLINK_FAST (2)  /* 100ms */
#define LED_BLINK_MID  (10) /* 500ms */
#define LED_BLINK_SLOW (15) /* 750ms */

/**
 * @description: 创建任务并对创建失败做统一处理
 * @return {*}
 */
static void App_Task_Create(TaskFunction_t  taskFunc,
                            const char     *name,
                            uint16_t        stackDepth,
                            UBaseType_t     priority,
                            TaskHandle_t   *handle)
{
    if(xTaskCreate(taskFunc, name, stackDepth, NULL, priority, handle) != pdPASS)
    {
        debug_printfln("task create failed: %s", name);
    }
}

/**
 * @description: 启动实时系统
 * @return {*}
 */
void App_Task_FreeRTOSStart(void)
{
    /* 1. 初始化debug模块 */
    debug_start();
    debug_printfln(FW_NAME " " FW_VERSION);

    /* 2. 启动飞行模块(电机/MPU6050/激光测距) */
    App_Flight_Start();

    /* 3. 启动通讯模块 */
    App_Communication_Start();

    /* 4. 创建启动任务: 由它在调度器运行起来之后创建其余业务任务 */
    App_Task_Create(startTask,
                    START_TASK_NAME,
                    START_TASK_STACK,
                    START_TASK_PRIORITY,
                    &startTaskHandle);

    /* 5. 启动调度器(正常情况下不会返回) */
    vTaskStartScheduler();
}

/* 启动任务函数 */
static void startTask(void *args)
{
    debug_printfln("start_task running");

    /* 1. 电源控制任务 */
    App_Task_Create(powerTask,
                    POWER_TASK_NAME,
                    POWER_TASK_STACK,
                    POWER_TASK_PRIORITY,
                    &powerTaskHandle);

    /* 2. 灯控任务 */
    App_Task_Create(ledTask, LED_TASK_NAME, LED_TASK_STACK, LED_TASK_PRIORITY, &ledTaskHandle);

    /* 3. 飞控任务 */
    App_Task_Create(flightTask,
                    FLIGHT_TASK_NAME,
                    FLIGHT_TASK_STACK,
                    FLIGHT_TASK_PRIORITY,
                    &flightTaskHandle);

    /* 4. 通讯任务 */
    App_Task_Create(communicationTask,
                    COMMUNICATION_TASK_NAME,
                    COMMUNICATION_TASK_STACK,
                    COMMUNICATION_TASK_PRIORITY,
                    &communicationTaskHandle);

    /* 业务任务都已创建, 删除自己(vTaskDelete 不再返回) */
    vTaskDelete(NULL);
}

/* 1. 电源任务 */
static void powerTask(void *args)
{
    debug_printfln("power_task running");

    while(1)
    {
        /* 等关机命令; 超时则补一次短按脉冲, 避免IP5305T长时间无操作自动关机 */
        if(ulTaskNotifyTake(pdTRUE, POWER_EXEC_CYCLE))
        {
            debug_printfln("power off requested");
            Inf_IP5305T_Close();
            vTaskDelay(POWER_EXEC_CYCLE); /* 关机流程较长, 让出CPU */
        }
        else
        {
            Inf_IP5305T_Open();
        }
    }
}

/* 2. 灯控任务: 用4个LED指示 未连接 / 已连接 / 已解锁 / 定高 四种状态 */
static void ledTask(void *args)
{
    debug_printfln("led_task running");

    uint32_t preTime = xTaskGetTickCount();
    uint32_t cnt     = 0;

    static uint8_t isFirstUnlock = 1; /* 是否是本次上电后的第一次解锁 */
    while(1)
    {
        /* 定高 > 解锁 > 已连接 > 未连接, 优先级从高到低 */
        if(isFixHeight == Com_OK)
        {
            ledLeftTop.status = LED_BLINK_FAST;
            ledLeftBottom.status = LED_BLINK_FAST;
            ledRightTop.status = LED_BLINK_FAST;
            ledRightBottom.status = LED_BLINK_FAST;
        }
        else if(isRemoteUnlocked == Com_OK)
        {
            if(isFirstUnlock == 1)
            {
                /* 第一次解锁: 常灭, 作为解锁动作的确认 */
                ledLeftTop.status = LED_OFF;
                ledLeftBottom.status = LED_OFF;
                ledRightTop.status = LED_OFF;
                ledRightBottom.status = LED_OFF;

                isFirstUnlock = 0;
            }
            else
            {
                ledLeftTop.status = LED_BLINK_MID;
                ledLeftBottom.status = LED_BLINK_MID;
                ledRightTop.status = LED_BLINK_MID;
                ledRightBottom.status = LED_BLINK_MID;
            }
        }
        else if(isRemoteConnected == Com_OK)
        {
            isFirstUnlock = 1;

            /* 已连接未解锁: 对角常灭, 另一对角慢闪 */
            ledLeftTop.status = LED_OFF;
            ledRightTop.status = LED_OFF;
            ledLeftBottom.status = LED_BLINK_SLOW;
            ledRightBottom.status = LED_BLINK_SLOW;
        }
        else
        {
            /* 未连接: 对角慢闪 */
            ledLeftTop.status = LED_BLINK_SLOW;
            ledRightTop.status = LED_BLINK_SLOW;
            ledLeftBottom.status = LED_OFF;
            ledRightBottom.status = LED_OFF;
        }

        cnt++;

        /* status >= 2 时按周期翻转; status <= 1(常亮/常灭)时每次都直接刷新 */
        if(ledLeftTop.status <= LED_OFF || cnt % ledLeftTop.status == 0)
        {
            Inf_LED_SetStatus(&ledLeftTop);
        }

        if(ledLeftBottom.status <= LED_OFF || cnt % ledLeftBottom.status == 0)
        {
            Inf_LED_SetStatus(&ledLeftBottom);
        }

        if(ledRightTop.status <= LED_OFF || cnt % ledRightTop.status == 0)
        {
            Inf_LED_SetStatus(&ledRightTop);
        }

        if(ledRightBottom.status <= LED_OFF || cnt % ledRightBottom.status == 0)
        {
            Inf_LED_SetStatus(&ledRightBottom);
        }

        vTaskDelayUntil(&preTime, LED_EXEC_CYCLE);
    }
}

/* 3. 飞控任务 */
static void flightTask(void *args)
{
    debug_printfln("flight_task running");

    uint32_t preTime = xTaskGetTickCount();
    float    dt      = FLIGHT_EXEC_CYCLE / 1000.0f; /* 控制周期(秒) */

    while(1)
    {
        /* 1. 对6轴数据做滤波 */
        App_Flight_GetGyroAccelWithFilter(&gyroAccel);

        /* 2. 姿态解算, 得到欧拉角 */
        App_Flight_GetEulerAngle(&gyroAccel, &eulerAngle, dt);

        /* 3. 计算3组串级姿态pid(共6个pid) */
        App_Flight_PIDPosture(&gyroAccel, &eulerAngle, dt);

        /* 4. 把姿态pid的作用量分配到4个电机 */
        App_Flight_MotorWithPosturePID(isRemoteUnlocked);

        /* 5. 读取当前高度 */
        uint16_t height = App_Flight_GetHeight();

        /* 6. 计算定高串级pid */
        App_Flight_PIDHeight(isRemoteUnlocked, height, dt);

        /* 7. 把定高pid的作用量叠加到4个电机 */
        App_Flight_MotorWithHeightPID(isRemoteUnlocked);

        /* 8. 真正输出到电机 */
        App_Flight_Work(isRemoteUnlocked);

        vTaskDelayUntil(&preTime, FLIGHT_EXEC_CYCLE);
    }
}

/* 4. 通讯任务 */
static void communicationTask(void *args)
{
    debug_printfln("comm_task running");

    uint32_t preTime = xTaskGetTickCount();

    while(1)
    {
        /* 1. 读取2.4g数据 */
        Com_Status isReceiveData = App_Communication_ReceiveJoyStickData();

        /* 2. 判断遥控器的连接情况 */
        isRemoteConnected = App_Communication_CheckConnection(isReceiveData);

        /* 3. 摇杆解锁/上锁 */
        isRemoteUnlocked = App_Communication_RemoteUnlock(isRemoteConnected);

        /* 4. 遥控器发来的关机命令 */
        if(isReceiveData == Com_OK && joyStick.isPowerDown)
        {
            debug_printfln("power off requested by remote");
            joyStick.isPowerDown = 0;
            xTaskNotifyGive(powerTaskHandle);
        }

        vTaskDelayUntil(&preTime, COMMUNICATION_EXEC_CYCLE);
    }
}
