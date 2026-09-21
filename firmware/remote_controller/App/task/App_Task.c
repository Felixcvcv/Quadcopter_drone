#include "App_Task.h"

#define FW_NAME    "Quadcopter Remote Controller"
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

/* 2. 通讯任务: 把摇杆数据发出去 */
static void communicationTask(void *args);
#define COMMUNICATION_TASK_NAME     "comm_task"
#define COMMUNICATION_TASK_STACK    (256)
#define COMMUNICATION_TASK_PRIORITY (8)
static TaskHandle_t communicationTaskHandle;
#define COMMUNICATION_EXEC_CYCLE 6

/* 3. 按键任务 */
static void keyTask(void *args);
#define KEY_TASK_NAME     "key_task"
#define KEY_TASK_STACK    (256)
#define KEY_TASK_PRIORITY (7)
static TaskHandle_t keyTaskHandle;
#define KEY_EXEC_CYCLE 50

/* 4. 摇杆采样任务 */
static void joyStickTask(void *args);
#define JOY_STICK_TASK_NAME     "joystick_task"
#define JOY_STICK_TASK_STACK    (256)
#define JOY_STICK_TASK_PRIORITY (7)
static TaskHandle_t joyStickTaskHandle;
#define JOY_STICK_EXEC_CYCLE 4

/* 5. 显示任务 */
static void displayTask(void *args);
#define DISPLAY_TASK_NAME     "display_task"
#define DISPLAY_TASK_STACK    (256)
#define DISPLAY_TASK_PRIORITY (7)
static TaskHandle_t displayTaskHandle;
#define DISPLAY_EXEC_CYCLE 6

/* 任务启动前的等待: 给射频模块和ADC留出自检/稳定的时间 */
#define COMM_STARTUP_DELAY_MS     (1000)
#define JOYSTICK_STARTUP_DELAY_MS (500)

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

    /* 2. 启动通讯模块 */
    App_Communication_Start();

    /* 3. 启动数据处理模块(摇杆/按键) */
    App_DataProcess_Start();

    /* 4. 启动显示模块 */
    App_Display_Start();

    /* 5. 创建启动任务, 由它在调度器运行起来之后创建其余业务任务 */
    App_Task_Create(startTask,
                    START_TASK_NAME,
                    START_TASK_STACK,
                    START_TASK_PRIORITY,
                    &startTaskHandle);

    /* 6. 启动调度器(正常情况下不会返回) */
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

    /* 2. 通讯任务 */
    App_Task_Create(communicationTask,
                    COMMUNICATION_TASK_NAME,
                    COMMUNICATION_TASK_STACK,
                    COMMUNICATION_TASK_PRIORITY,
                    &communicationTaskHandle);

    /* 3. 按键任务 */
    App_Task_Create(keyTask, KEY_TASK_NAME, KEY_TASK_STACK, KEY_TASK_PRIORITY, &keyTaskHandle);

    /* 4. 摇杆采样任务 */
    App_Task_Create(joyStickTask,
                    JOY_STICK_TASK_NAME,
                    JOY_STICK_TASK_STACK,
                    JOY_STICK_TASK_PRIORITY,
                    &joyStickTaskHandle);

    /* 5. 显示任务 */
    App_Task_Create(displayTask,
                    DISPLAY_TASK_NAME,
                    DISPLAY_TASK_STACK,
                    DISPLAY_TASK_PRIORITY,
                    &displayTaskHandle);

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

/* 2. 通讯任务 */
static void communicationTask(void *args)
{
    vTaskDelay(COMM_STARTUP_DELAY_MS);
    debug_printfln("comm_task running");

    uint32_t preTime = xTaskGetTickCount();

    while(1)
    {
        App_Communication_SendJoyStickData();
        vTaskDelayUntil(&preTime, COMMUNICATION_EXEC_CYCLE);
    }
}

/* 3. 按键任务 */
static void keyTask(void *args)
{
    debug_printfln("key_task running");

    uint32_t preTime = xTaskGetTickCount();

    while(1)
    {
        App_DataProcess_KeyDataProcess();
        vTaskDelayUntil(&preTime, KEY_EXEC_CYCLE);
    }
}

/* 4. 摇杆采样任务 */
static void joyStickTask(void *args)
{
    vTaskDelay(JOYSTICK_STARTUP_DELAY_MS);
    debug_printfln("joystick_task running");

    uint32_t preTime = xTaskGetTickCount();

    while(1)
    {
        App_DataProcess_JoyStickDataProcess();
        vTaskDelayUntil(&preTime, JOY_STICK_EXEC_CYCLE);
    }
}

/* 5. 显示任务 */
static void displayTask(void *args)
{
    debug_printfln("display_task running");

    uint32_t preTime = xTaskGetTickCount();

    while(1)
    {
        App_Display_Show();
        vTaskDelayUntil(&preTime, DISPLAY_EXEC_CYCLE);
    }
}
