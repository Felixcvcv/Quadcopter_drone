#ifndef __APP_TASK_H
#define __APP_TASK_H
#include "Com_Debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "Inf_IP5305T.h"
#include "Inf_LED.h"
#include "App_Flight.h"
#include "App_Communication.h"






void App_Task_FreeRTOSStart(void);
#endif
