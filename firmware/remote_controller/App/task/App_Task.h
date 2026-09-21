#ifndef __APP_TASK_H
#define __APP_TASK_H
#include "Com_Debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "Inf_IP5305T.h"

#include "App_Communication.h"
#include "App_DataProcess.h"
#include "App_Display.h"

void App_Task_FreeRTOSStart(void);
#endif
