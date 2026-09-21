#ifndef __APP_COMMUNICATION_H
#define __APP_COMMUNICATION_H
#include "Com_Debug.h"
#include "Inf_Si24R1.h"
#include "Com_Config.h"
#include "FreeRTOS.h"
#include "task.h"

void App_Communication_Start(void);
void App_Communication_SendJoyStickData(void);
#endif
