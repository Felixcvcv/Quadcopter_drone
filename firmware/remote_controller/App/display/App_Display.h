#ifndef __APP_DISPLAY_H
#define __APP_DISPLAY_H
#include "Com_Debug.h"
#include "oled.h"
#include "oledfont.h"
#include "Inf_Si24R1.h"
#include "Inf_JoyStickAndKey.h"

/* OLED 是 128x64, 每行 12 像素高, 共 5 行 */
#define Y0 (0)
#define Y1 (14)
#define Y2 (Y1 + 12) /* 26 */
#define Y3 (Y2 + 12) /* 38 */

/* 各行的水平起始位置(居中排布) */
#define Line1_Begin (29)
#define Line3_Begin (5)
#define Line4_Begin (30)

void App_Display_Start(void);

void App_Display_Show(void);

#endif
