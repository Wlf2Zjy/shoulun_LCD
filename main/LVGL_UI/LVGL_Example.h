#pragma once

#include "lvgl.h"
#include "demos/lv_demos.h"
#include "LVGL_Driver.h"

#define EXAMPLE1_LVGL_TICK_PERIOD_MS  1000

extern volatile int current_axis;  // 声明外部变量
extern volatile bool func_btn_pressed;  // 声明功能按键按下标志变量

void coordinate_display_init(void);


