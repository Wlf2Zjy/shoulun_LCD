#pragma once

#include "lvgl.h"
#include "demos/lv_demos.h"
#include "LVGL_Driver.h"

#define EXAMPLE1_LVGL_TICK_PERIOD_MS  1000

extern volatile int current_axis;  // 声明外部变量(拨档)

// 功能按键状态枚举
typedef enum {
    FUNC_BTN_STATE_CENTERING1 = 0,  // 分中值1轴
    FUNC_BTN_STATE_CENTERING2 = 1,  // 分中值2轴
    FUNC_BTN_STATE_OK = 2           // OK键
} func_btn_state_t;

// 功能按键状态变量
extern volatile func_btn_state_t func_btn_current_state;
extern volatile bool func_btn_pressed;

// 分中值输入框对象
extern lv_obj_t *centering1_value;
extern lv_obj_t *centering2_value;

void coordinate_display_init(void);

// 当功能按键状态或相关UI需刷新时调用，更新高亮与轴标签
void ui_update_on_state_change(void);

// 更新分中值输入框
void update_centering_values(void);

// 获取当前轴索引
int get_current_axis_index(void);

void update_mechanical_coords(float x, float y, float z);  // 更新机械坐标
void update_workpiece_coords(float x, float y, float z);  // 更新工件坐标
void update_centering_values(void);
void ui_update_on_state_change(void);

// 添加函数声明
void request_axis_labels_update(void);
void check_and_update_axis_labels(void);

