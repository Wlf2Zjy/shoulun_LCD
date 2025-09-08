#include "LVGL_Example.h"
#include "lvgl.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

static const char *DATA_TAG = "DATA";

/**********************
 *  STATIC VARIABLES
 **********************/
// 定义各种样式对象
static lv_style_t style_value_box;          // 值框样式
static lv_style_t style_editable_box;       // 可编辑框样式
static lv_style_t style_editable_box_focus; // 可编辑框焦点样式（高亮）
static lv_style_t style_axis_label;         // 坐标轴标签样式
static lv_style_t style_left_label;         // 左侧标签样式
static lv_style_t style_centering_label;    // 分中标签样式
static lv_style_t style_ok_button;          // OK按钮样式
static lv_style_t style_ok_button_focus;    // OK按钮焦点样式
static lv_style_t style_number_label;       // 数字标签样式
static lv_style_t style_axis_label_small;   // 小号坐标轴标签样式

// 坐标显示对象
static lv_obj_t *mechanical_x;      // 机械坐标X
static lv_obj_t *mechanical_y;      // 机械坐标Y
static lv_obj_t *mechanical_z;      // 机械坐标Z
static lv_obj_t *workpiece_x;       // 工件坐标X
static lv_obj_t *workpiece_y;       // 工件坐标Y
static lv_obj_t *workpiece_z;       // 工件坐标Z
static lv_obj_t *centering_value;   // 分中值显示
static lv_obj_t *centering1_value;  // 分中值1
static lv_obj_t *centering2_value;  // 分中值2
static lv_obj_t *ok_button;         // OK按钮对象

// 轴标签对象
static lv_obj_t *axis_label_small1; // 分中值1轴标签
static lv_obj_t *axis_label_small2; // 分中值2轴标签

static float mechanical_coords[3] = {0.0f, 0.0f, 0.0f}; // 机械坐标X, Y, Z
static float workpiece_coords[3] = {0.0f, 0.0f, 0.0f};  // 工件坐标X, Y, Z

// 外部声明的拨档状态变量（来自编码器代码）
extern volatile int current_axis; // 当前选择的轴 (0=X, 1=Y, 2=Z, 3=A)

// 功能按键状态
volatile bool func_btn_pressed = false;
static int current_focus = 0; // 0: 分中值1, 1: 分中值2

// 防止重复处理标志
static bool processing_func_btn = false;

// UART数据缓冲区
#define UART_BUFFER_SIZE 256
static char uart_buffer[UART_BUFFER_SIZE];
static int uart_buffer_index = 0;

/**********************
 *  STATIC FUNCTIONS
 **********************/

/**
 * @brief 创建所有需要的样式
 * 初始化各种UI元素的样式，包括颜色、边框、字体等属性
 */
static void create_styles(void)
{
    // 值框样式 - 用于显示坐标值的框
    lv_style_init(&style_value_box);
    lv_style_set_border_color(&style_value_box, lv_color_hex(0xCCCCCC)); // 设置边框颜色
    lv_style_set_border_width(&style_value_box, 1);                      // 设置边框宽度
    lv_style_set_radius(&style_value_box, 3);                            // 设置圆角半径
    lv_style_set_bg_color(&style_value_box, lv_color_hex(0xF9F9F9));     // 设置背景颜色
    lv_style_set_text_font(&style_value_box, &lv_font_montserrat_12);    // 设置字体
    lv_style_set_pad_all(&style_value_box, 2);                           // 设置内边距
    lv_style_set_width(&style_value_box, 55);                            // 设置宽度
    lv_style_set_height(&style_value_box, 20);                           // 设置高度

    
    // 可编辑框样式 - 用于可编辑的输入框
    lv_style_init(&style_editable_box);
    lv_style_set_border_color(&style_editable_box, lv_color_hex(0x4CAF50)); // 绿色边框表示可编辑
    lv_style_set_border_width(&style_editable_box, 1);
    lv_style_set_radius(&style_editable_box, 3);
    lv_style_set_bg_color(&style_editable_box, lv_color_hex(0xFFFFFF));     // 白色背景
    lv_style_set_text_font(&style_editable_box, &lv_font_montserrat_12);
    lv_style_set_pad_all(&style_editable_box, 2);
    lv_style_set_width(&style_editable_box, 55);  // 设置宽度
    lv_style_set_height(&style_editable_box, 20); // 设置高度

    // 可编辑框焦点样式（高亮）
    lv_style_init(&style_editable_box_focus);
    lv_style_set_border_color(&style_editable_box_focus, lv_color_hex(0xFF0000)); // 红色边框
    lv_style_set_border_width(&style_editable_box_focus, 2); // 更粗的边框
    lv_style_set_radius(&style_editable_box_focus, 3);
    lv_style_set_bg_color(&style_editable_box_focus, lv_color_hex(0xFFFFFF));
    lv_style_set_text_font(&style_editable_box_focus, &lv_font_montserrat_12);
    lv_style_set_pad_all(&style_editable_box_focus, 2);
    lv_style_set_width(&style_editable_box_focus, 55);
    lv_style_set_height(&style_editable_box_focus, 20);

     // OK按钮焦点样式（高亮）
    lv_style_init(&style_ok_button_focus);
    lv_style_set_radius(&style_ok_button_focus, 9);
    lv_style_set_bg_color(&style_ok_button_focus, lv_color_hex(0x4CAF50)); // 绿色背景
    lv_style_set_text_color(&style_ok_button_focus, lv_color_white()); // 白色文字
    lv_style_set_text_font(&style_ok_button_focus, &lv_font_montserrat_12);
    lv_style_set_pad_all(&style_ok_button_focus, 0);
    lv_style_set_width(&style_ok_button_focus, 220);
    lv_style_set_height(&style_ok_button_focus, 18);
    
    // 轴标签样式 - 用于X、Y、Z轴标签
    lv_style_init(&style_axis_label);
    lv_style_set_text_font(&style_axis_label, &lv_font_montserrat_12);
    lv_style_set_text_color(&style_axis_label, lv_color_black()); // 黑色文字
    lv_style_set_width(&style_axis_label, 12);                    // 设置宽度
    
    // 左侧标签样式 - 用于左侧的"Meth"和"Workpiece"标签
    lv_style_init(&style_left_label);
    lv_style_set_text_font(&style_left_label, &lv_font_montserrat_12);
    lv_style_set_text_color(&style_left_label, lv_color_black());
    lv_style_set_text_font(&style_left_label, &lv_font_montserrat_12);
    
    // 分中标签样式 - 用于分中相关的标签
    lv_style_init(&style_centering_label);
    lv_style_set_text_font(&style_centering_label, &lv_font_montserrat_12);
    lv_style_set_text_color(&style_centering_label, lv_color_black());
    
    // OK按钮样式
    lv_style_init(&style_ok_button);
    lv_style_set_radius(&style_ok_button, 9);                         // 圆角按钮
    lv_style_set_bg_color(&style_ok_button, lv_color_hex(0xF0F0F0));  // 浅灰色背景
    lv_style_set_text_color(&style_ok_button, lv_color_hex(0x141313));// 深灰色文字
    lv_style_set_text_font(&style_ok_button, &lv_font_montserrat_12);
    lv_style_set_pad_all(&style_ok_button, 0);
    lv_style_set_width(&style_ok_button, 240);                        // 设置宽度
    lv_style_set_height(&style_ok_button, 18);                        // 设置高度
    
    // 数字标签样式 - 用于分中值的数字标签
    lv_style_init(&style_number_label);
    lv_style_set_text_font(&style_number_label, &lv_font_montserrat_12);
    lv_style_set_text_color(&style_number_label, lv_color_black());
    lv_style_set_width(&style_number_label, 12);
    
    // 小轴标签样式 - 用于分中部分的小号轴标签
    lv_style_init(&style_axis_label_small);
    lv_style_set_text_font(&style_axis_label_small, &lv_font_montserrat_12);
    lv_style_set_text_color(&style_axis_label_small, lv_color_black());
    lv_style_set_width(&style_axis_label_small, 12);
}


/**
 * @brief 读取拨档状态并返回当前轴字符
 */
static char get_current_axis_char(void)
{
    switch(current_axis) {
        case 0: return 'X';
        case 1: return 'Y';
        case 2: return 'Z';
        case 3: return 'A';
        default: return 'X'; // 默认返回X轴
    }
}

static void parse_coordinates(const char* data)
{
    // 查找MPos和WPos的位置
    const char* mpos_start = strstr(data, "MPos:");
    const char* wpos_start = strstr(data, "WPos:");
    
    if (mpos_start && wpos_start) {
        // 解析机械坐标
        if (sscanf(mpos_start + 5, "%f,%f,%f", 
                  &mechanical_coords[0], 
                  &mechanical_coords[1], 
                  &mechanical_coords[2]) == 3) {
            // 解析成功
        }
        
        // 解析工件坐标
        if (sscanf(wpos_start + 5, "%f,%f,%f", 
                  &workpiece_coords[0], 
                  &workpiece_coords[1], 
                  &workpiece_coords[2]) == 3) {
            // 解析成功
        }
    }
}

/**
 * @brief 处理UART接收到的数据
 */
void process_uart_data(const char* data, int length)
{
    ESP_LOGI(DATA_TAG, "Processing data: %.*s", length, data);
    // 将数据添加到缓冲区
    for (int i = 0; i < length; i++) {
        if (uart_buffer_index < UART_BUFFER_SIZE - 1) {
            uart_buffer[uart_buffer_index++] = data[i];
            
            // 检查是否收到完整的指令帧（以'>'结尾）
            if (data[i] == '>') {
                uart_buffer[uart_buffer_index] = '\0'; // 添加字符串结束符

                // 调试输出：打印接收到的指令帧
               printf("Received: %s\n", uart_buffer);
                
                // 解析坐标数据
                parse_coordinates(uart_buffer);
                
                // 重置缓冲区
                uart_buffer_index = 0;
            }
        } else {
            // 缓冲区溢出，重置
            uart_buffer_index = 0;
        }
    }
}

/**
 * @brief 切换焦点到下一个文本框
 */
static void switch_focus(void)
{
    // 移除当前焦点样式
    switch (current_focus) {
        case 0: // 当前焦点在分中值1文本框
            lv_obj_remove_style(centering1_value, &style_editable_box_focus, 0);
            lv_obj_add_style(centering1_value, &style_editable_box, 0);
            break;
        case 1: // 当前焦点在分中值2文本框
            lv_obj_remove_style(centering2_value, &style_editable_box_focus, 0);
            lv_obj_add_style(centering2_value, &style_editable_box, 0);
            break;
        case 2: // 当前焦点在OK按钮
            lv_obj_remove_style(ok_button, &style_ok_button_focus, 0);
            lv_obj_add_style(ok_button, &style_ok_button, 0);
            break;
    }
    
    // 切换到下一个焦点 (0=分中值1, 1=分中值2, 2=OK按钮)
    current_focus = (current_focus + 1) % 3;
    
    // 应用新焦点样式
    switch (current_focus) {
        case 0: // 焦点切换到分中值1文本框
            lv_obj_remove_style(centering1_value, &style_editable_box, 0);
            lv_obj_add_style(centering1_value, &style_editable_box_focus, 0);
            break;
        case 1: // 焦点切换到分中值2文本框
            lv_obj_remove_style(centering2_value, &style_editable_box, 0);
            lv_obj_add_style(centering2_value, &style_editable_box_focus, 0);
            break;
        case 2: // 焦点切换到OK按钮
            lv_obj_remove_style(ok_button, &style_ok_button, 0);
            lv_obj_add_style(ok_button, &style_ok_button_focus, 0);
            break;
    }
}

/**
 * @brief 定时器回调函数，用于更新显示的数据
 * 根据拨档状态更新界面显示
 * @param timer 定时器对象
 */
static void update_display_cb(lv_timer_t *timer)
{
    // 检查功能按键是否被按下
    if (func_btn_pressed) {
        func_btn_pressed = false;
        switch_focus();
    }

    // 更新显示
    char buf[16];
    
    // 更新机械坐标（使用从UART解析的数据）
    snprintf(buf, sizeof(buf), "%.3f", mechanical_coords[0]);
    lv_label_set_text(mechanical_x, buf);
    
    snprintf(buf, sizeof(buf), "%.3f", mechanical_coords[1]);
    lv_label_set_text(mechanical_y, buf);
    
    snprintf(buf, sizeof(buf), "%.3f", mechanical_coords[2]);
    lv_label_set_text(mechanical_z, buf);
    
    // 更新工件坐标（使用从UART解析的数据）
    snprintf(buf, sizeof(buf), "%.3f", workpiece_coords[0]);
    lv_label_set_text(workpiece_x, buf);
    
    snprintf(buf, sizeof(buf), "%.3f", workpiece_coords[1]);
    lv_label_set_text(workpiece_y, buf);
    
    snprintf(buf, sizeof(buf), "%.3f", workpiece_coords[2]);
    lv_label_set_text(workpiece_z, buf);

    // 根据拨档状态更新轴标签 更新分中值
    char axis_char = get_current_axis_char();
    char axis_buf[2] = {axis_char, '\0'};
    lv_label_set_text(centering_value, axis_buf);

    // 更新分中值1轴标签
    char axis_buf1[2] = {axis_char, '\0'};
    lv_label_set_text(axis_label_small1, axis_buf1);
    
    // 更新分中值2轴标签
    char axis_buf2[2] = {axis_char, '\0'};
    lv_label_set_text(axis_label_small2, axis_buf2);
}
void handle_func_button(void) //功能按键事件处理
{
    func_btn_pressed = true;
}
/**
 * @brief 创建旋转的容器来放置所有界面元素
 * 由于屏幕可能是竖屏，但需要横屏显示，因此创建旋转容器
 * @param parent 父对象
 */
static void create_rotated_container(lv_obj_t *parent)
{
    // 创建主容器 - 使用屏幕的物理尺寸
    lv_obj_t *screen_container = lv_obj_create(parent);
    lv_obj_clear_flag(screen_container, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(screen_container, 172, 320); // 使用物理尺寸172x320
    lv_obj_set_style_bg_opa(screen_container, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(screen_container, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_center(screen_container); // 居中显示
    
    // 创建旋转容器 - 使用逻辑横屏尺寸
    lv_obj_t *rotated_container = lv_obj_create(screen_container);
    lv_obj_clear_flag(rotated_container, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(rotated_container, 320, 172); // 横屏尺寸
    lv_obj_set_style_bg_color(rotated_container, lv_color_white(), 0); // 白色背景
    lv_obj_set_style_border_width(rotated_container, 0, 0); // 无边框
    lv_obj_set_style_radius(rotated_container, 0, 0); // 无圆角
    lv_obj_set_style_pad_all(rotated_container, 5, 0); // 内边距
    lv_obj_set_style_transform_angle(rotated_container, 2700, 0); // 旋转270度（逆时针）
    lv_obj_set_style_transform_pivot_x(rotated_container, 160, 0); // 设置旋转中心X
    lv_obj_set_style_transform_pivot_y(rotated_container, 86, 0); // 设置旋转中心Y
    lv_obj_align(rotated_container, LV_ALIGN_CENTER, 0, 0); // 居中显示
    
    // 创建内部容器（实际内容）
    lv_obj_t *container = lv_obj_create(rotated_container);
    lv_obj_clear_flag(container, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(container, 310, 162); // 设置尺寸
    lv_obj_align(container, LV_ALIGN_CENTER, 0, 0); // 居中显示
    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN); // 设置为列布局
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(container, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(container, 0, 0); // 无内边距
    
    // 顶部区域 - 显示坐标信息
    lv_obj_t *top_section = lv_obj_create(container);
    lv_obj_clear_flag(top_section, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(top_section, 310, 80); // 设置尺寸
    lv_obj_set_flex_flow(top_section, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(top_section, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(top_section, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_border_width(top_section, 0, 0); // 无边框
    lv_obj_set_style_border_side(top_section, LV_BORDER_SIDE_NONE, 0); // 无边框
    lv_obj_set_style_pad_all(top_section, 0, 0); // 无内边距
    
    // 创建左侧标签区域
    lv_obj_t *left_label = lv_obj_create(top_section);
    lv_obj_clear_flag(left_label, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(left_label, 35, 80); // 设置尺寸
    lv_obj_set_flex_flow(left_label, LV_FLEX_FLOW_COLUMN); // 设置为列布局
    lv_obj_set_style_bg_opa(left_label, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(left_label, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_ver(left_label, 8, 0); // 垂直内边距
    lv_obj_set_style_pad_hor(left_label, 0, 0); // 水平内边距
    
    // 创建机械坐标标签
    lv_obj_t *mechanical_label = lv_label_create(left_label);
    lv_obj_clear_flag(mechanical_label, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(mechanical_label, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(mechanical_label, "Meth"); // 设置文本
    lv_obj_add_style(mechanical_label, &style_left_label, 0); // 应用样式
    lv_obj_set_style_pad_row(left_label, 30, 0); // 设置子元素之间的行间距
    
    // 创建工件坐标标签
    lv_obj_t *workpiece_label = lv_label_create(left_label);
    lv_obj_clear_flag(workpiece_label, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(workpiece_label, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(workpiece_label, "Workpiece"); // 设置文本
    lv_obj_add_style(workpiece_label, &style_left_label, 0); // 应用样式
    
    // 创建坐标区域
    lv_obj_t *coordinates = lv_obj_create(top_section);
    lv_obj_clear_flag(coordinates, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(coordinates, 270, 80); // 设置尺寸
    lv_obj_set_flex_flow(coordinates, LV_FLEX_FLOW_COLUMN); // 设置为列布局
    lv_obj_set_style_bg_opa(coordinates, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(coordinates, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(coordinates, 0, 0); // 无内边距
    
    // 第一行坐标 (机械坐标)
    lv_obj_t *coordinate_row1 = lv_obj_create(coordinates);
    lv_obj_clear_flag(coordinate_row1, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(coordinate_row1, 270, 40); // 设置尺寸
    lv_obj_set_flex_flow(coordinate_row1, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(coordinate_row1, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(coordinate_row1, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(coordinate_row1, 0, 0); // 无内边距
    
    lv_obj_t *axis_group1 = lv_obj_create(coordinate_row1);
    lv_obj_clear_flag(axis_group1, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(axis_group1, 270, 40); // 设置尺寸
    lv_obj_set_flex_flow(axis_group1, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(axis_group1, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(axis_group1, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(axis_group1, 0, 0); // 无内边距
    
    // X轴坐标显示
    lv_obj_t *axis_item_x1 = lv_obj_create(axis_group1);
    lv_obj_clear_flag(axis_item_x1, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(axis_item_x1, 80, 40); // 设置尺寸
    lv_obj_set_flex_flow(axis_item_x1, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(axis_item_x1, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(axis_item_x1, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(axis_item_x1, 0, 0); // 无内边距
    
    // X轴标签
    lv_obj_t *axis_label_x1 = lv_label_create(axis_item_x1);
    lv_obj_clear_flag(axis_label_x1, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(axis_label_x1, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(axis_label_x1, "X"); // 设置文本
    lv_obj_add_style(axis_label_x1, &style_axis_label, 0); // 应用样式
    
    // X轴坐标值显示
    mechanical_x = lv_label_create(axis_item_x1);
    lv_label_set_text(mechanical_x, "5.000"); // 设置初始文本
    lv_obj_add_style(mechanical_x, &style_value_box, 0); // 应用样式
    
    // Y轴坐标显示
    lv_obj_t *axis_item_y1 = lv_obj_create(axis_group1);
    lv_obj_clear_flag(axis_item_y1, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(axis_item_y1, 80, 40); // 设置尺寸
    lv_obj_set_flex_flow(axis_item_y1, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(axis_item_y1, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(axis_item_y1, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(axis_item_y1, 0, 0); // 无内边距
    
    // Y轴标签
    lv_obj_t *axis_label_y1 = lv_label_create(axis_item_y1);
    lv_obj_clear_flag(axis_label_y1, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(axis_label_y1, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(axis_label_y1, "Y"); // 设置文本
    lv_obj_add_style(axis_label_y1, &style_axis_label, 0); // 应用样式
    
    // Y轴坐标值显示
    mechanical_y = lv_label_create(axis_item_y1);
    lv_label_set_text(mechanical_y, "5.000"); // 设置初始文本
    lv_obj_add_style(mechanical_y, &style_value_box, 0); // 应用样式
    
    // Z轴坐标显示
    lv_obj_t *axis_item_z1 = lv_obj_create(axis_group1);
    lv_obj_clear_flag(axis_item_z1, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(axis_item_z1, 80, 40); // 设置尺寸
    lv_obj_set_flex_flow(axis_item_z1, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(axis_item_z1, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(axis_item_z1, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(axis_item_z1, 0, 0); // 无内边距
    
    // Z轴标签
    lv_obj_t *axis_label_z1 = lv_label_create(axis_item_z1);
    lv_obj_clear_flag(axis_label_z1, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(axis_label_z1, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(axis_label_z1, "Z"); // 设置文本
    lv_obj_add_style(axis_label_z1, &style_axis_label, 0); // 应用样式
    
    // Z轴坐标值显示
    mechanical_z = lv_label_create(axis_item_z1);
    lv_label_set_text(mechanical_z, "5.000"); // 设置初始文本
    lv_obj_add_style(mechanical_z, &style_value_box, 0); // 应用样式
    
    // 第二行坐标 (工件坐标)
    lv_obj_t *coordinate_row2 = lv_obj_create(coordinates);
    lv_obj_clear_flag(coordinate_row2, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(coordinate_row2, 270, 40); // 设置尺寸
    lv_obj_set_flex_flow(coordinate_row2, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(coordinate_row2, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(coordinate_row2, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(coordinate_row2, 0, 0); // 无内边距
    
    lv_obj_t *axis_group2 = lv_obj_create(coordinate_row2);
    lv_obj_clear_flag(axis_group2, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(axis_group2, 270, 40); // 设置尺寸
    lv_obj_set_flex_flow(axis_group2, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(axis_group2, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(axis_group2, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(axis_group2, 0, 0); // 无内边距
    
    // X轴坐标显示
    lv_obj_t *axis_item_x2 = lv_obj_create(axis_group2);
    lv_obj_clear_flag(axis_item_x2, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(axis_item_x2, 80, 40); // 设置尺寸
    lv_obj_set_flex_flow(axis_item_x2, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(axis_item_x2, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(axis_item_x2, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(axis_item_x2, 0, 0); // 无内边距
    
    // X轴标签
    lv_obj_t *axis_label_x2 = lv_label_create(axis_item_x2);
    lv_obj_clear_flag(axis_label_x2, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(axis_label_x2, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(axis_label_x2, "X"); // 设置文本
    lv_obj_add_style(axis_label_x2, &style_axis_label, 0); // 应用样式
    
    // X轴坐标值显示
    workpiece_x = lv_label_create(axis_item_x2);
    lv_label_set_text(workpiece_x, "6.000"); // 设置初始文本
    lv_obj_add_style(workpiece_x, &style_value_box, 0); // 应用样式
    
    // Y轴坐标显示
    lv_obj_t *axis_item_y2 = lv_obj_create(axis_group2);
    lv_obj_clear_flag(axis_item_y2, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(axis_item_y2, 80, 40); // 设置尺寸
    lv_obj_set_flex_flow(axis_item_y2, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(axis_item_y2, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(axis_item_y2, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(axis_item_y2, 0, 0); // 无内边距
    
    // Y轴标签
    lv_obj_t *axis_label_y2 = lv_label_create(axis_item_y2);
    lv_obj_clear_flag(axis_label_y2, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(axis_label_y2, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(axis_label_y2, "Y"); // 设置文本
    lv_obj_add_style(axis_label_y2, &style_axis_label, 0); // 应用样式
    
    // Y轴坐标值显示
    workpiece_y = lv_label_create(axis_item_y2);
    lv_label_set_text(workpiece_y, "6.000"); // 设置初始文本
    lv_obj_add_style(workpiece_y, &style_value_box, 0); // 应用样式
    
    // Z轴坐标显示
    lv_obj_t *axis_item_z2 = lv_obj_create(axis_group2);
    lv_obj_clear_flag(axis_item_z2, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(axis_item_z2, 80, 40); // 设置尺寸
    lv_obj_set_flex_flow(axis_item_z2, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(axis_item_z2, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(axis_item_z2, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(axis_item_z2, 0, 0); // 无内边距
    
    // Z轴标签
    lv_obj_t *axis_label_z2 = lv_label_create(axis_item_z2);
    lv_obj_clear_flag(axis_label_z2, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(axis_label_z2, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(axis_label_z2, "Z"); // 设置文本
    lv_obj_add_style(axis_label_z2, &style_axis_label, 0); // 应用样式
    
    // Z轴坐标值显示
    workpiece_z = lv_label_create(axis_item_z2);
    lv_label_set_text(workpiece_z, "6.000"); // 设置初始文本
    lv_obj_add_style(workpiece_z, &style_value_box, 0); // 应用样式
    
    // 创建底部区域 - 分中功能区域
    lv_obj_t *bottom_section = lv_obj_create(container);
    lv_obj_clear_flag(bottom_section, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(bottom_section, 310, 75); // 设置尺寸
    lv_obj_set_flex_flow(bottom_section, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(bottom_section, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(bottom_section, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_top(bottom_section, 3, 0); // 顶部内边距
    lv_obj_set_style_pad_hor(bottom_section, 0, 0); // 水平内边距
    lv_obj_set_style_pad_bottom(bottom_section, 0, 0); // 底部内边距
    
    // 创建左侧分中区域
    lv_obj_t *centering_left = lv_obj_create(bottom_section);
    lv_obj_clear_flag(centering_left, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(centering_left, 32, 75); // 设置尺寸
    lv_obj_set_flex_flow(centering_left, LV_FLEX_FLOW_COLUMN); // 设置为列布局
    lv_obj_set_style_bg_opa(centering_left, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(centering_left, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(centering_left, 0, 0); // 无内边距
    lv_obj_set_flex_align(centering_left, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START); // 设置对齐方式
    
    // 分中标签
    lv_obj_t *centering_label = lv_label_create(centering_left);
    lv_obj_clear_flag(centering_label, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(centering_label, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(centering_label, "BrCe"); // 设置文本
    lv_obj_add_style(centering_label, &style_centering_label, 0); // 应用样式
    lv_obj_set_style_text_align(centering_label, LV_TEXT_ALIGN_CENTER, 0); // 文本居中
    //lv_obj_set_style_pad_column(centering_left, 10, 0); // 设置子元素之间的列间距
    
    // 分中值显示
    centering_value = lv_label_create(centering_left);
    char initial_axis_char[2] = {get_current_axis_char(), '\0'};
    lv_label_set_text(centering_value, initial_axis_char); // 设置初始文本为轴字符
    lv_obj_set_size(centering_value, 25, 25); // 设置尺寸
    lv_obj_add_style(centering_value, &style_value_box, 0); // 应用样式
    lv_obj_set_style_text_align(centering_value, LV_TEXT_ALIGN_CENTER, 0); // 文本居中
    
    // 创建右侧分中区域
    lv_obj_t *centering_right = lv_obj_create(bottom_section);
    lv_obj_clear_flag(centering_right, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(centering_right, 270, 75); // 设置尺寸
    lv_obj_set_flex_flow(centering_right, LV_FLEX_FLOW_COLUMN); // 设置为列布局
    lv_obj_set_style_bg_opa(centering_right, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(centering_right, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(centering_right, 0, 0); // 无内边距
    
    // 第一行分中值
    lv_obj_t *centering_row = lv_obj_create(centering_right);
    lv_obj_clear_flag(centering_row, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(centering_row, 270, 25); // 设置尺寸
    lv_obj_set_flex_flow(centering_row, LV_FLEX_FLOW_ROW); // 设置为行布局
    lv_obj_set_style_bg_opa(centering_row, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(centering_row, LV_OPA_TRANSP, 0); // 透明边框
    lv_obj_set_style_pad_all(centering_row, 0, 0); // 无内边距
    lv_obj_set_style_pad_bottom(centering_row, 6, 0); // 底部内边距
    
    // 分中值1标签
    lv_obj_t *number_label1 = lv_label_create(centering_row);
    lv_obj_clear_flag(number_label1, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(number_label1, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(number_label1, "1"); // 设置文本
    lv_obj_add_style(number_label1, &style_number_label, 0); // 应用样式
    
     // 分中值1轴标签
    axis_label_small1 = lv_label_create(centering_row);
    lv_obj_clear_flag(axis_label_small1, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(axis_label_small1, LV_OBJ_FLAG_CLICKABLE);
    char initial_axis1[2] = {get_current_axis_char(), '\0'};
    lv_label_set_text(axis_label_small1, initial_axis1); // 初始值为当前轴字符
    lv_obj_add_style(axis_label_small1, &style_axis_label_small, 0);
    
    // 分中值1输入框
    centering1_value = lv_textarea_create(centering_row);
    lv_textarea_set_text(centering1_value, "0.000"); // 设置初始文本
    lv_textarea_set_one_line(centering1_value, true); // 单行模式
    lv_obj_add_style(centering1_value, &style_editable_box, 0); // 应用样式
    
    // 添加间距
    lv_obj_t *spacer = lv_obj_create(centering_row);
    lv_obj_clear_flag(spacer, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_set_size(spacer, 40, 20); // 设置尺寸
    lv_obj_set_style_bg_opa(spacer, LV_OPA_TRANSP, 0); // 透明背景
    lv_obj_set_style_border_opa(spacer, LV_OPA_TRANSP, 0); // 透明边框
    
    // 分中值2标签
    lv_obj_t *number_label2 = lv_label_create(centering_row);
    lv_obj_clear_flag(number_label2, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(number_label2, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(number_label2, "2"); // 设置文本
    lv_obj_add_style(number_label2, &style_number_label, 0); // 应用样式
    
    axis_label_small2 = lv_label_create(centering_row);
    lv_obj_clear_flag(axis_label_small2, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(axis_label_small2, LV_OBJ_FLAG_CLICKABLE);
    char initial_axis2[2] = {get_current_axis_char(), '\0'};
    lv_label_set_text(axis_label_small2, initial_axis2); // 初始值为当前轴字符
    lv_obj_add_style(axis_label_small2, &style_axis_label_small, 0);
    
    // 分中值2输入框
    centering2_value = lv_textarea_create(centering_row);
    lv_textarea_set_text(centering2_value, "0.000"); // 设置初始文本
    lv_textarea_set_one_line(centering2_value, true); // 单行模式
    lv_obj_add_style(centering2_value, &style_editable_box, 0); // 应用样式
    
    // OK按钮
    lv_obj_t *ok_button = lv_btn_create(centering_right);
    lv_obj_clear_flag(ok_button, LV_OBJ_FLAG_SCROLLABLE); // 禁用滚动条
    lv_obj_add_style(ok_button, &style_ok_button, 0); // 应用样式
    //lv_obj_add_event_cb(ok_button, ok_button_event_cb, LV_EVENT_ALL, NULL); // 添加事件回调
    lv_obj_set_style_bg_opa(ok_button, LV_OPA_COVER, 0); // 背景不透明
    lv_obj_set_style_border_opa(ok_button, LV_OPA_TRANSP, 0); // 透明边框
    
    // OK按钮标签
    lv_obj_t *ok_label = lv_label_create(ok_button);
    lv_obj_clear_flag(ok_label, LV_OBJ_FLAG_CLICKABLE); // 非点击able
    lv_obj_add_flag(ok_label, LV_OBJ_FLAG_CLICKABLE); // 设置为可点击
    lv_label_set_text(ok_label, "Branch Center"); // 设置文本
    lv_obj_center(ok_label); // 居中显示
    
    // 创建定时器更新显示
    lv_timer_create(update_display_cb, 100, NULL);
}

/**
 * @brief 初始化函数
 * 创建样式和旋转界面
 */
void coordinate_display_init(void)
{
    // 创建样式
    create_styles();
    
    // 创建旋转的界面
    create_rotated_container(lv_scr_act());

    // 设置初始焦点
    current_focus = 0;
    lv_obj_add_style(centering1_value, &style_editable_box_focus, 0);
}