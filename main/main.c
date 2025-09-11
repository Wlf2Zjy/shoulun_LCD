#include "ST7789.h"
#include "LVGL_UI/LVGL_Example.h"

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"   // FreeRTOS实时操作系统核心库
#include "freertos/task.h"
#include "driver/gpio.h"         // ESP32 GPIO驱动程序
#include "driver/pcnt.h"        // ESP32脉冲计数器驱动程序
#include "esp_log.h"
#include <math.h>
#include "driver/uart.h"         // UART 驱动
#include <string.h>

#define ENCODER_A GPIO_NUM_1  //A相接开发板1
#define ENCODER_B GPIO_NUM_0  //B相接开发板2，地是3，电压是4
#define LEFT_SW1 GPIO_NUM_4    // X轴选择开关(开发板上的2，拨档的10)以下拨档仅限5个档位那款
#define LEFT_SW2 GPIO_NUM_5    // Y轴选择开关（开发板1，拨档的11）
#define LEFT_SW3 GPIO_NUM_3    // Z轴选择开关（开发板的3，拨档的13）
#define LEFT_SW4 GPIO_NUM_2    // A轴选择开关（开发板的4，拨档的14）
#define RIGHT_SW1 GPIO_NUM_9  //0.1倍（开发板1，拨档14）
#define RIGHT_SW2 GPIO_NUM_18  //1倍（开发板2，拨档16）
#define RIGHT_SW3 GPIO_NUM_19  //5倍（开发板3，拨档17）
#define UART_PORT_NUM UART_NUM_0
#define UART_TX_PIN GPIO_NUM_16  // TX接开发板1
#define UART_RX_PIN GPIO_NUM_17  // RX接开发板2
#define UART_BAUD_RATE 115200
#define ESTOP_PIN GPIO_NUM_23  // 紧急停止按钮(开发板接ON接1，c接地)
#define ESTOP_DEBOUNCE_MS 50    // 防抖时间 50ms
#define FUNC_BTN_PIN GPIO_NUM_20  // 功能按键
#define FUNC_BTN_DEBOUNCE_MS 500 // 功能键防抖 500ms

// 全局变量声明
static const char *TAG = "ENCODER";
static pcnt_unit_t pcnt_unit = PCNT_UNIT_0;
static volatile bool estop_triggered = false;
static void IRAM_ATTR estop_isr_handler(void* arg);
static void IRAM_ATTR func_btn_isr_handler(void* arg);
static volatile uint32_t last_estop_tick = 0;
static volatile uint32_t last_func_btn_tick = 0;
static volatile float axis_counts[4] = {0, 0, 0, 0};
static volatile float axis_last_report[4] = {0, 0, 0, 0};
volatile int current_axis = 0;
static volatile float right_multiplier = 1.0f;

// 功能按键状态变量
volatile func_btn_state_t func_btn_current_state = FUNC_BTN_STATE_CENTERING1;
volatile bool func_btn_pressed = false;

#define COORDINATE_BUFFER_SIZE 128
static char coordinate_buffer[COORDINATE_BUFFER_SIZE];
static int coordinate_buffer_index = 0;
static volatile bool coordinate_updated = false;
static float received_mechanical_coords[4] = {0, 0, 0, 0}; // X, Y, Z, A
static float received_workpiece_coords[4] = {0, 0, 0, 0};  // X, Y, Z, A

// ==================== 编码器初始化 ====================
static void encoder_init(void) {
    pcnt_config_t pcnt_config = {
.pulse_gpio_num = ENCODER_A,  // 脉冲输入引脚
        .ctrl_gpio_num = ENCODER_B,   // 方向控制引脚
        .lctrl_mode = PCNT_MODE_KEEP, // 控制线低电平时计数模式保持不变
        .hctrl_mode = PCNT_MODE_REVERSE, // 控制线高电平时反转计数方向
        .pos_mode = PCNT_COUNT_INC,   // 正向计数模式（递增）
        .neg_mode = PCNT_COUNT_DEC,   // 反向计数模式（递减）
        .counter_h_lim = 32767,       // 计数器上限（16位有符号整数）
        .counter_l_lim = -32768,      // 计数器下限（16位有符号整数）
        .unit = pcnt_unit,            // 使用的计数器单元
        .channel = PCNT_CHANNEL_0      // 使用的计数器通道
    };

    pcnt_unit_config(&pcnt_config);   //初始化计数器
    pcnt_set_filter_value(pcnt_unit, 1000); // 硬件滤波，防止抖动
    pcnt_filter_enable(pcnt_unit);
    pcnt_counter_pause(pcnt_unit);
    pcnt_counter_clear(pcnt_unit);
    pcnt_counter_resume(pcnt_unit);
}

// ==================== UART 初始化 ====================
static void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT_NUM, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// ==================== UART 接收配置 ====================
static void uart_receive_config(void) {
    // 配置UART接收参数
    uart_param_config(UART_PORT_NUM, &(uart_config_t){
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    });
    
    // 安装UART驱动，设置接收缓冲区
    uart_driver_install(UART_PORT_NUM, 1024, 1024, 10, NULL, 0);
}

// ==================== 接收坐标解析函数 ====================
static bool parse_coordinate_frame(const char* buffer) {
    // 查找MPos和WPos字段
    const char* mpos_start = strstr(buffer, "MPos:");
    const char* wpos_start = strstr(buffer, "WPos:");
    
    if (!mpos_start || !wpos_start) {
        return false;
    }
    
    // 解析机械坐标
    if (sscanf(mpos_start, "MPos:%f,%f,%f,%f", 
               &received_mechanical_coords[0], 
               &received_mechanical_coords[1], 
               &received_mechanical_coords[2], 
               &received_mechanical_coords[3]) != 4) {
        return false;
    }
    
    // 解析工件坐标
    if (sscanf(wpos_start, "WPos:%f,%f,%f,%f", 
               &received_workpiece_coords[0], 
               &received_workpiece_coords[1], 
               &received_workpiece_coords[2], 
               &received_workpiece_coords[3]) != 4) {
        return false;
    }
    
    return true;
}

// ==================== UART 接收任务 ====================
static void uart_receive_task(void *arg) {
    uint8_t data[128];
    int length = 0;
    
    while (1) {
        // 读取UART数据
        length = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1, 20 / portTICK_PERIOD_MS);
        
        if (length > 0) {
            data[length] = '\0'; // 添加字符串结束符
            
            // 处理接收到的每个字符
            for (int i = 0; i < length; i++) {
                // 如果遇到帧开始符 '<'
                if (data[i] == '<') {
                    coordinate_buffer_index = 0;
                    coordinate_buffer[coordinate_buffer_index++] = data[i];
                } 
                // 如果遇到帧结束符 '>'
                else if (data[i] == '>' && coordinate_buffer_index > 0) {
                    coordinate_buffer[coordinate_buffer_index++] = data[i];
                    coordinate_buffer[coordinate_buffer_index] = '\0'; // 确保字符串结束
                    
                    // 解析坐标帧
                    if (parse_coordinate_frame(coordinate_buffer)) {
                        coordinate_updated = true;
                    }
                    
                    coordinate_buffer_index = 0; // 重置缓冲区索引
                } 
                // 如果正在收集帧数据
                else if (coordinate_buffer_index > 0 && coordinate_buffer_index < COORDINATE_BUFFER_SIZE - 1) {
                    coordinate_buffer[coordinate_buffer_index++] = data[i];
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== 功能按键初始化 ====================
static void func_btn_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,  // 下降沿触发（按下）
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << FUNC_BTN_PIN),
        .pull_up_en = 1,   // 开启上拉
        .pull_down_en = 0
    };
    gpio_config(&io_conf);

    // 注册中断
    gpio_isr_handler_add(FUNC_BTN_PIN, func_btn_isr_handler, NULL);
}

// ==================== 急停GPIO初始化 ====================
static void estop_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,  // 上升沿+下降沿都触发
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ESTOP_PIN),
        .pull_up_en = 1,   // 开启上拉
        .pull_down_en = 0
    };
    gpio_config(&io_conf);

    // 安装GPIO中断服务
    gpio_install_isr_service(0);
    // 注册中断处理函数
    gpio_isr_handler_add(ESTOP_PIN, estop_isr_handler, NULL);
}

// ==================== 急停中断服务函数 ====================
static void IRAM_ATTR estop_isr_handler(void* arg) {
    uint32_t now_tick = xTaskGetTickCountFromISR();
    if ((now_tick - last_estop_tick) < pdMS_TO_TICKS(ESTOP_DEBOUNCE_MS)) {
        return;  // 在防抖时间内，忽略
    }
    last_estop_tick = now_tick;

    int level = gpio_get_level(ESTOP_PIN);

    if (level == 0) {  
        // 按钮按下（假设低电平有效）
        estop_triggered = true;
        const char stop_cmd = 0x18;  // GRBL 急停指令
        uart_write_bytes(UART_PORT_NUM, &stop_cmd, 1);
        //ESP_EARLY_LOGW(TAG, "急停触发，发送0x18");
    } else {  
        // 按钮松开
        estop_triggered = false;
        const char *unlock_cmd = "$X\n";  // GRBL 解锁指令
        uart_write_bytes(UART_PORT_NUM, unlock_cmd, strlen(unlock_cmd));
        //ESP_EARLY_LOGI(TAG, "急停解除，发送$X");
    }
}

// ==================== 功能按键中断回调 ====================
static void IRAM_ATTR func_btn_isr_handler(void* arg) {
    // 中断级防抖
    uint32_t now_tick = xTaskGetTickCountFromISR();
    if ((now_tick - last_func_btn_tick) < pdMS_TO_TICKS(FUNC_BTN_DEBOUNCE_MS)) {
        return;
    }
    last_func_btn_tick = now_tick;
    // 设置功能按键按下标志，供LVGL界面处理
    func_btn_pressed = true;
}

// ==================== 长按检测 ====================
static bool is_func_btn_long_pressed(void) {
    // 检测功能按键是否长按2秒
    uint32_t press_start_time = xTaskGetTickCount();
    while (gpio_get_level(FUNC_BTN_PIN) == 0) {  // 按键被按下
        vTaskDelay(pdMS_TO_TICKS(50));  // 每50ms检查一次
        if ((xTaskGetTickCount() - press_start_time) >= pdMS_TO_TICKS(2000)) {  // 长按2秒
            return true;
        }
    }
    return false;
}

// ==================== 发送指令帧 ====================
static void send_command_frame(float scaled_steps, int axis_index) {
    if (estop_triggered) {
        // 急停状态下不再发送运动指令
        return;
    }
    char cmd[128];
    float x = 0, y = 0, z = 0,A = 0;
    float feedrate = 3000.0f;   // 默认移动速度

    switch (axis_index) {
        case 0: x = scaled_steps; break;
        case 1: y = scaled_steps; break;
        case 2: z = scaled_steps; break;
        case 3: A = scaled_steps; break; 
    }

    snprintf(cmd, sizeof(cmd), "$J=G21G91X%.2fY%.2fZ%.2fA%.2fF%.1f\n",
             x, y, z, A,feedrate);

    uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
    //ESP_LOGI(TAG, "发送指令: %s", cmd);
}

// ==================== 发送中点指令帧 ====================
static void send_midpoint_command_frame(float midpoint, int axis_index) {
    if (estop_triggered) {
        // 急停状态下不再发送运动指令
        return;
    }
    char cmd[128];
    char axis_char;
    
    // 根据轴索引确定轴字符
    switch (axis_index) {
        case 0: axis_char = 'X'; break;
        case 1: axis_char = 'Y'; break;
        case 2: axis_char = 'Z'; break;
        case 3: axis_char = 'A'; break;
        default: axis_char = 'X'; break;
    }
    
    // 生成指令帧，格式为"G10 L2 P1 [轴][中点]"
    snprintf(cmd, sizeof(cmd), "G10 L2 P1 %c%.3f\n", axis_char, midpoint);
    
    uart_write_bytes(UART_PORT_NUM, cmd, strlen(cmd));
    //ESP_LOGI(TAG, "发送中点指令: %s", cmd);
}


// ==================== 拨档初始化 ====================
static void switch_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,  // 禁用中断
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,   // 启用上拉
        .pull_down_en = 0  // 禁用下拉
    };

    // 左拨档
    io_conf.pin_bit_mask = (1ULL << LEFT_SW1) | (1ULL << LEFT_SW2) |
                           (1ULL << LEFT_SW3) | (1ULL << LEFT_SW4);
    gpio_config(&io_conf);

    // 右拨档
    io_conf.pin_bit_mask = (1ULL << RIGHT_SW1) | (1ULL << RIGHT_SW2) |
                           (1ULL << RIGHT_SW3);
    gpio_config(&io_conf);
}

// ==================== 拨档读取（原始值） ====================
static char read_left_switch_raw(void) {
    if (gpio_get_level(LEFT_SW1) == 0) return 'X';
    if (gpio_get_level(LEFT_SW2) == 0) return 'Y';
    if (gpio_get_level(LEFT_SW3) == 0) return 'Z';
    if (gpio_get_level(LEFT_SW4) == 0) return 'A';
    return 0;  //无开关按下
}

static float read_right_switch_raw(void) {
    if (gpio_get_level(RIGHT_SW1) == 0) return 0.1f;
    if (gpio_get_level(RIGHT_SW2) == 0) return 1.0f;
    if (gpio_get_level(RIGHT_SW3) == 0) return 5.0f;
    return 0.0f;
}

// ==================== 防抖函数宏 ====================
#define SWITCH_SAMPLES 5     // 采样次数
#define SWITCH_DELAY_MS 10   // 每次采样间隔（毫秒）

#define DEFINE_SWITCH_STABLE(type) \
static type read_switch_stable_##type(type (*read_func)(void), type last_value) { \
    type stable_value = last_value; \
    int count_same = 0; \
    for (int i = 0; i < SWITCH_SAMPLES; i++) { \
        type val = read_func(); \
        if (val == stable_value) { \
            count_same++; \
        } else { \
            stable_value = val; \
            count_same = 1; \
        } \
        vTaskDelay(pdMS_TO_TICKS(SWITCH_DELAY_MS)); \
    } \
    return (count_same >= 3) ? stable_value : last_value; \
}
/*拨档档位的防抖，只有大于三次是同一个档位才输出*/
DEFINE_SWITCH_STABLE(char)
DEFINE_SWITCH_STABLE(float)


// ==================== 编码器轮询任务 ====================
//每20ms检测一次，有位移就输出，没有位移就不输出
static void encoder_poll_task(void *arg) {
    int16_t raw_count = 0;
    const char axis_names[4] = {'X', 'Y', 'Z', 'A'};
    while (1) {
        // 获取当前脉冲计数值
        pcnt_get_counter_value(pcnt_unit, &raw_count);
        
        // 如果有脉冲，处理并输出增量
        if (raw_count != 0) {
            // // 除以2.0f是因为每个完整周期有2个脉冲（A相和B相）
            float scaled_steps = (raw_count / 2.0f) * right_multiplier;
            
            // 累加到当前轴的总位移
            axis_counts[current_axis] += scaled_steps;
            
            // 输出增量信息
            //ESP_LOGI(TAG, "轴: %c, 倍率: ×%.1f, 增量: %.2f", 
                    // axis_names[current_axis], right_multiplier, scaled_steps);
            // 发送指令帧
                    send_command_frame(scaled_steps, current_axis);
                    axis_last_report[current_axis] = axis_counts[current_axis];
            // 清除计数器
            pcnt_counter_clear(pcnt_unit);
        }
        
        // 等待20ms进行下一次检测
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==================== 拨档任务（更新轴和倍率） ====================
static void switch_task(void *arg) {
    char left_pos = 0;
    float right_pos = 1.0f;
    //bool initialized = false; // 添加初始化标志
    while (1) {
        char new_left = read_switch_stable_char(read_left_switch_raw, left_pos);
        float new_right = read_switch_stable_float(read_right_switch_raw, right_pos);

        if (new_left != left_pos || new_right != right_pos) {
            if (new_left != 0) {
                left_pos = new_left;
                switch (new_left) {
                    case 'X': current_axis = 0; break;
                    case 'Y': current_axis = 1; break;
                    case 'Z': current_axis = 2; break;
                    case 'A': current_axis = 3; break;
                }
                //ESP_LOGI(TAG, "切换到轴: %c", new_left);
            }
            if (new_right != 0.0f) {
                right_pos = new_right;
                right_multiplier = right_pos;
                //ESP_LOGI(TAG, "倍率调整: ×%.1f", right_pos);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==================== 功能按钮任务 ====================
static void main_loop_task(void *arg) {
    while (1) {
        // 检查是否有新的坐标数据需要更新
        if (coordinate_updated) {
            coordinate_updated = false;
            
            // 更新机械坐标显示 (只使用XYZ，忽略A轴)
            update_mechanical_coords(
                received_mechanical_coords[0],
                received_mechanical_coords[1],
                received_mechanical_coords[2]
            );
            
            // 更新工件坐标显示 (只使用XYZ，忽略A轴)
            update_workpiece_coords(
                received_workpiece_coords[0],
                received_workpiece_coords[1],
                received_workpiece_coords[2]
            );
        }
        
        if (func_btn_pressed) {
            func_btn_pressed = false;  // 清除按键标志
            
            // 检测长按
            if (is_func_btn_long_pressed()) {
                // 更新分中值输入框
                update_centering_values();
                
                // 根据当前状态更新坐标
                if (func_btn_current_state == FUNC_BTN_STATE_CENTERING1) {
                    // 更新机械坐标（使用接收到的坐标值）
                    update_mechanical_coords(
                        received_mechanical_coords[0],
                        received_mechanical_coords[1],
                        received_mechanical_coords[2]
                    );
                } else if (func_btn_current_state == FUNC_BTN_STATE_CENTERING2) {
                    // 更新工件坐标（使用接收到的坐标值）
                    update_workpiece_coords(
                        received_workpiece_coords[0],
                        received_workpiece_coords[1],
                        received_workpiece_coords[2]
                    );
                } else if (func_btn_current_state == FUNC_BTN_STATE_OK) {
                    // 获取分中值1和分中值2的文本内容
                    const char *centering1_text = lv_textarea_get_text(centering1_value);
                    const char *centering2_text = lv_textarea_get_text(centering2_value);
                    
                    // 将文本内容转换为浮点数
                    float centering1_val = atof(centering1_text);
                    float centering2_val = atof(centering2_text);
                    
                    // 计算中点
                    float midpoint = (centering1_val + centering2_val) / 2.0f;
                    
                    // 发送中点指令帧
                    send_midpoint_command_frame(midpoint, current_axis);
                }
            }
            else {
                // 短按，切换状态
                func_btn_current_state = (func_btn_current_state + 1) % 3;
                // 刷新UI高亮
                ui_update_on_state_change();
                
                // 输出调试信息
                switch (func_btn_current_state) {
                case FUNC_BTN_STATE_CENTERING1:
                       // ESP_LOGI(TAG, "功能按键: 切换到分中值1轴");
                        break;
                    case FUNC_BTN_STATE_CENTERING2:
                        //ESP_LOGI(TAG, "功能按键: 切换到分中值2轴");
                        break;
                    case FUNC_BTN_STATE_OK:
                       // ESP_LOGI(TAG, "功能按键: 切换到OK键");
                        break;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();
    }
}
void app_main(void)
{
    ESP_LOGI(TAG, "初始化旋转编码器 + 拨档开关");

    // 初始化编码器相关功能
    uart_init();
    uart_receive_config();  
    encoder_init();
    switch_init();
    estop_init();
    func_btn_init();

    // 创建编码器任务
    xTaskCreate(encoder_poll_task, "encoder_poll_task", 4096, NULL, 5, NULL);
    xTaskCreate(switch_task, "switch_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_receive_task, "uart_receive_task", 4096, NULL, 4, NULL);  

    LCD_Init();
    BK_Light(50);
    LVGL_Init();
    coordinate_display_init();

    lv_obj_add_flag(lv_layer_sys(), LV_OBJ_FLAG_HIDDEN);  // 隐藏性能监视器标签（FPS和CPU显示）

    // 创建功能按钮任务
    xTaskCreate(main_loop_task, "main_loop_task", 4096, NULL, 5, NULL);
}

