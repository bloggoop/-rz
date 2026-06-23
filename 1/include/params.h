#pragma once

// =========================
// 基础硬件参数
// =========================

// 电机 PWM 频率，影响驱动频率
constexpr int PWM_FREQ = 20000;
// PWM 分辨率，ESP32 这里一般保持 10 位
constexpr int PWM_RESOLUTION = 10;
// PWM 最大占空比，对应 100%
constexpr int PWM_MAX = 1023;

// 左电机整体补偿。
// 左轮偏快就填负数，左轮偏慢就填正数。
constexpr int LEFT_SPEED_TRIM = -2;

// 基础前进速度。
// 太容易冲出去就调小，跑得太慢就调大。
constexpr int BASE_SPEED = 60;

// 最大速度限制，防止输出过大。
constexpr int MAX_SPEED = 80;

// 普通循迹时的转向系数。
// 车身抖动大就调小，拐弯不够就调大。
constexpr int TURN_SPEED_STEP = 10;

// 检测到直角弯时的强制转向权重。
// 响应慢就调大，转向过猛就调小。
constexpr int HARD_TURN_WEIGHT = 25;

// 丢线后的找线转向权重。
// 找不回线就调大，来回乱甩就调小。
constexpr int LOST_TURN_WEIGHT = 4;

// 连续检测到几次外侧压线，才认定为直角弯。
// 误判多就调大，反应慢就调小。
constexpr int HARD_CONFIRM_COUNT = 1;

// 连续丢线几次后，才进入找线模式。
// 频繁误触发就调大，丢线后反应太慢就调小。
constexpr int LOST_CONFIRM_COUNT = 4;

// 主循环延时。
// 数值小响应更快，数值大更稳定。
constexpr int LOOP_DELAY_MS = 3;

constexpr int GRAY_SENSOR_PINS[5] = {33, 32, 35, 34, 27};
// 灰度传感器触发阈值。
// 读数高于这个值就认为压到线了。
constexpr int GRAY_THRESHOLD = 600;

// 五个灰度传感器对应的转向权重：
// 左外、左内、中、右内、右外。
// 绝对值越大，转向影响越强。
constexpr int GRAY_SENSOR_WEIGHTS[5] = {-4, -2, 0, 2, 4};

// 指示灯 PWM 参数
constexpr int LED_PIN = 22;
constexpr int LED_PWM_FREQ = 1000;
constexpr int LED_BREATH_STEP = 24;

// 左右电机驱动引脚
constexpr int LEFT_MOTOR_PINS[2] = {14, 25};
constexpr int RIGHT_MOTOR_PINS[2] = {13, 15};

// 左右编码器引脚
constexpr int LEFT_ENCODER_PINS[2] = {18, 19};
constexpr int RIGHT_ENCODER_PINS[2] = {16, 17};
