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
constexpr int BASE_SPEED = 80;

// 最大速度限制，防止输出过大。
constexpr int MAX_SPEED = 90;

// 普通循迹转向力度。
// 这里相当于差速转向的 P：弯转不过就加大，左右抖就减小。
constexpr int TURN_SPEED_STEP = 18;

// 是否启用直道锁中。中间探头在线且左右外侧没压线时，不做小幅修正。
constexpr bool ENABLE_STRAIGHT_CENTER_LOCK = true;

// 连续检测到几次直道状态，才进入锁中。避免弯道入口/传感器临界值来回切换。
constexpr int STRAIGHT_LOCK_CONFIRM_COUNT = 3;

// 是否启用 V1 本地灰度 PD 的 D 项。
// 使用拟合后的连续目标线误差，D 项可以重新启用。
constexpr bool ENABLE_GRAY_D_CORRECTION = true;

// 灰度误差变化量对转向的阻尼强度。
// 太抖就加大一点；入弯变钝或转不过就减小。
constexpr float GRAY_D_GAIN = 10.0f;

// 灰度 D 单次最大修正，防止权重跳变时一把修过头。
constexpr float GRAY_D_CORRECTION_CLAMP = 12.0f;

// 直角弯时使用的强制转向权重。
// 转出去或甩头就降低；直角转不过再升。
constexpr int HARD_TURN_WEIGHT = 62;

// 丢线找线时使用的转向权重。
// 丢线后找不回来就升到 5；乱甩就降到 3
constexpr int LOST_TURN_WEIGHT = 5;

// 连续检测到几次外侧压线，才认为是直角弯
// 抖动和误识别时用 2 或 3；响应太慢再降到 1。
constexpr int HARD_CONFIRM_COUNT = 1;

// 连续丢线几次，才进入丢线找线
// 误触发 LOST 就升到 4；丢线救不回来就降到 2
constexpr int LOST_CONFIRM_COUNT = 4;

// 主循环延时
// 3ms 反应更快；太抖再加到 5ms。
constexpr int LOOP_DELAY_MS = 3;

// 串口打印间隔
constexpr int PRINT_INTERVAL_MS = 200;

// 左外、左内、中、右内、右外。
constexpr int GRAY_SENSOR_PINS[5] = {33, 32, 35, 34, 27};
// 灰度传感器触发阈值。
// 读数高于这个值就认为压到线了。
constexpr int GRAY_THRESHOLD = 600;

// 五个灰度传感器对应的转向权重：
// 左外、左内、中、右内、右外。
// 比 {-10,-5,0,5,10} 温和，但比 {-4,-2,0,2,4} 更早点给出弯道偏差。
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
