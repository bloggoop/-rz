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
constexpr int BASE_SPEED = 90;

// 最大速度限制，防止输出过大。
constexpr int MAX_SPEED = 120;

// 普通循迹转向力度。
// 这里相当于差速转向的 P：弯转不过就加大，左右抖就减小。
constexpr int TURN_SPEED_STEP = 13;


// 是否启用 V1 本地灰度 PD 的 D 项。
// 默认关闭，保持当前稳定巡线行为；要抑制小幅摆头时再打开。
constexpr bool ENABLE_GRAY_D_CORRECTION = true;

// 灰度误差变化量对转向的阻尼强度。
// 太抖就加大一点；入弯变钝或转不过就减小。
constexpr float GRAY_D_GAIN = 30.0f;

// 灰度 D 单次最大修正，防止权重跳变时一把修过头。
constexpr float GRAY_D_CORRECTION_CLAMP =40.0f;

// 直角弯时使用的强制转向权重。
// 转出去或甩头就降低；直角转不过再升。
constexpr int HARD_TURN_WEIGHT = 35;

// 丢线找线时使用的转向权重。
// 丢线后找不回来就升到 5；乱甩就降到 3
constexpr int LOST_TURN_WEIGHT = 5;

// 连续检测到几次外侧压线，才认为是直角弯
// 抖动和误识别时用 2 或 3；响应太慢再降到 1。
constexpr int HARD_CONFIRM_COUNT = 2;

// 连续丢线几次，才进入丢线找线
// 误触发 LOST 就升到 4；丢线救不回来就降到 2
constexpr int LOST_CONFIRM_COUNT = 4;

// 主循环延时
// 3ms 反应更快；太抖再加到 5ms。
constexpr int LOOP_DELAY_MS = 5;

// =========================
// MPU / S3 串口联动参数
// =========================

// 是否启用 S3 -> V1 的 MPU/CAM 串口接收链路。
// 当前使用文本行协议：IMU,seq,pitch,roll,yawOrGyro*CRC
constexpr bool ENABLE_MPU_TELEMETRY = true;

// V1 板 UART1 引脚。
// 接线：S3 GPIO45 TX -> V1 GPIO22 RX，S3 GPIO46 RX <- V1 GPIO23 TX，两板 GND 共地。
constexpr int TELEMETRY_UART_RX_PIN = 22;
constexpr int TELEMETRY_UART_TX_PIN = 23;
constexpr uint32_t TELEMETRY_UART_BAUD = 115200;

// 车端接收到的 MPU 数据多久没更新就认为失效。
constexpr uint32_t TELEMETRY_TIMEOUT_MS = 300;

// MPU 的角速度对转向的抑制强度。
// 太抖就调小；入弯不够稳就调大一点。
constexpr float MPU_GYRO_D_GAIN = 0.03f;

// 如果发现 MPU 的正负方向和车身实际转向相反，就改成 1.0f。
constexpr float MPU_GYRO_D_SIGN = -1.0f;

// 是否把 MPU 角速度真正加到转向里。
// 先关掉，确认串口数据、方向和零漂正常后再打开。
constexpr bool ENABLE_MPU_D_CORRECTION = true;

// 只有在灰度权重接近中间时，才允许 MPU 参与纠偏。
// 这样可以避免 MPU 在大弯、丢线时把车越推越偏。
constexpr int MPU_D_MAX_TURN_WEIGHT = 2;

// MPU 单次纠偏最大幅度，防止一次修正过头。
constexpr float MPU_D_CORRECTION_CLAMP = 1.5f;

// 是否用 S3 相机做速度前馈：前方直道加速，前方弯道减速。
// 相机装得高、看得远，所以这里会延迟一段时间再改变速度。
constexpr bool ENABLE_CAMERA_SPEED_FEED_FORWARD = true;

// 相机速度前馈延时。相机越靠前/越高，这个值越大；车速越快，这个值越小。
constexpr uint32_t CAMERA_SPEED_DELAY_MS = 180;

// 低于这个线质量就不使用相机调速。
constexpr float CAMERA_SPEED_MIN_LINE_QUALITY = 0.45f;

// 认为前方是弯道的视觉偏移阈值。
// near/far/center 任一明显偏离或上下变化明显，就会减速。
constexpr float CAMERA_CURVE_OFFSET_THRESHOLD = 0.22f;

// 相机确认直道时使用的速度。
constexpr int CAMERA_STRAIGHT_SPEED = 85;

// 相机看到弯道时使用的速度。
constexpr int CAMERA_CURVE_SPEED = 55;

// 是否在串口里打印更详细的 MPU/循迹状态。
constexpr bool ENABLE_SERIAL_TRACE = false;

// 详细日志的打印间隔。
constexpr uint32_t SERIAL_TRACE_INTERVAL_MS = 100;

// 串口打印间隔
constexpr int PRINT_INTERVAL_MS = 200;

// 左外、左内、中、右内、右外。
// V1 不开 WiFi，GPIO27 可以继续作为灰度 ADC 使用。
constexpr int GRAY_SENSOR_PINS[5] = {33, 32, 35, 34, 27};
// 灰度传感器触发阈值。
// 读数高于这个值就认为压到线了。
constexpr int GRAY_THRESHOLD = 600;

// 五个灰度传感器对应的转向权重：
// 左外、左内、中、右内、右外。
// 比 {-10,-5,0,5,10} 温和，但比 {-4,-2,0,2,4} 更早点给出弯道偏差。
constexpr int GRAY_SENSOR_WEIGHTS[5] = {-4, -2, 0, 2, 4};

// 指示灯 PWM 参数
// GPIO22 已作为 UART RX，调 MPU 阶段先关闭 V1 呼吸灯。
constexpr int LED_PIN = -1;
constexpr int LED_PWM_FREQ = 1000;
constexpr int LED_BREATH_STEP = 24;

// 左右电机驱动引脚
constexpr int LEFT_MOTOR_PINS[2] = {14, 25};
constexpr int RIGHT_MOTOR_PINS[2] = {13, 15};

// 左右编码器引脚
constexpr int LEFT_ENCODER_PINS[2] = {18, 19};
constexpr int RIGHT_ENCODER_PINS[2] = {16, 17};
