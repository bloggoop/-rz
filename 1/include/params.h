
#pragma once
//7.9 game_use

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
constexpr int BASE_SPEED = 65;

// 最大速度限制，防止输出过大。
constexpr int MAX_SPEED = 80;

// 普通循迹转向力度。
// 这里相当于差速转向的 P：弯转不过就加大，左右抖就减小。
constexpr int TURN_SPEED_STEP = 25;

// =========================
// 比赛速度分档参数
// =========================

// 当前直道速度。直道接弯飞出去就继续降。
constexpr int STRAIGHT_SPEED = 74;

// 轻微入弯/小修正速度，对应 abs(turnWeight) >= SMALL_CURVE_TURN_WEIGHT。
constexpr int SMALL_CURVE_SPEED = 50;

// 大弯速度，对应 abs(turnWeight) >= BIG_CURVE_TURN_WEIGHT。
constexpr int BIG_CURVE_SPEED = 45;

// 直角弯/丢线找线速度。
constexpr int HARD_OR_LOST_SPEED = 60;

// 灰度权重到这个值开始认为是小弯/入弯。
constexpr int SMALL_CURVE_TURN_WEIGHT = 2;

// 灰度权重到这个值开始认为是大弯。
constexpr int BIG_CURVE_TURN_WEIGHT = 4;

// 普通弯差速上限：effectiveBaseSpeed + NORMAL_TURN_DELTA_MARGIN。
constexpr int NORMAL_TURN_DELTA_MARGIN = 10;

// 直角弯/丢线时允许更大的差速。
constexpr int HARD_OR_LOST_MAX_SPEED_DELTA = 200;

// 转向死区，小于这个控制量时当作直行。
constexpr float TURN_DEADZONE = 0.3f;


// 是否启用 V1 本地灰度 PD 的 D 项。
// 默认关闭，保持当前稳定巡线行为；要抑制小幅摆头时再打开。
constexpr bool ENABLE_GRAY_D_CORRECTION = true;

// 灰度误差变化量对转向的阻尼强度。
// 太抖就加大一点；入弯变钝或转不过就减小。
constexpr float GRAY_D_GAIN = 16.0f;

// 灰度 D 单次最大修正，防止权重跳变时一把修过头。
constexpr float GRAY_D_CORRECTION_CLAMP = 18.0f;

// 直角弯时使用的强制转向权重。
// 转出去或甩头就降低；直角转不过再升。
constexpr int HARD_TURN_WEIGHT = 95;

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

// =========================
// MPU / S3 串口联动参数
// =========================

// 是否启用 S3 -> V1 的 MPU/CAM 串口接收链路。
// 当前使用文本行协议：IMU,seq,pitch,roll,yawOrGyro*CRC
constexpr bool ENABLE_MPU_TELEMETRY = true;
constexpr bool ENABLE_PULSE_TELEMETRY = false;

// 使用 I2C 接收 S3 的 MPU 数据。
// 接线：S3 GPIO47 -> V1 GPIO22 (SDA)，S3 GPIO21 -> V1 GPIO23 (SCL)，两板 GND 共地。
constexpr bool ENABLE_I2C_TELEMETRY = false;
constexpr uint8_t TELEMETRY_I2C_ADDR = 0x12;
constexpr int TELEMETRY_I2C_SDA_PIN = 22;
constexpr int TELEMETRY_I2C_SCL_PIN = 23;
constexpr uint32_t TELEMETRY_I2C_CLOCK = 100000;

// V1 板 UART1 引脚。
// 接线：S3 GPIO45 TX -> V1 GPIO22 RX，S3 GPIO46 RX <- V1 GPIO23 TX，两板 GND 共地。
constexpr int TELEMETRY_UART_RX_PIN = 22;
constexpr int TELEMETRY_UART_TX_PIN = 23;
constexpr uint32_t TELEMETRY_UART_BAUD = 115200;

// 通信没收到字节时，扫描这些普通 ESP32 合法空闲脚上的电平跳变。
// 把 S3_TX 接到不同 V1 排针时，看日志里哪个 GPIO 的 edges 明显增加。
constexpr bool ENABLE_UART_RX_PIN_SCAN = false;
constexpr int UART_RX_SCAN_PINS[] = {4, 5, 21, 23, 26, 36, 39};
constexpr uint32_t UART_RX_SCAN_INTERVAL_MS = 1000;

// 自动轮询候选 RX 脚，哪个脚真正收到 UART 字节就停在哪个脚。
constexpr bool ENABLE_UART_RX_AUTO_PROBE = false;
constexpr int UART_RX_PROBE_PINS[] = {26, 36, 39, 23, 21, 5, 4};
constexpr uint32_t UART_RX_PROBE_INTERVAL_MS = 1500;

// 车端接收到的 MPU 数据多久没更新就认为失效。
constexpr uint32_t TELEMETRY_TIMEOUT_MS = 300;

// MPU 的角速度对转向的抑制强度。
// 太抖就调小；入弯不够稳就调大一点。
constexpr float MPU_GYRO_D_GAIN = 0.03f;

// 如果发现 MPU 的正负方向和车身实际转向相反，就改成 1.0f。
constexpr float MPU_GYRO_D_SIGN = -1.0f;

// 是否把 MPU 角速度真正加到转向里。
// 先关掉，确认串口数据、方向和零漂正常后再打开。
constexpr bool ENABLE_MPU_D_CORRECTION = false;

// 只有在灰度权重接近中间时，才允许 MPU 参与纠偏。
// 这样可以避免 MPU 在大弯、丢线时把车越推越偏。
constexpr int MPU_D_MAX_TURN_WEIGHT = 2;

// MPU 单次纠偏最大幅度，防止一次修正过头。
constexpr float MPU_D_CORRECTION_CLAMP = 1.5f;

// 是否在串口里打印更详细的 MPU/循迹状态。
constexpr bool ENABLE_SERIAL_TRACE = true;

// 详细日志的打印间隔。
constexpr uint32_t SERIAL_TRACE_INTERVAL_MS = 100;

// 串口打印间隔
constexpr int PRINT_INTERVAL_MS = 200;

// 左外、左内、中、右内、右外。
// V1 不开 WiFi，GPIO27 可以继续作为灰度 ADC 使用。
constexpr int GRAY_SENSOR_PINS[5] = {33, 32, 35, 34, 27};
// 灰度传感器触发阈值。
// 读数高于这个值就认为压到线了。
constexpr int GRAY_THRESHOLD = 2000;

// 五个灰度传感器对应的转向权重：
// 左外、左内、中、右内、右外。
// 比 {-10,-5,0,5,10} 温和，但比 {-4,-2,0,2,4} 更早点给出弯道偏差。
constexpr int GRAY_SENSOR_WEIGHTS[5] = {-4, -2, 0, 2, 4};

// 指示灯 PWM 参数
// GPIO22 已作为 I2C SDA，调 MPU 阶段先关闭 V1 呼吸灯。
constexpr int LED_PIN = -1;
constexpr int LED_PWM_FREQ = 1000;
constexpr int LED_BREATH_STEP = 24;

// 左右电机驱动引脚
constexpr int LEFT_MOTOR_PINS[2] = {14, 25};
constexpr int RIGHT_MOTOR_PINS[2] = {13, 15};

// 左右编码器引脚
constexpr int LEFT_ENCODER_PINS[2] = {18, 19};
constexpr int RIGHT_ENCODER_PINS[2] = {16, 17};
