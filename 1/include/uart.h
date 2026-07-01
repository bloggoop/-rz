#include <Arduino.h>

// =========================
// MPU / WiFi 联动参数
// =========================

// 是否启用 MPU UDP 接收链路。
// 注意：ESP32 开 WiFi 后 ADC2 引脚会不可用，所以灰度传感器必须全部接 ADC1。
// 现在先保证 V1 单循迹稳定，所以关闭 WiFi/MPU。
constexpr bool ENABLE_MPU_TELEMETRY = false;

// 负责发 MPU 数据的 AP 名称和密码。
// 需要和 2/src/main.cpp 保持一致。
constexpr char TELEMETRY_AP_SSID[] = "ESP32S3-CAM";
constexpr char TELEMETRY_AP_PASSWORD[] = "12345678";

// MPU 车身控制用的 UDP 端口。
constexpr uint16_t TELEMETRY_UDP_PORT = 3333;

// 车端接收到的 MPU 数据多久没更新就认为失效。
constexpr uint32_t TELEMETRY_TIMEOUT_MS = 300;

// MPU 的角速度对转向的抑制强度。
// 太抖就调小；入弯不够稳就调大一点。
constexpr float MPU_GYRO_D_GAIN = 0.08f;

// 如果发现 MPU 的正负方向和车身实际转向相反，就改成 1.0f。
constexpr float MPU_GYRO_D_SIGN = -1.0f;

// 是否把 MPU 角速度真正加到转向里。
// 调试阶段建议先关掉，确认数据方向和数值正常后再打开。
constexpr bool ENABLE_MPU_D_CORRECTION = false;

// 只有在灰度权重接近中间时，才允许 MPU 参与纠偏。
// 这样可以避免 MPU 在大弯、丢线时把车越推越偏。
constexpr int MPU_D_MAX_TURN_WEIGHT = 2;

// MPU 单次纠偏最大幅度，防止一次修正过头。
constexpr float MPU_D_CORRECTION_CLAMP = 1.5f;

// 是否在串口里打印更详细的 MPU/循迹状态。
constexpr bool ENABLE_SERIAL_TRACE = false;