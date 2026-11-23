#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 * BỘ KHUNG PHẦN CỨNG NỀN (BRANCH: main)
 * STM32L476RG Nucleo-64 + TB6612FNG + MPU6050 + Encoder GA12-N20
 *
 * Chuyển sang nhánh cụ thể để lấy thuật toán điều khiển hoàn chỉnh:
 *   - git checkout Project_self_PID : Tự cân bằng đứng yên tại chỗ
 *   - git checkout Project_motion   : Cân bằng + di chuyển qua ESP32-C6 WebSocket
 * ====================================================================== */

/* Chu kỳ vòng điều khiển chính (200 Hz = 5 ms từ TIM6) */
#define CONTROL_PERIOD_MS       5U
#define CONTROL_WATCHDOG_MS   500U

/* Cấu hình PWM & giới hạn công suất động cơ (TIM1 ARR = 999 -> 10 kHz) */
#define DEFAULT_PWM_LIMIT     999
#define ABSOLUTE_PWM_LIMIT    999
#define MANUAL_MAX_PWM        999
#define MANUAL_TIMEOUT_MS    2000U
#define DEFAULT_DEADBAND      125

/* Chiều quay & chiều đếm Encoder mặc định của khung cơ khí */
#define DEFAULT_CONTROL_SIGN    1
#define DEFAULT_LEFT_SIGN       1
#define DEFAULT_RIGHT_SIGN      1
#define DEFAULT_ENC_LEFT_SIGN  -1
#define DEFAULT_ENC_RIGHT_SIGN  1

/* Thông số cơ khí bánh xe & Encoder GA12-N20 (1:50 -> 1400 CPR) */
#define DEFAULT_ENC_ENABLE      1
#define DEFAULT_ENC_CPR      1400.0f
#define WHEEL_RADIUS_M        0.022f

/* Cấu hình cảm biến góc nghiêng MPU6050 (I2C1: PB8 SCL, PB9 SDA) */
#define IMU_TILT_AXIS           1      /* 1 = Trục Y: atan2(-ax, az) */
#define GYRO_LPF_ALPHA          0.70f
#define IMU_IO_TIMEOUT_MS       10U
#define IMU_CALIB_SAMPLES      150U

#endif /* BOARD_CONFIG_H */
