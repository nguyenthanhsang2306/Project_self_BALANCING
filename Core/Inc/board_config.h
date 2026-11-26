#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 * BỘ KHUNG PHẦN CỨNG NỀN (BRANCH: main)
 * STM32L476RG Nucleo-64 + TB6612FNG + ICM-20602 + Encoder GA12-N20
 *
 * Chuyển sang nhánh cụ thể để lấy thuật toán điều khiển hoàn chỉnh:
 *   - git checkout Project_self_PID : Tự cân bằng đứng yên tại chỗ
 *   - git checkout Project_motion   : Cân bằng + di chuyển qua ESP32-C6 WebSocket
 * ====================================================================== */

/* Chu kỳ vòng điều khiển chính (200 Hz = 5 ms từ TIM6) */
#define CONTROL_PERIOD_MS       5U
#define CONTROL_WATCHDOG_MS   500U

/* Cấu hình PWM & Bù vùng chết mềm động cơ (TIM1 ARR = 999 -> 10 kHz) */
#define DEFAULT_PWM_LIMIT     999
#define ABSOLUTE_PWM_LIMIT    999
#define MANUAL_MAX_PWM        999
#define MANUAL_TIMEOUT_MS    2000U
#define DEFAULT_DEADBAND      130
#define MOTOR_DEADBAND_RAMP    35

/* Chiều quay & chiều đếm Encoder đồng bộ với khung cơ khí (+ = chiều TIẾN thực tế) */
#define DEFAULT_CONTROL_SIGN   -1
#define DEFAULT_LEFT_SIGN      -1
#define DEFAULT_RIGHT_SIGN     -1
#define DEFAULT_ENC_LEFT_SIGN   1
#define DEFAULT_ENC_RIGHT_SIGN -1

/* Thông số cơ khí bánh xe & Encoder GA12-N20 (1:50 -> 1400 CPR) */
#define DEFAULT_ENC_ENABLE      1
#define DEFAULT_ENC_CPR      1400.0f
#define WHEEL_RADIUS_M        0.022f

/* Cấu hình cảm biến góc nghiêng ICM-20602 (I2C1: PB8 SCL, PB9 SDA, CS -> 3V3, SA0 -> GND) */
#define IMU_TILT_AXIS            1      /* 1 = Trục Y: atan2(-ax, az); 0 = Trục X: atan2(ay, az) */
#define ICM20602_GYRO_DLPF_CFG   2U     /* Thanh ghi 0x1A: 2 = 92 Hz (trễ 3.9 ms) */
#define ICM20602_ACCEL_DLPF_CFG  3U     /* Thanh ghi 0x1D: 3 = 44.8 Hz (4 = 21.2 Hz) */
#define GYRO_LPF_ALPHA           0.70f
#define IMU_IO_TIMEOUT_MS       10U
#define IMU_RUNTIME_TIMEOUT_US 2500U    /* Giới hạn cứng 2.5 ms bằng DWT ở vòng 200 Hz */
#define IMU_MAX_CONSEC_MISS      3U     /* Tối đa 3 chu kỳ (15 ms) dự đoán liên tiếp */
#define IMU_MAX_AGE_MS          20U
#define IMU_CALIB_SAMPLES      300U     /* Calib ~0.6 giây kèm kiểm tra xe đứng yên */
#define IMU_CALIB_MAX_GYRO_SPREAD_DPS 6.0f
#define IMU_CALIB_MAX_ACC_SPREAD_G    0.15f
#define IMU_CALIB_MAX_BIAS_DPS        8.0f
#define IMU_MAX_LIN_ACC_G        0.25f
#define IMU_KALMAN_R_BASE        0.03f
#define IMU_KALMAN_R_LIN_SCALE  40.0f
#define IMU_KALMAN_R_NORM_SCALE 20.0f
#define IMU_BIAS_FREEZE_ACC_G    0.025f
#define IMU_BIAS_FREEZE_NORM_G   0.06f

#endif /* BOARD_CONFIG_H */
