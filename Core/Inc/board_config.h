#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 * BỘ KHUNG PHẦN CỨNG NỀN (BRANCH: main)
 * STM32L476RG Nucleo-64 + TB6612FNG + ICM-20602 + Encoder GA12-N20
 *
 * Chuyển sang nhánh cụ thể để lấy thuật toán điều khiển hoàn chỉnh:
 *   - git checkout Project_self_PID    : Tự cân bằng đứng yên tại chỗ
 *   - git checkout Project_motion      : Cân bằng + di chuyển qua ESP32-C6 WebSocket
 *   - git checkout Project_self_tuning : Tinh chỉnh PID trực tiếp qua UART + Motor_PID_Panel
 * ====================================================================== */

/* Chu kỳ vòng điều khiển chính (200 Hz = 5 ms từ TIM6) */
#define CONTROL_PERIOD_MS       5U
#define CONTROL_WATCHDOG_MS   500U

/* Cấu hình PWM & Bù vùng chết mềm động cơ (TIM1 ARR = 999 -> 10 kHz, VM ổn áp 6.0V) */
#define DEFAULT_PWM_LIMIT     999
#define ABSOLUTE_PWM_LIMIT    999
#define MANUAL_MAX_PWM        999
#define MANUAL_TIMEOUT_MS    2000U
#define DEFAULT_DEADBAND      110
#define MOTOR_DEADBAND_RAMP    48

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

/* Cấu hình cảm biến góc nghiêng ICM-20602 (I2C1 @ 100 kHz + DNF=10: PB8 SCL, PB9 SDA, CS -> 3V3, SA0 -> GND) */
#define IMU_TILT_AXIS            1      /* 1 = Trục Y: atan2(-ax, az); 0 = Trục X: atan2(ay, az) */
#define ICM20602_GYRO_DLPF_CFG   2U     /* Thanh ghi 0x1A: 2 = 92 Hz / 3.9 ms (đồng bộ phép đo 20261127_141016) */
#define ICM20602_ACCEL_DLPF_CFG  3U     /* Thanh ghi 0x1D: 3 = 44.8 Hz (đồng bộ phép đo 20261127_141016) */
#define GYRO_LPF_ALPHA           0.55f
/* Offset gyro cố định (LSB @ +-500 dps, 65.5 LSB/(deg/s)) đo tĩnh 12000 mẫu [90,150)s @ 29.39°C
 * (log 20261127_141016, kiểm chứng chéo [30,60)s dư < 0.009 dps). Chỉ trừ 1 lần trong ReadData(). */
#define IMU_USE_FIXED_GYRO_BIAS  1
#define IMU_GYRO_X_OFFSET_LSB  (-8.784917f)
#define IMU_GYRO_Y_OFFSET_LSB   (21.700750f)
#define IMU_GYRO_Z_OFFSET_LSB  (-14.417417f)
#define IMU_I2C_TIMING          0x30955763U /* 100 kHz @ PCLK1=80MHz, DNF=10 (PRESC=3, SCLDEL=9, SDADEL=5, SCLH=0x57, SCLL=0x63) */
#define IMU_I2C_DNF             10U         /* Bộ lọc số chống xung nhiễu EMI 125 ns (10 * 12.5 ns) */
#define IMU_IO_TIMEOUT_MS       10U
#define IMU_RUNTIME_TIMEOUT_US 2200U    /* Giới hạn cứng 2.2 ms bằng DWT ở vòng 200 Hz */
#define IMU_RETRY_BUDGET_US    1800U    /* Chỉ thử đọc lại ngay trong cùng chu kỳ 5 ms nếu lỗi nhanh + recover < 1.8 ms */
#define IMU_MAX_CONSEC_MISS      6U     /* Tối đa 6 chu kỳ (30 ms) dự đoán liên tiếp */
#define IMU_MAX_AGE_MS          35U
#define IMU_CALIB_SAMPLES      300U     /* Calib ~0.6 giây kèm kiểm tra xe đứng yên */
#define IMU_CALIB_MAX_GYRO_SPREAD_DPS 6.0f
#define IMU_CALIB_MAX_ACC_SPREAD_G    0.15f
#define IMU_CALIB_MAX_BIAS_DPS        8.0f
#define IMU_MAX_LIN_ACC_G        0.25f
#define IMU_KALMAN_R_BASE        0.03f
#define IMU_KALMAN_R_LIN_SCALE  40.0f
#define IMU_KALMAN_R_NORM_SCALE 20.0f
#define IMU_BIAS_FREEZE_ACC_G    0.065f
#define IMU_BIAS_FREEZE_NORM_G   0.10f

#endif /* BOARD_CONFIG_H */
