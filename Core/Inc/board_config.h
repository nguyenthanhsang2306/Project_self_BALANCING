#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* Pinout STM32L476RG Nucleo-64 + TB6612FNG + ICM-20602:
 * - IMU ICM-20602 I2C1 @ 100 kHz (0x68):
 *   VCC -> 3V3 (hoặc 5V qua ổn áp trên module), GND -> GND,
 *   CS -> 3V3 (khóa I2C), SA0 -> GND (0x68), SCL -> PB8 (D15), SDA -> PB9 (D14)
 * - Motor PWM (TIM1 10kHz): PA8 (CH1 / Motor A), PA9 (CH2 / Motor B)
 * - Motor Dir Left:  PB4 (AIN1 / D5), PB5 (AIN2 / D4)
 * - Motor Dir Right: PB6 (BIN1 / D10), PA10 (BIN2 / D2)
 * - Motor Standby:   PC0 (STBY / A5)
 * - PB7: Reserved LOW
 * - Encoder Left:    PA0 (TIM2_CH1 / A0), PA1 (TIM2_CH2 / A1)
 * - Encoder Right:   PA6 (TIM3_CH1 / D12), PA7 (TIM3_CH2 / D11) */

#define CONTROL_PERIOD_MS          5U   /* Ts = 0.005 s (200 Hz) */
#define CONTROL_WATCHDOG_MS      500U
#define CONTROL_MAX_MISSED_TICKS   3U
#define MANUAL_TIMEOUT_MS       2000U

/* Góc an toàn & Khôi phục khi dựng xe (STBY = LOW khi nằm ngã): */
#define ARM_MAX_ANGLE_DEG       30.0f
#define ARM_MAX_RATE_DPS        80.0f
#define FALL_ANGLE_DEG          35.0f
#define FALL_RESUME_DEG         15.0f

/* Công suất & Bù vùng chết mềm động cơ (PWM_MAX = 1000, TIM1 ARR = 999, VM = 6.0V) */
#define DEFAULT_PWM_LIMIT      999
#define ABSOLUTE_PWM_LIMIT     999
#define MANUAL_MAX_PWM         999
#define DEFAULT_DEADBAND       110      /* Chỉnh live bằng lệnh DEAD (0..250) */
#define MOTOR_DEADBAND_RAMP     48      /* Chỉnh live bằng lệnh RAMP (1..100) */

/* ---------------- VÒNG TRONG: PD CÂN BẰNG GÓC THUẦN (200 Hz, VM = 6.0V) ---------------- */
#define DEFAULT_KP              92.00f
#define DEFAULT_KI               0.00f
#define DEFAULT_KD               5.50f
#define ANGLE_INTEGRAL_LIMIT    25.0f

/* Góc cân bằng cơ học & bộ dấu đồng bộ với khung xe hiện tại (ICM-20602) */
#define DEFAULT_ZERO_DEG         1.50f
#define DEFAULT_CONTROL_SIGN    -1
#define DEFAULT_LEFT_SIGN       -1
#define DEFAULT_RIGHT_SIGN      -1

/* ---------------- VÒNG NGOÀI: PID GIỮ VỊ TRÍ KHỐI TÂM x_ref = 0 (50 Hz) ---------------- */
#define DEFAULT_KP_POS           9.00f
#define DEFAULT_KI_POS           1.80f
#define DEFAULT_KD_POS          12.50f
#define MAX_TARGET_ANGLE_DEG     8.0f
#define POS_INTEGRAL_LIMIT       3.20f  /* 1.80 * 3.20 = +-5.76 độ bù lệch trọng tâm tĩnh */
#define THETA_REF_LPF_ALPHA      0.35f
#define POS_LOOP_DIVIDER         4U     /* Vòng vị trí chạy 50 Hz (~20 ms) */
#define VEL_FILTER_N            16.0f   /* Bộ lọc đạo hàm vận tốc bậc 1 (N = 16 -> trễ ~62ms) */
#define POS_RUNAWAY_M            0.60f  /* Quá 0.60 m: rebase gốc x=0 nhưng GIỮ pid_pos.integral */
#define COM_HEIGHT_M             0.033f /* Chiều cao khối tâm từ CAD Khung_Frame.step: 32.9 mm */

/* ---------------- KHÓA HƯỚNG YAW TẠI CHỖ (Encoder vi sai + Gyro Z @ 200 Hz) ---------------- */
#define DEFAULT_YAW_SIGN        (-1)
#define YAW_HOLD_KP_PULSE        0.30f
#define YAW_HOLD_KD_RATE         1.20f
#define YAW_HOLD_MAX_PWM       120.0f
#define YAW_HOLD_MAX_TILT_DEG   12.0f

/* ---------------- ENCODER & BÁNH XE ---------------- */
#define WHEEL_RADIUS_M           0.022f /* Bánh D = 44 mm -> r = 22 mm */
#define DEFAULT_ENC_CPR       1400.0f   /* 1:50 -> 7*4*50 = 1400 xung/vòng */
#define DEFAULT_ENC_ENABLE       1
#define DEFAULT_ENC_LEFT_SIGN    1
#define DEFAULT_ENC_RIGHT_SIGN  -1

/* ---------------- CẤU HÌNH ICM-20602 & KALMAN CHỐNG LỆCH GIA TỐC ---------------- */
#define IMU_TILT_AXIS            1      /* 1 = Trục Y: atan2(-ax, az); 0 = Trục X: atan2(ay, az) */
#define ICM20602_GYRO_DLPF_CFG   2U     /* 0x1A: 2 = 92 Hz / 3.9 ms (đồng bộ phép đo 20261127_141016) */
#define ICM20602_ACCEL_DLPF_CFG  3U     /* 0x1D: 3 = 44.8 Hz (đồng bộ phép đo 20261127_141016) */
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
#define IMU_RUNTIME_TIMEOUT_US 2200U
#define IMU_RETRY_BUDGET_US     1800U /* Chỉ thử đọc lại ngay trong cùng chu kỳ 5 ms nếu lần 1 lỗi nhanh + recover < 1.8 ms */
#define IMU_MAX_CONSEC_MISS      6U
#define IMU_MAX_AGE_MS          35U
#define IMU_CALIB_SAMPLES      300U     /* Calib ~0.6 giây kèm kiểm tra đứng yên (IMU_MOVING) */
#define IMU_CALIB_MAX_GYRO_SPREAD_DPS 6.0f
#define IMU_CALIB_MAX_ACC_SPREAD_G    0.15f
#define IMU_CALIB_MAX_BIAS_DPS        8.0f
#define IMU_MAX_LIN_ACC_G        0.25f
#define IMU_KALMAN_R_BASE        0.03f
#define IMU_KALMAN_R_LIN_SCALE  40.0f
#define IMU_KALMAN_R_NORM_SCALE 20.0f
#define IMU_BIAS_FREEZE_ACC_G    0.065f
#define IMU_BIAS_FREEZE_NORM_G   0.10f

#endif
