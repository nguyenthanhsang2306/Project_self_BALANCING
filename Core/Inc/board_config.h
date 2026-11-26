#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 * SƠ ĐỒ ĐẤU DÂY PHẦN CỨNG: STM32L476RG Nucleo-64 + TB6612FNG + ICM-20602
 * (Dự án thuần tự cân bằng đứng yên tại chỗ - Station Keeping)
 *
 * 1. IMU ICM-20602 (I2C1 @ 200 kHz, Timing = 0x10425F73, địa chỉ 0x68):
 *    - VCC: 3V3 (hoặc 5V qua ổn áp trên module)
 *    - GND: GND
 *    - CS:  3V3 (khóa cứng chế độ I2C)
 *    - SA0: GND (cố định địa chỉ 0x68)
 *    - SCL: PB8 (D15) - AF4 Open-Drain + Pull-up
 *    - SDA: PB9 (D14) - AF4 Open-Drain + Pull-up
 *
 * 2. ĐỘNG CƠ TB6612FNG (PWM TIM1 @ 10 kHz, ARR = 999):
 *    - STBY: PC0 (A5)
 *    - PWMA (Bánh Trái): PA8 (D7)  - TIM1_CH1
 *    - PWMB (Bánh Phải): PA9 (D8)  - TIM1_CH2
 *    - AIN1 (Trái):      PB4 (D5)
 *    - AIN2 (Trái):      PB5 (D4)
 *    - BIN1 (Phải):      PB6 (D10)
 *    - BIN2 (Phải):      PA10 (D2)
 *    - PB7:              Reserved LOW (không dùng cho motor)
 *
 * 3. ENCODER GA12-N20 (Quadrature x4, tỉ số truyền 1:50 -> 1400 CPR):
 *    - Bánh Trái (TIM2): PA0 (A0 / CH1), PA1 (A1 / CH2) - Pull-up
 *    - Bánh Phải (TIM3): PA6 (D12 / CH1), PA7 (D11 / CH2) - Pull-up
 *
 * 4. DEBUG / TELEMETRY UART SONG SONG (115200 8N1, RXNEIE Ring Buffer):
 *    - USART2:  PA2 (D1 / TX), PA3 (D0 / RX) - ST-Link VCP
 *    - LPUART1: PC1 (A4 / TX), PB10 (D6 / RX) - Xem log không dây qua ESP32-C6
 *
 * 5. GIAO DIỆN TRÊN BOARD NUCLEO:
 *    - Nút nhấn B1: PC13 (Nhấn giữ > 50 ms -> cắt motor ngay, chờ nhả tối đa 3s, calib & mở khóa)
 *    - Đèn LED LD2: PA5  (Sáng = BALANCE, Nháy 200ms = FALL_WAIT [STBY=LOW], Tắt = Lỗi/Chốt an toàn)
 * ====================================================================== */

/* Chu kỳ điều khiển vòng lặp chính (200 Hz = 5 ms từ ngắt cứng TIM6 priority 0) */
#define CONTROL_PERIOD_MS          5U
#define CONTROL_WATCHDOG_MS      500U
#define CONTROL_ISR_WATCHDOG_TICKS 6U  /* Ngắt TIM6 tự cắt STBY/PWM nếu vòng chính trễ >= 30 ms */
#define CONTROL_MAX_MISSED_TICKS   3U  /* Chốt lỗi DEADLINE_ERR nếu bỏ lỡ >= 3 chu kỳ 5 ms */
#define CONTROL_MAX_DT_S        0.020f /* Không cho tiếp tục điều khiển nếu dt > 20 ms */
#define IWDG_TIMEOUT_MS          500U  /* Hardware Watchdog IWDG tự reset MCU nếu treo > 500 ms */
#define MANUAL_TIMEOUT_MS       2000U

/* Góc an toàn & Khôi phục khi dựng xe (Fail-Safe STBY = LOW khi nằm ngã):
 * - Khởi động / Nghiêng > 35° -> FALL_WAIT: gọi Motor_Stop() kéo STBY = LOW, PWM = 0
 * - Dựng thẳng lại <= 15° & |rate| <= 80 dps & IMU tươi -> tự bật STBY cân bằng đứng yên
 * - Lỗi phần cứng / Watchdog / mất encoder -> chốt fault_latched (STBY = LOW) tới khi bấm B1 / gửi 'r' */
#define ARM_MAX_ANGLE_DEG       15.0f
#define ARM_MAX_RATE_DPS        80.0f
#define FALL_ANGLE_DEG          35.0f
#define FALL_RESUME_DEG         15.0f

/* Công suất & Bù vùng chết mềm động cơ (TIM1 ARR = 999 -> 10 kHz)
 * - DEFAULT_DEADBAND = 130, MOTOR_DEADBAND_RAMP = 35 (~0.56°): làm mềm quanh điểm 0
 *   hết gằn bánh răng GA12-N20 nhưng vẫn thắng ma sát tĩnh (~165 PWM) khi nghiêng > 0.56°. */
#define DEFAULT_PWM_LIMIT      999
#define ABSOLUTE_PWM_LIMIT     999
#define MANUAL_MAX_PWM         999
#define DEFAULT_DEADBAND       130
#define MOTOR_DEADBAND_RAMP     35

/* ---------------- VÒNG TRONG: PD CÂN BẰNG GÓC THUẦN (200 Hz) ----------------
 * Đồng bộ độ cứng khung xe hiện tại và tắt tích phân vòng trong (DEFAULT_KI = 0.00f)
 * để khử tích phân kép gây lắc lư chậm qua độ rơ hộp số N20. */
#define DEFAULT_KP              62.00f
#define DEFAULT_KI               0.00f
#define DEFAULT_KD               6.60f
#define ANGLE_INTEGRAL_LIMIT    25.0f

/* Góc cân bằng cơ học & bộ dấu đồng bộ 100% với khung xe hiện tại (ICM-20602 / Project_Motion):
 * - DEFAULT_ZERO_DEG = +1.50° (đo thực tế trên khung xe hiện tại sau khi lắp ICM-20602).
 * - Quy ước chiều dương (+x, +enc_L, +enc_R, +tilt, +pwm) = chiều TIẾN thực tế của xe. */
#define DEFAULT_ZERO_DEG         1.50f
#define B1_CAPTURE_ZERO_ENABLE   0
#define B1_ZERO_CAPTURE_DEG      8.0f
#define DEFAULT_CONTROL_SIGN    -1
#define DEFAULT_LEFT_SIGN       -1
#define DEFAULT_RIGHT_SIGN      -1
#define DEFAULT_ENC_LEFT_SIGN    1
#define DEFAULT_ENC_RIGHT_SIGN  -1

/* ---------------- VÒNG NGOÀI: PID GIỮ VỊ TRÍ ĐỨNG YÊN x_ref = 0 (50 Hz) ----------------
 * - DEFAULT_KI_POS = 0.25 độ/(m.s) với giới hạn tích phân 16.0 -> tối đa ±4.0°:
 *   Nếu trọng tâm thật lệch 1-2°, xe tự tìm được góc cân bằng tĩnh mới trong vài giây
 *   ngay trong vài cm (và KHÔNG xóa tích phân nếu bị nhấc/đẩy vượt 0.60 m). */
#define DEFAULT_KP_POS           2.40f
#define DEFAULT_KI_POS           0.25f
#define DEFAULT_KD_POS           3.40f
#define MAX_TARGET_ANGLE_DEG     8.0f
#define POS_INTEGRAL_LIMIT      16.0f  /* 0.25 * 16.0 = ±4.0° góc bù trọng tâm tĩnh */
#define THETA_REF_LPF_ALPHA      0.30f /* Lọc mượt theta_ref từ 50 Hz lên 200 Hz */

#define POS_LOOP_DIVIDER         4U    /* 200 Hz / 4 = 50 Hz (dùng tổng dt thực tế pos_dt_accum) */
#define VEL_FILTER_N             8.0f  /* Bộ lọc đạo hàm vận tốc bậc 1 (N = 8) */
#define POS_RUNAWAY_M            0.60f /* Nếu bị nhấc/đẩy xa > 60 cm, đặt lại gốc x=0 nhưng GIỮ integral */

/* ---------------- KHÓA HƯỚNG YAW TẠI CHỖ (Encoder vi sai + Gyro Z @ 200 Hz) ----------------
 * Trục +Z của ICM-20602 hướng LÊN (az > 0), nên quay TRÁI (CCW) cho gz > 0, quay PHẢI (CW) cho gz < 0.
 * DEFAULT_YAW_SIGN = -1 quy đổi yaw_rate_dps > 0 = quay PHẢI, đồng dấu với (enc_left - enc_right) > 0.
 * Khi đầu xe xoay PHẢI (+yaw_err, +yaw_rate) -> turn_pwm < 0 -> giảm bánh trái, tăng bánh phải -> trả về TRÁI. */
#define DEFAULT_YAW_SIGN        (-1)
#define YAW_HOLD_KP_PULSE        0.45f /* Hệ số P khóa góc xoay theo hiệu xung [PWM/xung] (~9.5 PWM/độ) */
#define YAW_HOLD_KD_RATE         1.10f /* Hệ số D dập xoay ngang theo imu.gz [PWM/(độ/s)] */
#define YAW_HOLD_MAX_PWM       120.0f  /* Giới hạn vi sai khóa hướng tối đa (vẫn nhường headroom cân bằng) */
#define YAW_HOLD_MAX_TILT_DEG   12.0f  /* Cắt khóa hướng nếu nghiêng > 12° để ưu tiên 100% giữ thăng bằng */

/* ---------------- BÙ VẬN TỐC KHỐI TÂM (CoM) & GIÁM SÁT ENCODER ---------------- */
#define COM_HEIGHT_M             0.035f /* Khoảng cách trục bánh -> khối tâm thân xe l_com [m] */
#define WHEEL_RADIUS_M           0.022f /* Bán kính bánh xe đo thực tế D = 44 mm -> r = 22 mm */
#define DEFAULT_ENC_CPR       1400.0f   /* 1:50 -> 7*4*50 = 1400 xung/vòng */
#define DEFAULT_ENC_ENABLE       1
#define ENC_ACTIVE_MIN_PULSES    4.0f   /* Chỉ dùng phân loại NO_SIG vs STALL, KHÔNG chặn vòng vị trí */
#define ENC_LOSS_PWM_THRESH    320      /* Ngưỡng PWM phát hiện mất xung bất đối xứng 1 kênh */
#define ENC_LOSS_OTHER_MIN_D     4      /* Kênh còn lại phải đang quay >= 4 xung/tick mới báo mất 1 kênh */
#define ENC_LOSS_TICKS          20U     /* 20 tick @ 50 Hz = 0.40s -> ENC_LOSS_L / ENC_LOSS_R */
#define MOTOR_STALL_PWM_THRESH 420      /* Ngưỡng PWM cao giám sát kẹt/đứt dây cả 2 bánh */
#define MOTOR_STALL_TICKS       50U     /* 50 tick @ 50 Hz = 1.00s -> MOTOR_STALL / ENC_NO_SIG */

/* ---------------- CẤU HÌNH ICM-20602 & KALMAN CHỐNG LỆCH GIA TỐC ----------------
 * - ICM-20602 DLPF Gyro = 2 (92 Hz, trễ 3.9 ms) & DLPF Accel = 3 (44.8 Hz; đổi thành 4 = 21.2 Hz nếu rung mạnh).
 * - I2C1 mặc định 200 kHz (0x10425F73) an toàn dây Dupont; nếu dây ngắn có thể nâng 400 kHz (0x1042203A). */
#define IMU_TILT_AXIS            1      /* 1 = Trục Y: atan2(-ax, az); 0 = Trục X: atan2(ay, az) */
#define ICM20602_GYRO_DLPF_CFG   2U     /* Thanh ghi 0x1A: 2 = 92 Hz (trễ 3.9 ms) */
#define ICM20602_ACCEL_DLPF_CFG  3U     /* Thanh ghi 0x1D: 3 = 44.8 Hz (4 = 21.2 Hz) */
#define GYRO_LPF_ALPHA           0.70f
#define IMU_IO_TIMEOUT_MS       10U     /* Timeout I2C lúc Init/Calib (motor đang tắt) */
#define IMU_RUNTIME_TIMEOUT_US 2500U    /* Giới hạn cứng 2.5 ms (DWT) khi đọc IMU ở vòng 200 Hz */
#define IMU_MAX_CONSEC_MISS      3U     /* Tối đa 3 chu kỳ (15 ms) dự đoán liên tiếp khi mất mẫu mới */
#define IMU_MAX_AGE_MS          20U     /* Tuổi mẫu IMU tối đa 20 ms -> quá hạn cắt motor ngay */
#define IMU_CALIB_SAMPLES      300U     /* Calib ~0.6 giây (300 mẫu) giảm nhiễu bias gyro kèm kiểm tra đứng yên (IMU_MOVING) */
#define IMU_CALIB_MAX_GYRO_SPREAD_DPS 6.0f
#define IMU_CALIB_MAX_ACC_SPREAD_G    0.15f
#define IMU_CALIB_MAX_BIAS_DPS        8.0f
#define IMU_MAX_LIN_ACC_G        0.25f
#define IMU_KALMAN_R_BASE        0.03f
#define IMU_KALMAN_R_LIN_SCALE  40.0f
#define IMU_KALMAN_R_NORM_SCALE 20.0f
#define IMU_BIAS_FREEZE_ACC_G    0.025f
#define IMU_BIAS_FREEZE_NORM_G   0.06f

/* Chu kỳ Telemetry (40 ms = 25 Hz, tương thích hoàn toàn với ESP32-C6 Web UI) */
#define TELEMETRY_PERIOD_MS     40U

#endif /* BOARD_CONFIG_H */
