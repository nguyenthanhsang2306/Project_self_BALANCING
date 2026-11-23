#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 * SƠ ĐỒ ĐẤU DÂY PHẦN CỨNG: STM32L476RG Nucleo-64 + TB6612FNG + MPU6050
 *
 * 1. IMU MPU6050 (I2C1 @ 200 kHz, Timing = 0x10425F73):
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
 * 4. DEBUG UART (ST-Link VCP, Non-blocking TX):
 *    - USART2 @ 115200 8N1: PA2 (D1 / TX), PA3 (D0 / RX) - Pull-up
 *
 * 5. GIAO DIỆN TRÊN BOARD NUCLEO:
 *    - Nút nhấn B1: PC13 (Nhấn giữ > 60 ms -> cắt motor ngay, calib MPU6050 & xóa chốt lỗi)
 *    - Đèn LED LD2: PA5  (Sáng = BALANCE, Nháy chậm = FALL_WAIT, Tắt = Lỗi/Chốt an toàn)
 * ====================================================================== */

/* Chu kỳ điều khiển vòng lặp chính (200 Hz = 5 ms từ TIM6 phần cứng) */
#define CONTROL_PERIOD_MS       5U
#define CONTROL_WATCHDOG_MS   500U

/* Góc an toàn & Khôi phục tự động khi dựng xe:
 * - Nghiêng > 35°: Xe ngã -> cắt PWM tạm thời (FALL_WAIT)
 * - Dựng xe lại <= 18° và tốc độ góc <= 120 dps: Tự cân bằng tiếp
 * - Lỗi phần cứng (MOTOR_CUT / IMU_FAULT / I2C_ERR): Chốt tắt vĩnh viễn cho tới khi bấm B1 */
#define ARM_MAX_ANGLE_DEG      30.0f
#define ARM_MAX_RATE_DPS      120.0f
#define FALL_ANGLE_DEG         35.0f
#define FALL_RESUME_DEG        18.0f

/* Cấu hình PWM & Bù vùng chết động cơ (ARR = 999 -> 10 kHz) */
#define DEFAULT_PWM_LIMIT     999
#define ABSOLUTE_PWM_LIMIT    999
#define MANUAL_MAX_PWM        999
#define MANUAL_TIMEOUT_MS    2000U
#define DEFAULT_DEADBAND      125      /* Bù vùng chết khớp ma sát tĩnh thực tế GA12-N20 */

/* ---------------- VÒNG TRONG: PID CÂN BẰNG GÓC (200 Hz) ----------------
 * Hiệu chỉnh từ thực nghiệm phần cứng (giảm Kp mở dải tuyến tính, giảm Ki hạn chế trôi,
 * tăng Kd dập dao động lắc ngược khi qua vị trí cân bằng). */
#define DEFAULT_KP             48.00f
#define DEFAULT_KI              3.00f
#define DEFAULT_KD              5.60f
#define ANGLE_INTEGRAL_LIMIT   25.0f   /* Giới hạn chống bão hòa tích phân (Anti-windup) */

/* Góc cân bằng cơ học mục tiêu đo thực tế trên khung xe (độ).
 * Khóa giữ DEFAULT_ZERO_DEG khi bấm B1 (B1_CAPTURE_ZERO_ENABLE = 0) để tránh ghi đè nhầm
 * khi tay giữ nghiêng lúc nhấn nút; vẫn chỉnh tạm qua lệnh UART 'z' khi xe đang dừng. */
#define DEFAULT_ZERO_DEG       -3.65f
#define B1_CAPTURE_ZERO_ENABLE  0
#define B1_ZERO_CAPTURE_DEG     8.0f
#define DEFAULT_CONTROL_SIGN    1
#define DEFAULT_LEFT_SIGN       1
#define DEFAULT_RIGHT_SIGN      1

/* Chiều đếm Encoder tĩnh đã xác nhận trên khung xe (KHÔNG tự lật lúc đang chạy) */
#define DEFAULT_ENC_LEFT_SIGN  -1
#define DEFAULT_ENC_RIGHT_SIGN  1

/* ---------------- VÒNG NGOÀI: PID GIỮ VỊ TRÍ ĐỨNG YÊN x_ref = 0 (50 Hz) ----------------
 * Simulink: Kp_pos = 0.035 rad/m, Ki_pos = 0.0002 rad/(m.s), Kd_pos = 0.055 rad/(m/s)
 *   Kp_pos = 0.035  * 57.2958 = 2.005  độ/m
 *   Ki_pos = 0.0002 * 57.2958 = 0.0115 độ/(m.s)
 *   Kd_pos = 0.055  * 57.2958 = 3.151  độ/(m/s) */
#define DEFAULT_KP_POS          2.005f
#define DEFAULT_KI_POS          0.0115f
#define DEFAULT_KD_POS          3.151f
#define MAX_TARGET_ANGLE_DEG    8.0f   /* Giới hạn góc nghiêng bù của vòng ngoài */
#define POS_INTEGRAL_LIMIT     50.0f

#define POS_LOOP_DIVIDER        4U     /* 200 Hz / 4 = 50 Hz (chu kỳ vòng vị trí 20 ms) */
#define VEL_FILTER_N           10.0f   /* Lọc đạo hàm vận tốc bậc 1 (N = 10) */
#define POS_RUNAWAY_M           0.60f  /* Nếu bị nhấc/đẩy xa > 60 cm, đặt lại gốc đứng yên */

#define DEFAULT_ENC_ENABLE      1      /* Bật vòng vị trí khi phát hiện xung encoder hợp lệ */
#define DEFAULT_ENC_CPR      1400.0f   /* Số xung 1 vòng bánh (1:50 -> 7*4*50 = 1400 CPR) */
#define WHEEL_RADIUS_M        0.022f   /* Bán kính bánh xe đo thực tế D = 44 mm -> r = 22 mm */

/* Cấu hình trục nghiêng MPU6050 (1 = Trục Y: atan2(-ax, az); 0 = Trục X: atan2(ay, az)) */
#define IMU_TILT_AXIS           1
#define GYRO_LPF_ALPHA          0.70f  /* Lọc thông thấp gyro cho khâu D */
#define IMU_IO_TIMEOUT_MS       10U
#define IMU_CALIB_SAMPLES      150U    /* Calib nhanh ~0.3 giây */

#endif /* BOARD_CONFIG_H */
