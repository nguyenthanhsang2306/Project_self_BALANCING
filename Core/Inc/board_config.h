#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 * SƠ ĐỒ ĐẤU DÂY PHẦN CỨNG: STM32L476RG Nucleo-64 + TB6612FNG + MPU6050 + ESP32-C6
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
 * 4. UART GIAO TIẾP ESP32-C6 (Song song 115200 8N1, RX Interrupt Ring Buffer, Pull-up):
 *    - USART2:  PA3 (D0 / RX <- ESP32 GPIO18 TX), PA2 (D1 / TX -> ESP32 GPIO17 RX)
 *    - LPUART1: PB10 (D6 / RX <- ESP32 GPIO18 TX), PC1 (A4 / TX -> ESP32 GPIO17 RX)
 *      (Dùng D6/A4 mặc định trên Nucleo-L476RG vì D0/D1 có thể hở cầu chì SB62/SB63)
 *
 * 5. GIAO DIỆN BOARD:
 *    - Nút nhấn B1: PC13 (Nhấn giữ > 50 ms -> cắt motor ngay, calib IMU & xóa chốt lỗi)
 *    - Đèn LED LD2: PA5  (Sáng = BALANCE, Nháy 200ms = FALL_WAIT, Tắt = Lỗi/Chốt an toàn)
 * ====================================================================== */

#define CONTROL_PERIOD_MS       5U     /* Chu kỳ vòng trong 200 Hz (TIM6 5 ms) */
#define CONTROL_WATCHDOG_MS   500U
#define MANUAL_TIMEOUT_MS    2000U

/* ======================================================================
 * THÔNG SỐ CÂN BẰNG & DI CHUYỂN CASCADE PID
 * ====================================================================== */

/* Góc an toàn:
 * - FALL_ANGLE_DEG = 35°: Xe ngã -> ngắt PWM tạm thời (FALL_WAIT)
 * - Dựng xe lại <= 18° và tốc độ góc <= 120 dps: Tự cân bằng tiếp
 * - Lỗi khẩn cấp (MOTOR_CUT / IMU_FAULT / I2C_ERR): Chốt tắt vĩnh viễn tới khi bấm B1 hoặc gửi 'r' */
#define ARM_MAX_ANGLE_DEG      30.0f
#define ARM_MAX_RATE_DPS      120.0f
#define FALL_ANGLE_DEG         35.0f
#define FALL_RESUME_DEG        18.0f

/* Công suất & Bù vùng chết động cơ (TIM1 ARR = 999 -> 10 kHz) */
#define DEFAULT_PWM_LIMIT     999
#define ABSOLUTE_PWM_LIMIT    999
#define MANUAL_MAX_PWM        999
#define DEFAULT_DEADBAND      145

/* ---------------- VÒNG TRONG: PID GÓC (200 Hz) ----------------
 * Tăng độ cứng vòng cân bằng bù tải ESP32-C6 & ma sát hộp số GA12-N20:
 *   Kp = 62.00, Ki = 4.00, Kd = 6.60, Deadband = 145 */
#define DEFAULT_KP             62.00f
#define DEFAULT_KI              4.00f
#define DEFAULT_KD              6.60f
#define ANGLE_INTEGRAL_LIMIT   25.0f   /* Anti-windup */

/* Góc cân bằng cơ học mục tiêu đo thực tế trên khung xe (+0.50 độ).
 * Khóa giữ DEFAULT_ZERO_DEG khi bấm B1 để tránh ghi đè nhầm khi cầm nghiêng nhấn nút.
 * Quy ước chiều dương (+x, +enc, +theta_ref, phím W) = chiều TIẾN thực tế của xe
 * (đồng nhất dấu cả 5 hệ số để bảo toàn 100% chiều vòng cân bằng đã ổn định). */
#define DEFAULT_ZERO_DEG        0.50f
#define B1_CAPTURE_ZERO_ENABLE  0
#define B1_ZERO_CAPTURE_DEG     8.0f
#define DEFAULT_CONTROL_SIGN   -1
#define DEFAULT_LEFT_SIGN      -1
#define DEFAULT_RIGHT_SIGN     -1

/* Chiều đếm Encoder tĩnh đã xác nhận từ đo thực tế T2 (lăn tiến -> cả 2 bánh đếm dương) */
#define DEFAULT_ENC_LEFT_SIGN   1
#define DEFAULT_ENC_RIGHT_SIGN -1

/* ---------------- VÒNG NGOÀI: PID VỊ TRÍ ĐỨNG YÊN (50 Hz) ----------------
 *   Kp_pos = 2.40 độ/m, Kd_pos = 3.40 độ/(m/s)
 *   Ki_pos = 0.05 độ/(m.s), giới hạn tích phân 60 -> tối đa ±3.0° để chuyển mượt bumpless */
#define DEFAULT_KP_POS          2.40f
#define DEFAULT_KI_POS          0.05f
#define DEFAULT_KD_POS          3.40f
#define MAX_TARGET_ANGLE_DEG    8.0f
#define POS_INTEGRAL_LIMIT     60.0f
#define POS_LOOP_DIVIDER        4U     /* 50 Hz (20 ms) */
#define VEL_FILTER_N            8.0f   /* Bộ lọc đạo hàm bậc 1 (N = 8) */
#define POS_RUNAWAY_M           0.60f

/* ---------------- THÔNG SỐ ĐIỀU KHIỂN DI CHUYỂN TỪ ESP32-C6 ----------------
 *   - '1'/'2'/'3': chọn mức tốc độ mục tiêu (0.14 / 0.22 / 0.30 m/s)
 *   - 'w'/'s'/'a'/'d': đặt mục tiêu tiến/lùi/rẽ (không cộng dồn; ESP32 gửi lại mỗi 150ms)
 *   - 'k': vẫn hỗ trợ làm mới timeout để tương thích ngược
 *   - 'v'/'c': nhả tiến-lùi / nhả rẽ -> trả về 0 ngay lập tức */
#define MOTION_SPEED_SLOW       0.14f  /* Mức 1: Chậm an toàn để thử trên sàn */
#define MOTION_SPEED_MED        0.22f  /* Mức 2: Vừa (mặc định) */
#define MOTION_SPEED_FAST       0.30f  /* Mức 3: Nhanh tối đa */
#define MOTION_SPEED_MAX        0.30f
#define MOTION_ACCEL            0.50f  /* Gia tốc ramp v_ref (m/s^2) khởi động dứt khoát */
#define MOTION_DECEL            0.70f  /* Gia tốc giảm tốc (m/s^2) phanh mượt về 0 */
#define MOTION_KP_VEL           5.5f   /* Vòng PI vận tốc: hệ số P [độ/(m/s)] */
#define MOTION_KI_VEL           6.5f   /* Vòng PI vận tốc: hệ số I [độ/m] (tự bù zero & ma sát) */
#define MOTION_VEL_INT_LIMIT    5.5f   /* Giới hạn anti-windup tích phân vận tốc [độ] */
#define MOTION_THETA_LPF_ALPHA  0.35f  /* Làm mượt nhẹ theta_ref ở 200Hz (tau ~12ms, giữ phản xạ nhanh) */
#define MOTION_BRAKE_VEL_MPS    0.04f  /* Ngưỡng tốc độ thực để chuyển mượt về giữ vị trí đứng yên [m/s] */
#define MOTION_BRAKE_MAX_TICKS  50U    /* Tối đa 1.0s (50 tick @ 50Hz) phanh trước khi khóa vị trí */
#define MOTION_TRACK_ERR_MAX    0.25f  /* Giới hạn |x_ref - x| khi khóa vị trí đứng yên (m) */
#define MOTION_TILT_MAX_DEG     6.5f   /* Biên góc nghiêng mục tiêu khi chạy có encoder [độ] */
#define MOTION_NOENC_TILT_DEG   4.5f   /* Góc nghiêng mục tiêu dự phòng khi chưa có xung encoder [độ] */
#define MOTION_CMD_TIMEOUT_MS   500U   /* Tự trả tốc độ về 0 nếu mất lệnh > 500ms */
#define MOTION_TURN_PWM       170.0f   /* Xung PWM vi sai tối đa khi rẽ Trái/Phải */
#define MOTION_TURN_LPF_ALPHA   0.25f  /* Làm mượt rẽ hướng */
#define MOTION_TURN_TIMEOUT_MS  500U   /* Tự trả thẳng lái sau 500ms nếu mất lệnh */
#define MOTION_TURN_MAX_TILT_DEG 12.0f /* Cắt rẽ nếu thân xe đang nghiêng > 12° để ưu tiên giữ thăng bằng */

/* ---------------- ENCODER & BÁNH XE ĐO THỰC TẾ ---------------- */
#define WHEEL_RADIUS_M          0.022f /* Bánh xe đo thực tế D = 44 mm -> r = 0.022 m */
#define DEFAULT_ENC_CPR      1400.0f   /* 1:50 -> 7*4*50 = 1400 xung/vòng */
#define DEFAULT_ENC_ENABLE      1

/* Cấu hình trục cảm biến MPU6050 (1 = Trục Y: atan2(-ax, az); 0 = Trục X: atan2(ay, az)) */
#define IMU_TILT_AXIS           1
#define GYRO_LPF_ALPHA          0.70f
#define IMU_IO_TIMEOUT_MS       10U
#define IMU_CALIB_SAMPLES      150U    /* Calib nhanh ~0.3 giây */

#endif /* BOARD_CONFIG_H */
