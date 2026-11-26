#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ======================================================================
 * SƠ ĐỒ ĐẤU DÂY PHẦN CỨNG: STM32L476RG Nucleo-64 + TB6612FNG + ICM-20602 + ESP32-C6
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
#define CONTROL_ISR_WATCHDOG_TICKS 6U  /* Ngắt TIM6 tự cắt STBY/PWM nếu vòng chính trễ >= 30 ms */
#define CONTROL_MAX_MISSED_TICKS   3U  /* Chốt lỗi DEADLINE_ERR nếu bỏ lỡ >= 3 chu kỳ 5 ms */
#define CONTROL_MAX_DT_S       0.020f  /* Không cho tiếp tục điều khiển nếu dt > 20 ms */
#define IWDG_TIMEOUT_MS       500U     /* Hardware Watchdog IWDG tự reset MCU nếu treo > 500 ms */
#define MANUAL_TIMEOUT_MS    2000U

/* ======================================================================
 * THÔNG SỐ CÂN BẰNG & DI CHUYỂN CASCADE PID
 * ====================================================================== */

/* Góc an toàn (Fail-Safe nghiêm ngặt — KHÔNG tự ARM sau reset/ngã):
 * - Khởi động / Calib / bấm B1 -> luôn giữ DISARMED (PWM=0, STBY=LOW)
 * - Chỉ ARM khi có lệnh 'r' tường minh AND |tilt| <= 15° AND |rate| <= 80 dps AND IMU mới
 * - Nghiêng > 35° (FALL_ANGLE_DEG) -> PWM=0, STBY=LOW, chốt lỗi FALL_LATCH (không tự chạy lại) */
#define ARM_MAX_ANGLE_DEG      15.0f
#define ARM_MAX_RATE_DPS       80.0f
#define FALL_ANGLE_DEG         35.0f

/* Công suất & Bù vùng chết động cơ (TIM1 ARR = 999 -> 10 kHz)
 * - DEFAULT_DEADBAND = 130, MOTOR_DEADBAND_RAMP = 35 (~0.56°): làm mềm độ lợi quanh 0
 *   (từ 15.5x xuống 4.7x) để triệt tiêu gằn bánh răng hộp số GA12-N20 do nhiễu gyro,
 *   nhưng vẫn đạt đủ lực bứt ma sát tĩnh (~165 PWM) ngay khi nghiêng > 0.56°. */
#define DEFAULT_PWM_LIMIT     999
#define ABSOLUTE_PWM_LIMIT    999
#define MANUAL_MAX_PWM        999
#define DEFAULT_DEADBAND      130
#define MOTOR_DEADBAND_RAMP    35

/* ---------------- VÒNG TRONG: PD GÓC THUẦN (200 Hz) ----------------
 * Tắt tích phân vòng trong (DEFAULT_KI = 0.00f) để khử tích phân kép gây dao động
 * chu kỳ giới hạn (hunting) qua độ rơ hộp số N20; bù lệch tĩnh do vòng ngoài đảm nhiệm:
 *   Kp = 62.00, Ki = 0.00, Kd = 6.60 */
#define DEFAULT_KP             62.00f
#define DEFAULT_KI              0.00f
#define DEFAULT_KD              6.60f
#define ANGLE_INTEGRAL_LIMIT   25.0f   /* Dự phòng nếu bật Ki thủ công */

/* Góc cân bằng cơ học mục tiêu đo thực tế trên khung xe với module ICM-20602 (+1.50 độ).
 * Khóa giữ DEFAULT_ZERO_DEG khi bấm B1 để tránh ghi đè nhầm khi cầm nghiêng nhấn nút.
 * Quy ước chiều dương (+x, +enc, +theta_ref, phím W) = chiều TIẾN thực tế của xe
 * (đồng nhất dấu cả 5 hệ số để bảo toàn 100% chiều vòng cân bằng đã ổn định). */
#define DEFAULT_ZERO_DEG        1.50f
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
#define MOTION_TURN_LPF_ALPHA   0.12f  /* Làm mượt tốc độ góc rẽ mục tiêu ở 200Hz (tau ~40ms) */
#define MOTION_TURN_TIMEOUT_MS  500U   /* Tự trả thẳng lái sau 500ms nếu mất lệnh */
#define MOTION_TURN_MAX_TILT_DEG 12.0f /* Cắt rẽ nếu thân xe đang nghiêng > 12° để ưu tiên giữ thăng bằng */

/* ---------------- KHÓA HƯỚNG & RẼ BẰNG GYRO Z (imu.gz @ 200 Hz) ----------------
 * Trục +Z của ICM-20602 hướng LÊN (az > 0), nên quay TRÁI (CCW) cho gz > 0, quay PHẢI (CW) cho gz < 0.
 * DEFAULT_YAW_SIGN = -1 quy đổi yaw_rate_dps > 0 = quay PHẢI (cùng dấu cmd_turn_pwm > 0 khi bấm 'D'). */
#define DEFAULT_YAW_SIGN         (-1)
#define MOTION_TURN_RATE_DPS     95.0f /* Tốc độ góc rẽ mục tiêu khi giữ A/D [độ/s] */
#define MOTION_YAW_FF            1.15f /* Feedforward vi sai theo tốc độ góc đặt [PWM/(độ/s)] */
#define MOTION_KP_YAW_RATE       1.10f /* Hệ số P khóa tốc độ góc yaw [PWM/(độ/s)] */
#define MOTION_KI_YAW_HEADING    8.00f /* Hệ số giữ góc hướng đi thẳng (tự bù lệch 2 motor N20) [PWM/độ] */
#define MOTION_YAW_HEADING_LIM  12.0f  /* Biên tích lũy sai số góc hướng [độ] (tối đa ±96 PWM bù lệch) */
#define MOTION_YAW_DEADZONE_DPS  0.35f /* Ngưỡng bỏ qua trôi tĩnh gyro Z khi đứng yên [độ/s] */

/* ---------------- BÙ VẬN TỐC KHỐI TÂM (CoM) & ENCODER ĐO THỰC TẾ ---------------- */
#define COM_HEIGHT_M             0.035f /* Khoảng cách trục bánh -> khối tâm thân xe l_com [m] */
#define WHEEL_RADIUS_M           0.022f /* Bánh xe đo thực tế D = 44 mm -> r = 0.022 m */
#define DEFAULT_ENC_CPR       1400.0f   /* 1:50 -> 7*4*50 = 1400 xung/vòng */
#define DEFAULT_ENC_ENABLE       1
#define ENC_ACTIVE_MIN_PULSES    4.0f   /* Ngưỡng ghi nhận bánh đã từng phát xung (chỉ để phân loại NO_SIG vs STALL) */
#define ENC_LOSS_PWM_THRESH    320      /* Ngưỡng PWM phát hiện mất xung bất đối xứng 1 kênh */
#define ENC_LOSS_OTHER_MIN_D     4      /* Kênh còn lại phải đang quay >= 4 xung/tick mới kết luận mất encoder */
#define ENC_LOSS_TICKS          20U     /* 20 tick @ 50Hz = 0.40s mất xung 1 kênh khi bánh kia vẫn quay -> ENC_LOSS */
#define MOTOR_STALL_PWM_THRESH 420      /* Ngưỡng PWM cao (~550 sau deadband) giám sát kẹt cả 2 bánh */
#define MOTOR_STALL_TICKS       50U     /* 50 tick @ 50Hz = 1.00s cả 2 bánh đứng khựng -> MOTOR_STALL / ENC_NO_SIG */

/* Cấu hình trục cảm biến ICM-20602 & Kalman chống lệch gia tốc:
 *   1 = Trục Y: atan2(-ax, az); 0 = Trục X: atan2(ay, az)
 *   - ICM-20602 DLPF Gyro = 2 (92 Hz, trễ 3.9 ms) & DLPF Accel = 3 (44.8 Hz; đổi thành 4 = 21.2 Hz nếu rung mạnh).
 *   - I2C1 mặc định 200 kHz (0x10425F73) an toàn dây Dupont; nếu dây ngắn có thể nâng 400 kHz (0x1042203A). */
#define IMU_TILT_AXIS            1
#define ICM20602_GYRO_DLPF_CFG   2U     /* Thanh ghi 0x1A: 2 = 92 Hz (trễ 3.9 ms) */
#define ICM20602_ACCEL_DLPF_CFG  3U     /* Thanh ghi 0x1D: 3 = 44.8 Hz (4 = 21.2 Hz) */
#define GYRO_LPF_ALPHA          0.70f
#define IMU_MAX_LIN_ACC_G       0.25f   /* Giới hạn bù gia tốc ngang bánh xe đưa vào Kalman [g] */
#define IMU_KALMAN_R_BASE       0.03f   /* Hiệp phương sai nhiễu đo góc gia tốc kế khi đứng yên */
#define IMU_KALMAN_R_LIN_SCALE 40.0f    /* Hệ số tăng R thích nghi khi xe tăng tốc / phanh */
#define IMU_KALMAN_R_NORM_SCALE 20.0f   /* Hệ số tăng R thích nghi khi rung động làm |norm - 1g| lệch 0 */
#define IMU_BIAS_FREEZE_ACC_G   0.025f  /* Khóa cập nhật bias gyro khi |a_lin| > 0.025g (~0.25 m/s^2) */
#define IMU_BIAS_FREEZE_NORM_G  0.06f   /* Khóa cập nhật bias gyro khi |norm - 1g| > 0.06g */
#define IMU_IO_TIMEOUT_MS      10U     /* Timeout I2C lúc Init/Calib (khi motor đang tắt) */
#define IMU_RUNTIME_TIMEOUT_US 2500U   /* Giới hạn cứng 2.5 ms (đo bằng DWT) khi đọc IMU ở vòng 200 Hz */
#define IMU_MAX_CONSEC_MISS     3U     /* Tối đa 3 chu kỳ (15 ms) dự đoán liên tiếp khi mất mẫu mới */
#define IMU_MAX_AGE_MS         20U     /* Tuổi mẫu IMU tươi tối đa 20 ms -> quá hạn cắt motor ngay */
#define IMU_CALIB_SAMPLES     300U     /* Calib ~0.6 giây (300 mẫu) giảm nhiễu bias gyro kèm kiểm tra xe đứng yên */
#define IMU_CALIB_MAX_GYRO_SPREAD_DPS 6.0f  /* Chặn calib nếu gyro dao động > 6 dps */
#define IMU_CALIB_MAX_ACC_SPREAD_G    0.15f /* Chặn calib nếu gia tốc dao động > 0.15g */
#define IMU_CALIB_MAX_BIAS_DPS        8.0f  /* Sai lệch tĩnh gyro hợp lệ tối đa ±8 dps */

/* Chu kỳ Telemetry (40 ms = 25 Hz) */
#define TELEMETRY_PERIOD_MS    40U

#endif /* BOARD_CONFIG_H */
