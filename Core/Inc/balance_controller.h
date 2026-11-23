#ifndef BALANCE_CONTROLLER_H_
#define BALANCE_CONTROLLER_H_
#include "mpu6050.h"
#include "motor.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

/* Cấu trúc Cascade PID cân bằng & điều khiển di chuyển từ ESP32-C6:
 *
 *  [WebSocket: W/S/A/D, V/C, 1/2/3] ──► v_cmd / cmd_turn_pwm (Gửi lại mỗi 150ms khi giữ)
 *                                              │
 *  Đứng yên: PID vị trí (x_ref)  ─┐
 *  Di chuyển: PI vận tốc (v_ref) ─┴─► θ_ref (bumpless) ─► PID góc (200Hz) ─► Balance-First Mixer ─► PWM
 *            ▲ x, v (Encoder)                             ▲ θ, θ' (MPU6050)
 */
typedef struct {
    MPU6050_t *imu;
    Motor_System_t *motor;
    PID_t pid_angle;            /* Vòng trong (200 Hz): góc nghiêng (PWM/độ) */
    PID_t pid_pos;              /* Vòng ngoài đứng yên (50 Hz): giữ vị trí (độ/m) */
    float nominal_balance_angle, output;
    int8_t control_sign;
    bool armed;
    bool fall_waiting;
    bool fault_latched;         /* Chốt an toàn sau dừng khẩn cấp: không tự bật lại ở chu kỳ sau */
    bool encoder_enabled;
    bool enc_signal_active;     /* true khi đã xác nhận có xung Encoder hợp lệ (không tự tắt khi kẹt) */
    const char *reason;

    /* Biến trạng thái vòng ngoài (Position Hold / Velocity PI) */
    float enc_cpr;              /* Số xung / vòng bánh (đã x4) */
    float x;                    /* Độ lệch vị trí [m] */
    float v;                    /* Vận tốc dài đã lọc [m/s] */
    float x_prev;
    float theta_ref;            /* Góc nghiêng tham chiếu vòng ngoài 50Hz [độ] */
    float theta_ref_filt;       /* Góc nghiêng tham chiếu đã làm mượt ở 200Hz [độ] */
    float vel_integral;         /* Thành phần tích phân vòng PI vận tốc [độ] */
    bool was_moving;            /* Trạng thái chu kỳ trước để chuyển mượt bumpless */
    uint8_t brake_ticks;        /* Đếm thời gian phanh về 0 trước khi khóa vị trí */
    uint8_t pos_counter;

    /* Biến điều khiển di chuyển nhận từ ESP32-C6 */
    float speed_target_mag;     /* Mức tốc độ đang chọn (0.14 / 0.22 / 0.30 m/s) */
    float v_cmd;                /* Vận tốc đặt [m/s] (+: tiến, -: lùi) */
    float v_ref;                /* Vận tốc tham chiếu đã ramp gia tốc [m/s] */
    float x_ref;                /* Điểm đích di động [m] (x_ref += v_ref*dt) */
    float cmd_turn_pwm;         /* Xung vi sai rẽ hướng đặt (+: phải, -: trái) */
    float filtered_turn_pwm;    /* Xung vi sai rẽ hướng đã làm mượt */
    uint32_t last_speed_cmd_ms; /* Thời điểm nhận lệnh hoặc heartbeat W/S gần nhất */
    uint32_t last_turn_cmd_ms;  /* Thời điểm nhận lệnh hoặc heartbeat A/D gần nhất */
    uint32_t last_any_cmd_ms;   /* Hỗ trợ test quay thử khi xe đang nằm ở FALL_WAIT */
    char last_cmd;              /* Ký tự lệnh gần nhất ('W','S','A','D','X') */
} BalanceController_t;

void BalanceController_Init(BalanceController_t *c, MPU6050_t *imu, Motor_System_t *motor);
void BalanceController_ClearFault(BalanceController_t *c);
void BalanceController_EmergencyStop(BalanceController_t *c, const char *reason);
bool BalanceController_Arm(BalanceController_t *c);
bool BalanceController_Update(BalanceController_t *c, float dt);
void BalanceController_SetPositionLoop(BalanceController_t *c, bool enable);
void BalanceController_SetZeroAngle(BalanceController_t *c, float zero_angle);
void BalanceController_SetMotion(BalanceController_t *c, float speed_mps, float turn_pwm);
void BalanceController_ProcessCommandChar(BalanceController_t *c, char cmd);

#endif /* BALANCE_CONTROLLER_H_ */
