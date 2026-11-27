#ifndef BALANCE_CONTROLLER_H_
#define BALANCE_CONTROLLER_H_
#include "icm20602.h"
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
 *            ▲ x, v (Encoder)                             ▲ θ, θ' (ICM-20602)
 */
/* Cờ bão hòa & sự kiện lỗi (chốt OR qua toàn bộ cửa sổ 40 ms giữa 2 gói telemetry) */
#define CTRL_SAT_ANGLE_OUT     (1U << 0) /* Đầu ra PID vòng góc chạm biên ±effective */
#define CTRL_SAT_ANGLE_INT     (1U << 1) /* Tích phân vòng góc chạm ANGLE_INTEGRAL_LIMIT */
#define CTRL_SAT_THETA_REF     (1U << 2) /* Góc mục tiêu theta_ref chạm biên */
#define CTRL_SAT_TURN_CLIP     (1U << 3) /* Lực rẽ bị co theo headroom cân bằng hoặc cắt > 12° */
#define CTRL_SAT_IMU_PRED      (1U << 4) /* Có ít nhất 1 chu kỳ dùng góc dự đoán (mất mẫu IMU mới) */
#define CTRL_SAT_DEADLINE_MISS (1U << 5) /* Có ít nhất 1 chu kỳ bị lỡ deadline 5 ms trong cửa sổ */
#define CTRL_SAT_I2C_ERR       (1U << 6) /* Có phát sinh lỗi đọc I2C trong cửa sổ telemetry */

typedef struct {
    ICM20602_t *imu;
    Motor_System_t *motor;
    PID_t pid_angle;            /* Vòng trong (200 Hz): góc nghiêng (PWM/độ) */
    PID_t pid_pos;              /* Vòng ngoài đứng yên (50 Hz): giữ vị trí (độ/m) */
    float nominal_balance_angle, output;
    int8_t control_sign;
    bool armed;
    bool fault_latched;         /* Chốt an toàn sau lỗi / ngã: yêu cầu lệnh ARM hợp lệ để mở lại */
    bool encoder_enabled;
    bool enc_left_active;       /* Kênh trái đã xác nhận >= 24 xung thật */
    bool enc_right_active;      /* Kênh phải đã xác nhận >= 24 xung thật */
    bool enc_signal_active;     /* true chỉ khi CẢ 2 kênh encoder đều hoạt động hợp lệ */
    bool enc_fault_left;        /* Mất xung bất đối xứng kênh trái (bánh phải vẫn quay) */
    bool enc_fault_right;       /* Mất xung bất đối xứng kênh phải (bánh trái vẫn quay) */
    bool enc_no_signal;         /* Cả 2 kênh chưa từng phát xung dù PWM cao 1.0s */
    bool motor_stalled;         /* Cả 2 bánh cùng kẹt cơ khí / bị giữ chặt ở PWM cao 1.0s */
    uint16_t enc_loss_l_ticks;  /* Bộ đếm mất xung bất đối xứng kênh trái */
    uint16_t enc_loss_r_ticks;  /* Bộ đếm mất xung bất đối xứng kênh phải */
    uint16_t motor_stall_ticks; /* Bộ đếm kẹt đối xứng cả 2 bánh */
    int32_t enc_prev_l_total;
    int32_t enc_prev_r_total;
    int16_t last_pwm_left;      /* PWM đặt cho bánh trái (trước deadband) để giám sát encoder */
    int16_t last_pwm_right;     /* PWM đặt cho bánh phải (trước deadband) để giám sát encoder */
    uint8_t sat_flags;          /* Bitmask cờ bão hòa/sự kiện chu kỳ hiện tại CTRL_SAT_* */
    const char *reason;

    /* Biến trạng thái vòng ngoài (Position Hold / Velocity PI + bù khối tâm CoM) */
    float enc_cpr;              /* Số xung / vòng bánh (đã x4) */
    float x;                    /* Vị trí ngang khối tâm x_com [m] */
    float v;                    /* Vận tốc ngang khối tâm v_com đã lọc [m/s] */
    float x_wheel;              /* Vị trí ngang trục bánh xe [m] */
    float v_wheel;              /* Vận tốc dài trục bánh xe đã lọc [m/s] */
    float v_wheel_prev;         /* Vận tốc bánh chu kỳ trước để tính gia tốc ngang cho Kalman */
    float a_wheel_filt;         /* Gia tốc bánh xe đã lọc thông thấp cho Kalman */
    float x_prev;               /* Vị trí bánh xe chu kỳ trước */
    float theta_ref;            /* Góc nghiêng tham chiếu vòng ngoài 50Hz [độ] */
    float theta_ref_filt;       /* Góc nghiêng tham chiếu đã làm mượt ở 200Hz [độ] */
    float vel_integral;         /* Thành phần tích phân vòng PI vận tốc [độ] */
    float pos_dt_accum;         /* Tổng dt thực tế tích lũy cho vòng ngoài (khử sai số jitter) */
    float pos_rate_accum;       /* Tổng (tilt_rate * dt) tích lũy giữa 2 chu kỳ vòng ngoài [độ] */
    float yaw_rate_ref;         /* Tốc độ góc rẽ mục tiêu đã làm mượt [độ/s] */
    float yaw_heading_err;      /* Sai số góc hướng tích lũy khi đi thẳng / đứng yên [độ] */
    bool was_moving;            /* Trạng thái chu kỳ trước để chuyển mượt */
    uint8_t brake_ticks;        /* Đếm thời gian phanh về 0 trước khi khóa vị trí */
    uint8_t pos_counter;

    /* Biến điều khiển di chuyển nhận từ ESP32-C6 */
    float speed_target_mag;     /* Mức tốc độ đang chọn (0.14 / 0.22 / 0.30 m/s) */
    float v_cmd;                /* Vận tốc đặt [m/s] (+: tiến, -: lùi) */
    float v_ref;                /* Vận tốc tham chiếu đã ramp gia tốc [m/s] */
    float x_ref;                /* Điểm đích đứng yên [m] */
    float cmd_turn_pwm;         /* Xung vi sai rẽ hướng đặt (+: phải, -: trái) */
    float filtered_turn_pwm;    /* Xung vi sai rẽ hướng đã làm mượt */
    uint32_t last_speed_cmd_ms; /* Thời điểm nhận lệnh W/S gần nhất */
    uint32_t last_turn_cmd_ms;  /* Thời điểm nhận lệnh A/D gần nhất */
    char last_cmd;              /* Ký tự lệnh gần nhất ('W','S','A','D','X') */
} BalanceController_t;

void BalanceController_Init(BalanceController_t *c, ICM20602_t *imu, Motor_System_t *motor);
void BalanceController_ClearFault(BalanceController_t *c);
void BalanceController_EmergencyStop(BalanceController_t *c, const char *reason);
bool BalanceController_Arm(BalanceController_t *c);
bool BalanceController_Update(BalanceController_t *c, float dt);
void BalanceController_SetPositionLoop(BalanceController_t *c, bool enable);
void BalanceController_SetZeroAngle(BalanceController_t *c, float zero_angle);
void BalanceController_SetMotion(BalanceController_t *c, float speed_mps, float turn_pwm);
void BalanceController_ProcessCommandChar(BalanceController_t *c, char cmd);

#endif /* BALANCE_CONTROLLER_H_ */
