#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include "icm20602.h"
#include "motor.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

/* Cấu trúc Cascade PID thuần đứng yên "đóng đinh tại chỗ" (x_ref = 0 + khóa hướng Yaw):
 *
 *  x_ref=0 ─►(+)─► PID vị trí CoM (50Hz) ─► θ_ref (lọc 200Hz) ─► PD góc (200Hz) ─► Balance-First Mixer ─► PWM
 *            (-)                                                    ▲ θ, θ' (ICM-20602)      ▲
 *             ▲ x_com, v_com (Encoder + Gyro)                                                │
 *                                                PD khóa hướng Yaw (enc_L - enc_R, imu.gz) ──┘
 */

/* Cờ bão hòa & sự kiện lỗi (đồng nhất bitmask với Project_Motion / ESP32 Web UI) */
#define CTRL_SAT_ANGLE_OUT     (1U << 0)
#define CTRL_SAT_ANGLE_INT     (1U << 1)
#define CTRL_SAT_THETA_REF     (1U << 2)
#define CTRL_SAT_TURN_CLIP     (1U << 3)
#define CTRL_SAT_IMU_PRED      (1U << 4)
#define CTRL_SAT_DEADLINE_MISS (1U << 5)
#define CTRL_SAT_I2C_ERR       (1U << 6)

typedef struct {
    ICM20602_t *imu;
    Motor_System_t *motor;
    PID_t pid_angle;            /* Vòng trong (200 Hz): PD cân bằng góc nghiêng thuần */
    PID_t pid_pos;              /* Vòng ngoài (50 Hz): PID giữ vị trí khối tâm tại x = 0 */
    float nominal_balance_angle;
    float output;               /* PWM cân bằng góc (-999 .. +999 trước deadband) */
    float turn_output;          /* PWM vi sai khóa hướng Yaw hiện tại */
    int8_t control_sign;
    bool armed;                 /* true = hệ thống được phép tự đứng / chờ dựng dậy */
    bool fall_waiting;          /* true = xe nằm ngã -> kéo STBY = LOW, chờ dựng thẳng <= 15° */
    bool fault_latched;         /* true = chốt lỗi an toàn (STBY = LOW), cần bấm B1 hoặc gửi 'r' */
    bool encoder_enabled;       /* Cờ bật vòng ngoài giữ vị trí & khóa hướng */
    bool enc_left_active;       /* Kênh trái đã từng có xung thật (phân loại NO_SIG vs STALL) */
    bool enc_right_active;      /* Kênh phải đã từng có xung thật (phân loại NO_SIG vs STALL) */
    bool enc_signal_active;     /* true ngay khi encoder sẵn sàng và không có lỗi kênh */
    bool enc_fault_left;        /* Mất xung bất đối xứng kênh trái (bánh phải vẫn quay) */
    bool enc_fault_right;       /* Mất xung bất đối xứng kênh phải (bánh trái vẫn quay) */
    bool enc_no_signal;         /* Cả 2 kênh chưa từng có xung dù PWM cao suốt 1.0s */
    bool motor_stalled;         /* Cả 2 bánh cùng kẹt cơ khí ở PWM cao suốt 1.0s */
    uint16_t enc_loss_l_ticks;
    uint16_t enc_loss_r_ticks;
    uint16_t motor_stall_ticks;
    int32_t enc_prev_l_total;
    int32_t enc_prev_r_total;
    int16_t last_pwm_left;      /* PWM trước deadband cấp bánh trái (dùng giám sát encoder) */
    int16_t last_pwm_right;     /* PWM trước deadband cấp bánh phải (dùng giám sát encoder) */
    uint8_t sat_flags;          /* Bitmask CTRL_SAT_* của chu kỳ hiện tại */
    const char *reason;

    /* Biến trạng thái vòng ngoài giữ vị trí khối tâm (CoM) */
    float enc_cpr;              /* Số xung 1 vòng bánh (đã x4) */
    float x;                    /* Độ lệch vị trí khối tâm x_com [m] */
    float v;                    /* Vận tốc ngang khối tâm v_com đã lọc [m/s] */
    float x_wheel;              /* Độ lệch trục bánh xe thuần [m] */
    float v_wheel;              /* Vận tốc trục bánh xe đã lọc [m/s] */
    float v_wheel_prev;         /* Vận tốc bánh chu kỳ trước để ước lượng a_lin cho Kalman */
    float x_prev;               /* Vị trí bánh xe chu kỳ trước */
    float theta_ref;            /* Góc nghiêng bù vòng ngoài 50 Hz [độ] */
    float theta_ref_filt;       /* Góc nghiêng bù đã lọc mượt ở 200 Hz [độ] */
    float pos_dt_accum;         /* Tổng dt thực tế giữa 2 nhịp 50 Hz */
    float pos_rate_accum;       /* Tổng (tilt_rate * dt) giữa 2 nhịp 50 Hz cho v_com */
    uint8_t pos_counter;
} BalanceController_t;

void BalanceController_Init(BalanceController_t *c, ICM20602_t *imu, Motor_System_t *motor);
void BalanceController_SetZeroAngle(BalanceController_t *c, float zero_angle);
void BalanceController_ClearFault(BalanceController_t *c);
void BalanceController_EmergencyStop(BalanceController_t *c, const char *reason);
bool BalanceController_Arm(BalanceController_t *c);
bool BalanceController_Update(BalanceController_t *c, float dt);
void BalanceController_SetPositionLoop(BalanceController_t *c, bool enable);

#endif /* BALANCE_CONTROLLER_H */
