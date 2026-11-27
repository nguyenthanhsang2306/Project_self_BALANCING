#ifndef BALANCE_CONTROLLER_H_
#define BALANCE_CONTROLLER_H_
#include "icm20602.h"
#include "motor.h"
#include "pid.h"

/* Cờ bão hòa & trạng thái vòng điều khiển */
#define CTRL_SAT_ANGLE_OUT     (1U << 0)
#define CTRL_SAT_ANGLE_INT     (1U << 1)
#define CTRL_SAT_THETA_REF     (1U << 2)
#define CTRL_SAT_TURN_CLIP     (1U << 3)
#define CTRL_SAT_IMU_PRED      (1U << 4)

/* Cấu trúc Cascade PID cân bằng đứng yên tại chỗ (x_ref = 0 + khóa hướng Yaw):
 *
 *  x_ref=0 ─►(+)─► PID vị trí CoM (50Hz) ─► θ_ref (lọc 200Hz) ─► PD góc (200Hz) ─► Balance-First Mixer ─► PWM
 *            (-)                                                    ▲ θ, θ' (ICM-20602)      ▲
 *             ▲ x_com, v_com (Encoder + Gyro)                                                │
 *                                                PD khóa hướng Yaw (enc_L - enc_R, imu.gz) ──┘
 */
typedef struct {
    ICM20602_t *imu;
    Motor_System_t *motor;
    PID_t pid_angle;            /* Vòng trong (200 Hz): PD cân bằng góc nghiêng (PWM/độ) */
    PID_t pid_pos;              /* Vòng ngoài (50 Hz): PID giữ vị trí khối tâm tại x = 0 (độ/m) */
    float nominal_balance_angle, output, turn_output;
    float yaw_kp, yaw_kd;       /* Hệ số PD khóa hướng Yaw có thể chỉnh live */
    int8_t control_sign, yaw_sign;
    bool armed;
    bool fall_waiting;
    bool encoder_enabled;       /* ENC 0: chỉ cân bằng góc; ENC 1: giữ đứng yên + khóa Yaw */
    uint8_t sat_flags;
    const char *reason;

    /* Biến trạng thái vòng ngoài giữ vị trí khối tâm (CoM) */
    float enc_cpr;              /* Số xung / vòng bánh (đã x4) */
    float x;                    /* Độ lệch vị trí khối tâm x_com [m] */
    float v;                    /* Vận tốc khối tâm v_com đã lọc [m/s] */
    float x_wheel;              /* Độ lệch trục bánh xe thuần [m] */
    float v_wheel;              /* Vận tốc trục bánh xe đã lọc [m/s] */
    float v_wheel_prev;         /* Vận tốc bánh chu kỳ trước để ước lượng a_lin cho Kalman */
    float a_wheel_filt;         /* Gia tốc bánh xe đã lọc thông thấp cho Kalman */
    float x_prev;
    float theta_ref;            /* Góc nghiêng bù giữ vị trí 50 Hz [độ] */
    float theta_ref_filt;       /* Góc nghiêng bù đã lọc mượt ở 200 Hz [độ] */
    float pos_dt_accum;         /* Tổng dt thực tế giữa 2 nhịp 50 Hz */
    float pos_rate_accum;       /* Tổng (tilt_rate * dt) cho tính v_com */
    uint8_t pos_counter;
} BalanceController_t;

void BalanceController_Init(BalanceController_t *, ICM20602_t *, Motor_System_t *);
void BalanceController_EmergencyStop(BalanceController_t *, const char *reason);
bool BalanceController_Arm(BalanceController_t *);
bool BalanceController_Update(BalanceController_t *, float dt);
void BalanceController_SetPositionLoop(BalanceController_t *, bool enable);

#endif
