#include "balance_controller.h"
#include "board_config.h"
#include <math.h>
#include <string.h>

#define TWO_PI 6.28318531f

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Đặt lại gốc vị trí (x = 0 tại vị trí hiện tại của xe) */
static void reset_position(BalanceController_t *c) {
    Motor_ResetEncoders(c->motor);
    c->x = c->x_prev = 0.0f;
    c->v = 0.0f;
    c->theta_ref = 0.0f;
    c->pos_counter = 0;
    c->enc_no_pulse_ticks = 0U;
    c->enc_prev_abs_sum = 0;
    PID_Reset(&c->pid_pos);
    PID_SetTarget(&c->pid_pos, 0.0f);
}

void BalanceController_Init(BalanceController_t *c, MPU6050_t *imu, Motor_System_t *motor) {
    memset(c, 0, sizeof(*c));
    c->imu = imu;
    c->motor = motor;
    c->nominal_balance_angle = DEFAULT_ZERO_DEG;
    c->control_sign = DEFAULT_CONTROL_SIGN;
    c->reason = "STOP";
    c->fault_latched = false;
    c->encoder_enabled = (DEFAULT_ENC_ENABLE != 0 && motor != NULL && motor->enc_ready);
    c->enc_signal_active = false;
    c->enc_cpr = DEFAULT_ENC_CPR;

    /* Vòng trong: PID góc nghiêng 200 Hz */
    PID_Init(&c->pid_angle, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
             -(float)motor->limit, (float)motor->limit, ANGLE_INTEGRAL_LIMIT);

    /* Vòng ngoài: PID giữ vị trí đứng yên x = 0 (50 Hz), bão hòa góc nghiêng bù ±8° */
    PID_Init(&c->pid_pos, DEFAULT_KP_POS, DEFAULT_KI_POS, DEFAULT_KD_POS,
             -MAX_TARGET_ANGLE_DEG, MAX_TARGET_ANGLE_DEG, POS_INTEGRAL_LIMIT);
}

void BalanceController_SetZeroAngle(BalanceController_t *c, float zero_angle) {
    c->nominal_balance_angle = zero_angle;
}

void BalanceController_ClearFault(BalanceController_t *c) {
    c->fault_latched = false;
}

void BalanceController_EmergencyStop(BalanceController_t *c, const char *reason) {
    Motor_Stop(c->motor);
    c->armed = false;
    c->fall_waiting = false;
    c->fault_latched = true;    /* Chốt an toàn: không cho tự bật lại ở chu kỳ sau */
    c->output = 0.0f;
    c->reason = reason;
    PID_Reset(&c->pid_angle);
    reset_position(c);
}

bool BalanceController_Arm(BalanceController_t *c) {
    if (c->fault_latched) {
        return false;           /* Phải xóa chốt lỗi bằng nút B1 trước khi ARM lại */
    }
    if (!c->imu->ready || HAL_GetTick() - c->imu->last_sample_ms > 200U ||
        !isfinite(c->imu->pitch_kalman) || !isfinite(c->imu->gyro_rate_pitch) ||
        c->pid_angle.Kp <= 0 || !c->motor->ready) {
        return false;
    }
    PID_Reset(&c->pid_angle);
    reset_position(c);
    float tilt_err = fabsf(c->imu->pitch_kalman - c->nominal_balance_angle);
    c->fall_waiting = (tilt_err > FALL_RESUME_DEG || fabsf(c->imu->gyro_rate_pitch) > ARM_MAX_RATE_DPS);
    c->armed = Motor_Enable(c->motor);
    c->reason = !c->armed ? "PWM_ERROR" : (c->fall_waiting ? "FALL_WAIT" : "BALANCE");
    return c->armed;
}

void BalanceController_SetPositionLoop(BalanceController_t *c, bool enable) {
    reset_position(c);
    c->encoder_enabled = enable && c->motor->enc_ready;
    if (!c->encoder_enabled) c->enc_signal_active = false;
}

/* Vòng ngoài 50 Hz: giữ vị trí đứng yên tại chỗ (x_ref = 0)
 *   θ_ref = Kp_pos*(0 - x) + Ki_pos*∫(0 - x)dt + Kd_pos*(0 - v)
 * Nếu chưa có xung Encoder hợp lệ (hoặc đứt dây khi motor đang kéo mạnh), giữ θ_ref = 0°. */
static void position_hold_loop(BalanceController_t *c, float dtp) {
    int32_t l_tot_i = c->motor->enc_left_total;
    int32_t r_tot_i = c->motor->enc_right_total;
    int32_t abs_sum = (l_tot_i < 0 ? -l_tot_i : l_tot_i) + (r_tot_i < 0 ? -r_tot_i : r_tot_i);
    float l_tot = (float)l_tot_i;
    float r_tot = (float)r_tot_i;

    /* Yêu cầu ít nhất 24 xung thật để xác nhận Encoder đã cắm (tránh nhiễu gai 1-2 xung) */
    if (!c->enc_signal_active && (fabsf(l_tot) >= 24.0f || fabsf(r_tot) >= 24.0f)) {
        c->enc_signal_active = true;
        c->enc_no_pulse_ticks = 0U;
        c->enc_prev_abs_sum = abs_sum;
    }

    /* Nếu đang bật encoder mà motor kéo mạnh (>350 PWM) suốt 0.5s (25 tick 50Hz) không nhúc nhích xung -> ngắt an toàn vòng vị trí */
    if (c->enc_signal_active) {
        if (fabsf(c->output) > 350.0f) {
            int32_t d_abs = abs_sum - c->enc_prev_abs_sum;
            if (d_abs < 0) d_abs = -d_abs;
            if (d_abs < 2) {
                if (++c->enc_no_pulse_ticks >= 25U) {
                    c->enc_signal_active = false;
                    c->enc_no_pulse_ticks = 0U;
                    reset_position(c);
                    return;
                }
            } else {
                c->enc_no_pulse_ticks = 0U;
            }
        } else {
            c->enc_no_pulse_ticks = 0U;
        }
        c->enc_prev_abs_sum = abs_sum;
    }

    if (!c->enc_signal_active) {
        c->x = c->x_prev = 0.0f;
        c->v = 0.0f;
        c->theta_ref = 0.0f;
        return;
    }

    /* Đổi số xung trung bình sang quãng đường x [mét] */
    float counts = 0.5f * (l_tot + r_tot);
    if (fabsf(l_tot) < 5.0f && fabsf(r_tot) > 30.0f) counts = r_tot;
    else if (fabsf(r_tot) < 5.0f && fabsf(l_tot) > 30.0f) counts = l_tot;

    float cpr = (c->enc_cpr > 1.0f) ? c->enc_cpr : DEFAULT_ENC_CPR;
    c->x = counts / cpr * TWO_PI * WHEEL_RADIUS_M;

    /* Vận tốc qua bộ lọc đạo hàm bậc 1 (N = 10) */
    float raw_v = (c->x - c->x_prev) / dtp;
    c->x_prev = c->x;
    float alpha = dtp / (dtp + 1.0f / VEL_FILTER_N);
    c->v += alpha * (raw_v - c->v);

    /* Nếu xe bị ngoại lực nhấc/đẩy đi quá xa (> 60cm), đặt lại vị trí mới làm điểm đứng yên */
    if (fabsf(c->x) > POS_RUNAWAY_M) {
        reset_position(c);
        return;
    }

    /* PID giữ vị trí: mục tiêu x_ref = 0, khâu D dùng sai lệch vận tốc (0 - v) */
    PID_SetTarget(&c->pid_pos, 0.0f);
    float th = PID_ComputeWithDerivative(&c->pid_pos, c->x, -c->v, dtp);
    c->theta_ref = clampf(th, -MAX_TARGET_ANGLE_DEG, MAX_TARGET_ANGLE_DEG);
}

bool BalanceController_Update(BalanceController_t *c, float dt) {
    if (!c->armed || c->fault_latched) return false;
    if (!c->imu->ready || HAL_GetTick() - c->imu->last_sample_ms > 200U) {
        BalanceController_EmergencyStop(c, "IMU_FAULT");
        return false;
    }
    float angle = c->imu->pitch_kalman;
    float rate  = c->imu->gyro_rate_pitch;
    if (!isfinite(angle) || !isfinite(rate)) {
        BalanceController_EmergencyStop(c, "CONTROL_DATA");
        return false;
    }

    /* Quy ước: θ > 0 = nghiêng về phía PWM dương (phía trước) */
    float tilt      = -(float)c->control_sign * (angle - c->nominal_balance_angle);
    float tilt_rate = -(float)c->control_sign * rate;
    float tilt_err  = fabsf(tilt);

    /* === 1. BẢO VỆ NGÃ (> 35°) -> ngắt motor; DỰNG THẲNG LẠI (<= 18°) -> tự cân bằng === */
    if (!c->fall_waiting && tilt_err > FALL_ANGLE_DEG) {
        c->fall_waiting = true;
    }
    if (c->fall_waiting) {
        if (tilt_err <= FALL_RESUME_DEG && fabsf(rate) <= ARM_MAX_RATE_DPS) {
            c->fall_waiting = false;
            PID_Reset(&c->pid_angle);
            reset_position(c);
        } else {
            Motor_SetSpeed(c->motor, 0, 0);
            PID_Reset(&c->pid_angle);
            c->output = 0.0f;
            c->reason = "FALL_WAIT";
            return true;
        }
    }

    /* === 2. VÒNG NGOÀI: GIỮ VỊ TRÍ ĐỨNG YÊN (50 Hz / mỗi 20 ms) ===
     * Ghi chú an toàn: KHÔNG tự lật dấu Encoder khi đang chạy; chiều đếm cố định theo board_config.h */
    if (c->encoder_enabled) {
        if (++c->pos_counter >= POS_LOOP_DIVIDER) {
            c->pos_counter = 0;
            position_hold_loop(c, dt * (float)POS_LOOP_DIVIDER);
        }
    } else {
        c->theta_ref = 0.0f;
    }

    /* === 3. VÒNG TRONG: CÂN BẰNG GÓC (200 Hz / mỗi 5 ms) ===
     *     PWM = Kp*(θ - θ_ref) + Ki*∫(θ - θ_ref)dt + Kd*θ' */
    c->reason = "BALANCE";
    int effective = c->motor->limit - c->motor->deadband;
    if (effective < 1) effective = 1;
    c->pid_angle.out_min = (float)-effective;
    c->pid_angle.out_max = (float)effective;
    PID_SetTarget(&c->pid_angle, 0.0f);
    c->output = PID_ComputeWithDerivative(&c->pid_angle, c->theta_ref - tilt, tilt_rate, dt);

    /* Cấp tốc độ đối xứng cho 2 bánh để đứng cân bằng tại chỗ */
    int16_t pwm = (int16_t)lroundf(c->output);
    if (!Motor_SetSpeed(c->motor, pwm, pwm)) {
        BalanceController_EmergencyStop(c, "MOTOR_CUT");
        return false;
    }
    return true;
}
