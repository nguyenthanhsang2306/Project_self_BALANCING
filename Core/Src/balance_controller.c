#include "balance_controller.h"
#include "board_config.h"
#include <math.h>
#include <string.h>

#define TWO_PI     6.28318531f
#define DEG_TO_RAD 0.01745329252f
#define GRAVITY_G  9.80665f

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Đặt lại toàn bộ gốc vị trí & tích phân (khi ARM / STOP / dựng dậy từ FALL_WAIT) */
static void reset_position(BalanceController_t *c) {
    Motor_ResetEncoders(c->motor);
    c->x = c->x_wheel = c->x_prev = 0.0f;
    c->v = c->v_wheel = c->v_wheel_prev = 0.0f;
    c->a_wheel_filt = 0.0f;
    c->theta_ref = 0.0f;
    c->theta_ref_filt = 0.0f;
    c->turn_output = 0.0f;
    c->pos_dt_accum = 0.0f;
    c->pos_rate_accum = 0.0f;
    c->pos_counter = 0;
    if (c->imu != NULL) {
        ICM20602_SetLinearAccel(c->imu, 0.0f);
    }
    PID_Reset(&c->pid_pos);
    PID_SetTarget(&c->pid_pos, 0.0f);
}

/* Đặt lại gốc quãng đường khi bị đẩy/nhấc vượt POS_RUNAWAY_M (0.60 m),
 * NHƯNG GIỮ NGUYÊN pid_pos.integral để không làm mất góc bù lệch trọng tâm tĩnh. */
static void rebase_position_origin(BalanceController_t *c) {
    float saved_integral = clampf(c->pid_pos.integral,
                                  -c->pid_pos.integral_limit, c->pid_pos.integral_limit);
    Motor_ResetEncoders(c->motor);
    c->x = 0.0f;
    c->x_wheel = 0.0f;
    c->x_prev = 0.0f;
    c->pid_pos.prev_error = 0.0f;
    c->pid_pos.integral = saved_integral;
    PID_SetTarget(&c->pid_pos, 0.0f);
}

void BalanceController_Init(BalanceController_t *c, ICM20602_t *imu, Motor_System_t *motor) {
    memset(c, 0, sizeof(*c));
    c->imu = imu;
    c->motor = motor;
    c->nominal_balance_angle = DEFAULT_ZERO_DEG;
    c->control_sign = DEFAULT_CONTROL_SIGN;
    c->yaw_sign = DEFAULT_YAW_SIGN;
    c->yaw_kp = YAW_HOLD_KP_PULSE;
    c->yaw_kd = YAW_HOLD_KD_RATE;
    c->reason = "STOP";
    c->encoder_enabled = (DEFAULT_ENC_ENABLE != 0 && motor != NULL && motor->enc_ready);
    c->enc_cpr = DEFAULT_ENC_CPR;

    /* Vòng trong (200 Hz): PD góc (mặc định DEFAULT_KI = 0.0f) */
    PID_Init(&c->pid_angle, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
             -(float)motor->limit, (float)motor->limit, ANGLE_INTEGRAL_LIMIT);

    /* Vòng ngoài (50 Hz): PID giữ vị trí khối tâm tại x_ref = 0, bão hòa +-8 độ */
    PID_Init(&c->pid_pos, DEFAULT_KP_POS, DEFAULT_KI_POS, DEFAULT_KD_POS,
             -MAX_TARGET_ANGLE_DEG, MAX_TARGET_ANGLE_DEG, POS_INTEGRAL_LIMIT);
}

void BalanceController_EmergencyStop(BalanceController_t *c, const char *reason) {
    Motor_Stop(c->motor);
    c->armed = false;
    c->fall_waiting = false;
    c->output = 0.0f;
    c->turn_output = 0.0f;
    c->sat_flags = 0U;
    c->reason = reason;
    PID_Reset(&c->pid_angle);
    reset_position(c);
}

bool BalanceController_Arm(BalanceController_t *c) {
    uint32_t now = HAL_GetTick();
    if (!c->imu->ready || !c->imu->last_read_fresh ||
        c->imu->sample_age_ms > IMU_MAX_AGE_MS ||
        (now - c->imu->last_sample_ms) > IMU_MAX_AGE_MS ||
        !isfinite(c->imu->pitch_kalman) || !isfinite(c->imu->gyro_rate_pitch) ||
        c->pid_angle.Kp <= 0.0f || !c->motor->ready) {
        return false;
    }
    PID_Reset(&c->pid_angle);
    reset_position(c);
    c->sat_flags = 0U;
    float tilt_err = fabsf(c->imu->pitch_kalman - c->nominal_balance_angle);
    c->fall_waiting = (tilt_err > FALL_RESUME_DEG || fabsf(c->imu->gyro_rate_pitch) > ARM_MAX_RATE_DPS);
    if (c->fall_waiting) {
        /* Xe đang nằm nghiêng khi ARM -> KÉO STBY = LOW, chờ dựng thẳng <= 15 độ */
        Motor_Stop(c->motor);
        c->armed = true;
        c->reason = "FALL_WAIT";
        return true;
    }
    c->armed = Motor_Enable(c->motor);
    c->reason = c->armed ? "BALANCE" : "PWM_ERROR";
    return c->armed;
}

void BalanceController_SetPositionLoop(BalanceController_t *c, bool enable) {
    reset_position(c);
    c->encoder_enabled = enable && c->motor->enc_ready;
}

/* Vòng ngoài giữ vị trí khối tâm (x_ref = 0, chạy ~50 Hz với tổng dt thực tế dtp) */
static void position_hold_loop(BalanceController_t *c, float dtp) {
    float counts = 0.5f * ((float)c->motor->enc_left_total + (float)c->motor->enc_right_total);
    float cpr = (c->enc_cpr > 1.0f) ? c->enc_cpr : DEFAULT_ENC_CPR;
    float x_wheel = counts / cpr * TWO_PI * WHEEL_RADIUS_M;
    float raw_v_wheel = (x_wheel - c->x_prev) / dtp;
    c->x_prev = x_wheel;
    c->x_wheel = x_wheel;

    /* Bù vị trí & vận tốc khối tâm:
     *   x_com = x_wheel + l_com * sin(theta)
     *   v_com = v_wheel + l_com * theta_dot * cos(theta) */
    float tilt_deg = -(float)c->control_sign * (c->imu->pitch_kalman - c->nominal_balance_angle);
    float tilt_rad = tilt_deg * DEG_TO_RAD;
    float avg_tilt_rate_dps = c->pos_rate_accum / dtp;
    c->pos_rate_accum = 0.0f;
    float tilt_rate_rad = avg_tilt_rate_dps * DEG_TO_RAD;

    c->x = x_wheel + COM_HEIGHT_M * sinf(tilt_rad);
    float raw_v_com = raw_v_wheel + COM_HEIGHT_M * tilt_rate_rad * cosf(tilt_rad);

    float alpha = dtp / (dtp + 1.0f / VEL_FILTER_N);
    c->v_wheel += alpha * (raw_v_wheel - c->v_wheel);
    c->v += alpha * (raw_v_com - c->v);

    /* Bù gia tốc ngang bánh xe (đã lọc thông thấp) cho Kalman IMU */
    float raw_a_wheel = (c->v_wheel - c->v_wheel_prev) / dtp;
    c->v_wheel_prev = c->v_wheel;
    c->a_wheel_filt += 0.25f * (raw_a_wheel - c->a_wheel_filt);
    float a_imu_g = -(float)c->control_sign * (c->a_wheel_filt / GRAVITY_G);
    ICM20602_SetLinearAccel(c->imu, a_imu_g);

    /* Nếu bị nhấc/đẩy xa > POS_RUNAWAY_M -> đặt lại gốc x = 0 nhưng GIỮ NGUYÊN integral */
    if (fabsf(c->x) > POS_RUNAWAY_M) {
        rebase_position_origin(c);
    }

    PID_SetTarget(&c->pid_pos, 0.0f);
    float th = PID_ComputeWithDerivative(&c->pid_pos, c->x, -c->v, dtp);
    c->theta_ref = clampf(th, -MAX_TARGET_ANGLE_DEG, MAX_TARGET_ANGLE_DEG);
    if (fabsf(th) >= MAX_TARGET_ANGLE_DEG) {
        c->sat_flags |= CTRL_SAT_THETA_REF;
    }
}

bool BalanceController_Update(BalanceController_t *c, float dt) {
    if (!c->armed) return false;
    uint32_t now_ms = HAL_GetTick();
    if (!c->imu->ready || c->imu->sample_age_ms > IMU_MAX_AGE_MS ||
        (now_ms - c->imu->last_sample_ms) > IMU_MAX_AGE_MS) {
        BalanceController_EmergencyStop(c, "IMU_FAULT");
        return false;
    }
    float angle = c->imu->pitch_kalman;
    float rate  = c->imu->gyro_rate_pitch;
    if (!isfinite(angle) || !isfinite(rate)) {
        BalanceController_EmergencyStop(c, "CONTROL_DATA");
        return false;
    }

    c->sat_flags = c->imu->last_read_fresh ? 0U : CTRL_SAT_IMU_PRED;

    /* Quy ước chiều dương: theta > 0 = nghiêng về phía PWM dương */
    float tilt      = -(float)c->control_sign * (angle - c->nominal_balance_angle);
    float tilt_rate = -(float)c->control_sign * rate;
    float tilt_err  = fabsf(tilt);

    /* === 1. BẢO VỆ NGÃ (> 35 độ) -> kéo STBY = LOW; dựng thẳng (<= 15 độ) -> bật lại === */
    if (!c->fall_waiting && tilt_err > FALL_ANGLE_DEG) {
        c->fall_waiting = true;
        Motor_Stop(c->motor);
        PID_Reset(&c->pid_angle);
        reset_position(c);
    }
    if (c->fall_waiting) {
        if (tilt_err <= FALL_RESUME_DEG &&
            fabsf(rate) <= ARM_MAX_RATE_DPS &&
            c->imu->last_read_fresh) {
            c->fall_waiting = false;
            PID_Reset(&c->pid_angle);
            reset_position(c);
            if (!Motor_Enable(c->motor)) {
                BalanceController_EmergencyStop(c, "PWM_ERROR");
                return false;
            }
        } else {
            if (c->motor->enabled || !c->motor->cut) {
                Motor_Stop(c->motor);
            }
            ICM20602_SetLinearAccel(c->imu, 0.0f);
            PID_Reset(&c->pid_angle);
            c->output = 0.0f;
            c->turn_output = 0.0f;
            c->reason = "FALL_WAIT";
            return true;
        }
    }

    /* === 2. VÒNG NGOÀI: GIỮ ĐỨNG YÊN TẠI CHỖ (x_ref = 0, khi ENC = 1) === */
    if (c->encoder_enabled) {
        c->pos_dt_accum += dt;
        c->pos_rate_accum += tilt_rate * dt;
        if (++c->pos_counter >= POS_LOOP_DIVIDER) {
            float dtp = c->pos_dt_accum;
            c->pos_counter = 0;
            c->pos_dt_accum = 0.0f;
            if (dtp >= 0.004f) {
                position_hold_loop(c, dtp);
            } else {
                c->pos_rate_accum = 0.0f;
            }
        }
    } else {
        c->theta_ref = 0.0f;
        c->pos_rate_accum = 0.0f;
        ICM20602_SetLinearAccel(c->imu, 0.0f);
    }

    /* === 3. VÒNG TRONG: CÂN BẰNG GÓC (200 Hz / 5 ms) === */
    c->theta_ref_filt += THETA_REF_LPF_ALPHA * (c->theta_ref - c->theta_ref_filt);
    c->reason = "BALANCE";
    int effective = c->motor->limit - c->motor->deadband;
    if (effective < 1) effective = 1;
    c->pid_angle.out_min = (float)-effective;
    c->pid_angle.out_max = (float)effective;
    if (c->pid_angle.Ki <= 1e-6f) {
        c->pid_angle.integral = 0.0f;
    }
    PID_SetTarget(&c->pid_angle, 0.0f);
    c->output = PID_ComputeWithDerivative(&c->pid_angle, c->theta_ref_filt - tilt, tilt_rate, dt);

    if (fabsf(c->output) >= (float)(effective - 1)) {
        c->sat_flags |= CTRL_SAT_ANGLE_OUT;
    }
    if (c->pid_angle.Ki > 1e-6f && fabsf(c->pid_angle.integral) >= (ANGLE_INTEGRAL_LIMIT - 0.1f)) {
        c->sat_flags |= CTRL_SAT_ANGLE_INT;
    }

    /* === 4. KHÓA GÓC HƯỚNG YAW TẠI CHỖ (PD: hiệu xung 2 bánh + imu.gz) === */
    float headroom = (float)effective - fabsf(c->output);
    if (headroom < 0.0f) headroom = 0.0f;

    float turn_req = 0.0f;
    float turn_eff = 0.0f;
    if (tilt_err <= YAW_HOLD_MAX_TILT_DEG && (c->yaw_kp > 0.0f || c->yaw_kd > 0.0f)) {
        float yaw_err_pulses = c->encoder_enabled
            ? (float)(c->motor->enc_left_total - c->motor->enc_right_total)
            : 0.0f;
        float yaw_rate_dps = isfinite(c->imu->gyro_rate_yaw)
            ? ((float)c->yaw_sign * c->imu->gyro_rate_yaw)
            : 0.0f;
        turn_req = clampf(-c->yaw_kp * yaw_err_pulses - c->yaw_kd * yaw_rate_dps,
                          -YAW_HOLD_MAX_PWM, YAW_HOLD_MAX_PWM);
        turn_eff = clampf(turn_req, -headroom, headroom);
        if (fabsf(turn_req) > 1.0f && fabsf(turn_eff - turn_req) > 1.0f) {
            c->sat_flags |= CTRL_SAT_TURN_CLIP;
        }
    }
    c->turn_output = turn_eff;

    int16_t pwm_left  = (int16_t)lroundf(c->output + turn_eff);
    int16_t pwm_right = (int16_t)lroundf(c->output - turn_eff);
    if (!Motor_SetSpeed(c->motor, pwm_left, pwm_right)) {
        BalanceController_EmergencyStop(c, "MOTOR_CUT");
        return false;
    }
    return true;
}
