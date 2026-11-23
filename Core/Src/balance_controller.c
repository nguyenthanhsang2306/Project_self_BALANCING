#include "balance_controller.h"
#include "board_config.h"
#include <math.h>
#include <string.h>

#define TWO_PI 6.28318531f

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Đặt lại gốc vị trí (x = 0, x_ref = 0 tại vị trí hiện tại của xe) */
static void reset_position(BalanceController_t *c) {
    Motor_ResetEncoders(c->motor);
    c->x = c->x_prev = 0.0f;
    c->v = 0.0f;
    c->x_ref = 0.0f;
    c->v_ref = 0.0f;
    c->theta_ref = 0.0f;
    c->theta_ref_filt = 0.0f;
    c->vel_integral = 0.0f;
    c->was_moving = false;
    c->brake_ticks = 0U;
    c->pos_counter = 0;
    PID_Reset(&c->pid_pos);
    PID_SetTarget(&c->pid_pos, 0.0f);
}

/* Xóa mọi lệnh di chuyển (dùng khi ngã / ARM / dừng khẩn) */
static void clear_motion(BalanceController_t *c) {
    c->v_cmd = 0.0f;
    c->v_ref = 0.0f;
    c->vel_integral = 0.0f;
    c->was_moving = false;
    c->brake_ticks = 0U;
    c->cmd_turn_pwm = 0.0f;
    c->filtered_turn_pwm = 0.0f;
    c->last_speed_cmd_ms = 0U;
    c->last_turn_cmd_ms = 0U;
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
    c->speed_target_mag = MOTION_SPEED_MED;
    c->last_cmd = 'X';

    /* Vòng trong (200 Hz): điều khiển góc nghiêng */
    PID_Init(&c->pid_angle, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
             -(float)motor->limit, (float)motor->limit, ANGLE_INTEGRAL_LIMIT);

    /* Vòng ngoài (50 Hz): bám x_ref, bão hòa ±8° */
    PID_Init(&c->pid_pos, DEFAULT_KP_POS, DEFAULT_KI_POS, DEFAULT_KD_POS,
             -MAX_TARGET_ANGLE_DEG, MAX_TARGET_ANGLE_DEG, POS_INTEGRAL_LIMIT);
}

void BalanceController_SetZeroAngle(BalanceController_t *c, float zero_angle) {
    c->nominal_balance_angle = zero_angle;
}

void BalanceController_ClearFault(BalanceController_t *c) {
    c->fault_latched = false;
}

void BalanceController_SetMotion(BalanceController_t *c, float speed_mps, float turn_pwm) {
    c->v_cmd = clampf(speed_mps, -MOTION_SPEED_MAX, MOTION_SPEED_MAX);
    c->cmd_turn_pwm = turn_pwm;
    uint32_t now = HAL_GetTick();
    if (fabsf(speed_mps) > 0.001f) c->last_speed_cmd_ms = now;
    if (fabsf(turn_pwm) > 1.0f) c->last_turn_cmd_ms = now;
}

void BalanceController_ProcessCommandChar(BalanceController_t *c, char cmd) {
    uint32_t now = HAL_GetTick();
    if (cmd == '1') {
        c->speed_target_mag = MOTION_SPEED_SLOW;
        if (c->v_cmd > 0.001f) c->v_cmd = c->speed_target_mag;
        else if (c->v_cmd < -0.001f) c->v_cmd = -c->speed_target_mag;
    } else if (cmd == '2') {
        c->speed_target_mag = MOTION_SPEED_MED;
        if (c->v_cmd > 0.001f) c->v_cmd = c->speed_target_mag;
        else if (c->v_cmd < -0.001f) c->v_cmd = -c->speed_target_mag;
    } else if (cmd == '3') {
        c->speed_target_mag = MOTION_SPEED_FAST;
        if (c->v_cmd > 0.001f) c->v_cmd = c->speed_target_mag;
        else if (c->v_cmd < -0.001f) c->v_cmd = -c->speed_target_mag;
    } else if (cmd == 'w' || cmd == 'W') {
        /* Đặt mục tiêu tiến theo mức tốc độ đã chọn (KHÔNG cộng dồn mỗi lần giữ) */
        c->v_cmd = c->speed_target_mag;
        c->last_speed_cmd_ms = now;
        c->last_any_cmd_ms = now;
        c->last_cmd = 'W';
    } else if (cmd == 's' || cmd == 'S') {
        /* Đặt mục tiêu lùi theo mức tốc độ đã chọn */
        c->v_cmd = -c->speed_target_mag;
        c->last_speed_cmd_ms = now;
        c->last_any_cmd_ms = now;
        c->last_cmd = 'S';
    } else if (cmd == 'k' || cmd == 'K') {
        /* Gói Keep-Alive Heartbeat: chỉ làm mới bộ đếm timeout, KHÔNG cộng thêm tốc độ */
        if (fabsf(c->v_cmd) > 0.001f) c->last_speed_cmd_ms = now;
        if (fabsf(c->cmd_turn_pwm) > 1.0f) c->last_turn_cmd_ms = now;
        if (c->last_cmd == 'W' || c->last_cmd == 'S' || c->last_cmd == 'A' || c->last_cmd == 'D') {
            c->last_any_cmd_ms = now;
        }
    } else if (cmd == 'v' || cmd == 'V') {
        /* Nhả nút Tiến/Lùi -> trả tốc độ đặt về 0 ngay để phanh mượt */
        c->v_cmd = 0.0f;
        c->last_speed_cmd_ms = 0U;
        if (fabsf(c->cmd_turn_pwm) < 1.0f) {
            c->last_cmd = 'X';
            c->last_any_cmd_ms = 0U;
        }
    } else if (cmd == 'a' || cmd == 'A') {
        c->cmd_turn_pwm = -MOTION_TURN_PWM;
        c->last_turn_cmd_ms = now;
        c->last_any_cmd_ms = now;
        c->last_cmd = 'A';
    } else if (cmd == 'd' || cmd == 'D') {
        c->cmd_turn_pwm = MOTION_TURN_PWM;
        c->last_turn_cmd_ms = now;
        c->last_any_cmd_ms = now;
        c->last_cmd = 'D';
    } else if (cmd == 'c' || cmd == 'C') {
        /* Nhả nút rẽ -> trả thẳng lái, giữ nguyên tốc độ tiến/lùi */
        c->cmd_turn_pwm = 0.0f;
        c->last_turn_cmd_ms = 0U;
        if (fabsf(c->v_cmd) < 0.001f) {
            c->last_cmd = 'X';
            c->last_any_cmd_ms = 0U;
        }
    } else if (cmd == 'x' || cmd == 'X' || cmd == ' ') {
        /* Dừng di chuyển mượt: v_cmd = 0 -> ramp về đứng yên cân bằng tại chỗ */
        c->v_cmd = 0.0f;
        c->cmd_turn_pwm = 0.0f;
        c->last_speed_cmd_ms = 0U;
        c->last_turn_cmd_ms = 0U;
        c->last_any_cmd_ms = 0U;
        c->last_cmd = 'X';
    }
}

void BalanceController_EmergencyStop(BalanceController_t *c, const char *reason) {
    Motor_Stop(c->motor);
    c->armed = false;
    c->fall_waiting = false;
    c->fault_latched = true;    /* Chốt an toàn: không tự ARM lại ở chu kỳ sau */
    c->output = 0.0f;
    c->reason = reason;
    clear_motion(c);
    PID_Reset(&c->pid_angle);
    reset_position(c);
}

bool BalanceController_Arm(BalanceController_t *c) {
    if (c->fault_latched) {
        return false;           /* Phải xóa chốt lỗi bằng B1 hoặc 'r'/'i' khi dừng */
    }
    if (!c->imu->ready || HAL_GetTick() - c->imu->last_sample_ms > 200U ||
        !isfinite(c->imu->pitch_kalman) || !isfinite(c->imu->gyro_rate_pitch) ||
        c->pid_angle.Kp <= 0 || !c->motor->ready) {
        return false;
    }
    PID_Reset(&c->pid_angle);
    reset_position(c);
    clear_motion(c);
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

/* Vòng ngoài 50 Hz:
 * - Chế độ DI CHUYỂN (W/S & phanh về 0): vòng PI vận tốc có tích phân chống bão hòa,
 *   KHÔNG reset mỗi chu kỳ, tự bù sai số góc zero và ma sát động cơ.
 * - Chế độ ĐỨNG YÊN: PID giữ vị trí tại x_ref (giữ nguyên khi quay tại chỗ A/D).
 * - Chuyển đổi 2 chiều Đứng yên <-> Di chuyển kiểu Bumpless (không nhảy góc theta_ref). */
static void position_and_motion_loop(BalanceController_t *c, float dtp) {
    /* 1. Ramp v_ref về v_cmd với gia tốc giới hạn (khởi động / phanh êm) */
    float accel = (fabsf(c->v_cmd) > 0.001f) ? MOTION_ACCEL : MOTION_DECEL;
    float dv_max = accel * dtp;
    c->v_ref += clampf(c->v_cmd - c->v_ref, -dv_max, dv_max);
    if (fabsf(c->v_cmd) <= 0.001f && fabsf(c->v_ref) < 0.005f) {
        c->v_ref = 0.0f;
    }

    /* 2. Xác nhận xung Encoder thực tế (>= 24 xung tránh nhiễu gai; KHÔNG tự tắt khi bánh kẹt) */
    float l_tot = (float)c->motor->enc_left_total;
    float r_tot = (float)c->motor->enc_right_total;
    if (!c->enc_signal_active && (fabsf(l_tot) >= 24.0f || fabsf(r_tot) >= 24.0f)) {
        c->enc_signal_active = true;
    }

    if (!c->enc_signal_active) {
        c->x = c->x_prev = 0.0f;
        c->v = 0.0f;
        c->x_ref = 0.0f;
        c->vel_integral = 0.0f;
        c->was_moving = false;
        c->brake_ticks = 0U;
        c->theta_ref = clampf((c->v_ref / MOTION_SPEED_MAX) * MOTION_NOENC_TILT_DEG,
                              -MOTION_TILT_MAX_DEG, MOTION_TILT_MAX_DEG);
        return;
    }

    float counts = 0.5f * (l_tot + r_tot);
    if (fabsf(l_tot) < 5.0f && fabsf(r_tot) > 30.0f) counts = r_tot;
    else if (fabsf(r_tot) < 5.0f && fabsf(l_tot) > 30.0f) counts = l_tot;
    float cpr = (c->enc_cpr > 1.0f) ? c->enc_cpr : DEFAULT_ENC_CPR;
    c->x = counts / cpr * TWO_PI * WHEEL_RADIUS_M;

    /* 3. Vận tốc qua bộ lọc đạo hàm bậc 1 (N = 10) */
    float raw_v = (c->x - c->x_prev) / dtp;
    c->x_prev = c->x;
    float alpha = dtp / (dtp + 1.0f / VEL_FILTER_N);
    c->v += alpha * (raw_v - c->v);

    /* 4. Xác định chế độ DI CHUYỂN (gồm cả giai đoạn phanh về vận tốc gần 0) hay ĐỨNG YÊN */
    bool cmd_active = (fabsf(c->v_cmd) > 0.001f) || (fabsf(c->v_ref) > 0.005f);
    bool braking = false;
    if (cmd_active) {
        c->brake_ticks = 0U;
    } else if (c->was_moving && fabsf(c->v) > MOTION_BRAKE_VEL_MPS && c->brake_ticks < MOTION_BRAKE_MAX_TICKS) {
        c->brake_ticks++;
        braking = true;
    } else {
        c->brake_ticks = 0U;
    }
    bool moving = cmd_active || braking;

    if (moving) {
        float vel_err = c->v_ref - c->v;
        float p_term = MOTION_KP_VEL * vel_err;

        if (!c->was_moving) {
            /* Bumpless: khởi tạo tích phân vận tốc từ theta_ref hiện tại (giữ nguyên bù góc zero) */
            c->vel_integral = clampf(c->theta_ref - p_term,
                                     -MOTION_VEL_INT_LIMIT, MOTION_VEL_INT_LIMIT);
        } else {
            /* Tích phân có anti-windup: chỉ dừng tích phân nếu đang bão hòa cùng chiều sai số */
            float next_i = clampf(c->vel_integral + MOTION_KI_VEL * vel_err * dtp,
                                  -MOTION_VEL_INT_LIMIT, MOTION_VEL_INT_LIMIT);
            float cand = p_term + next_i;
            if (!((cand > MOTION_TILT_MAX_DEG && vel_err > 0.0f) ||
                  (cand < -MOTION_TILT_MAX_DEG && vel_err < 0.0f))) {
                c->vel_integral = next_i;
            }
        }

        c->x_ref = c->x;
        c->theta_ref = clampf(p_term + c->vel_integral, -MOTION_TILT_MAX_DEG, MOTION_TILT_MAX_DEG);
        c->was_moving = true;
    } else {
        if (c->was_moving) {
            /* Bumpless: chốt vị trí mới và nạp tích phân pid_pos khớp đúng theta_ref hiện tại */
            c->x_ref = c->x;
            c->pid_pos.prev_error = 0.0f;
            if (c->pid_pos.Ki > 1e-5f) {
                float desired_i = c->theta_ref - c->pid_pos.Kd * (-c->v);
                c->pid_pos.integral = clampf(desired_i / c->pid_pos.Ki,
                                             -c->pid_pos.integral_limit, c->pid_pos.integral_limit);
            } else {
                c->pid_pos.integral = 0.0f;
            }
            c->was_moving = false;
        }

        /* Bị nhấc/đẩy đi quá xa khi đang đứng yên -> lấy chỗ mới làm gốc */
        if (fabsf(c->x - c->x_ref) > POS_RUNAWAY_M) {
            c->x_ref = c->x;
            PID_Reset(&c->pid_pos);
        } else {
            c->x_ref = clampf(c->x_ref, c->x - MOTION_TRACK_ERR_MAX, c->x + MOTION_TRACK_ERR_MAX);
        }

        /* PID giữ vị trí đứng yên tại x_ref (giữ nguyên kể cả khi quay tại chỗ A/D) */
        PID_SetTarget(&c->pid_pos, c->x_ref);
        float th = PID_ComputeWithDerivative(&c->pid_pos, c->x, -c->v, dtp);
        c->theta_ref = clampf(th, -MAX_TARGET_ANGLE_DEG, MAX_TARGET_ANGLE_DEG);
    }
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

    /* Quy ước: θ > 0 = nghiêng về phía PWM dương (tiến) */
    float tilt      = -(float)c->control_sign * (angle - c->nominal_balance_angle);
    float tilt_rate = -(float)c->control_sign * rate;
    float tilt_err  = fabsf(tilt);

    /* === 1. NGÃ (> 35°) -> ngắt motor tạm; DỰNG THẲNG (<= 18°) -> tự cân bằng tiếp === */
    if (!c->fall_waiting && tilt_err > FALL_ANGLE_DEG) {
        c->fall_waiting = true;
        clear_motion(c);
    }
    if (c->fall_waiting) {
        if (tilt_err <= FALL_RESUME_DEG && fabsf(rate) <= ARM_MAX_RATE_DPS) {
            c->fall_waiting = false;
            clear_motion(c);
            PID_Reset(&c->pid_angle);
            reset_position(c);
        } else {
            /* Xe nằm trên bàn mà bấm W/A/S/D -> quay thử bánh 0.45 s để test chiều quay */
            if (c->last_any_cmd_ms != 0U && (HAL_GetTick() - c->last_any_cmd_ms < 450U)) {
                int16_t tl = 0, tr = 0;
                if (c->last_cmd == 'W')      { tl = 320;  tr = 320;  }
                else if (c->last_cmd == 'S') { tl = -320; tr = -320; }
                else if (c->last_cmd == 'A') { tl = -280; tr = 280;  }
                else if (c->last_cmd == 'D') { tl = 280;  tr = -280; }
                Motor_SetSpeed(c->motor, tl, tr);
            } else {
                Motor_SetSpeed(c->motor, 0, 0);
                c->v_cmd = 0.0f;
            }
            PID_Reset(&c->pid_angle);
            c->output = 0.0f;
            c->reason = "FALL_WAIT";
            return true;
        }
    }

    /* Tự trả tốc độ tiến/lùi về 0 và trả thẳng lái nếu nhả nút hoặc mất kết nối quá timeout */
    uint32_t now_ms = HAL_GetTick();
    if (fabsf(c->v_cmd) > 0.001f && (now_ms - c->last_speed_cmd_ms > MOTION_CMD_TIMEOUT_MS)) {
        c->v_cmd = 0.0f;
        if (fabsf(c->cmd_turn_pwm) < 1.0f) c->last_cmd = 'X';
    }
    if (fabsf(c->cmd_turn_pwm) > 1.0f && (now_ms - c->last_turn_cmd_ms > MOTION_TURN_TIMEOUT_MS)) {
        c->cmd_turn_pwm = 0.0f;
        if (fabsf(c->v_cmd) < 0.001f) c->last_cmd = 'X';
    }
    c->filtered_turn_pwm += MOTION_TURN_LPF_ALPHA * (c->cmd_turn_pwm - c->filtered_turn_pwm);

    /* === 2. VÒNG NGOÀI: VỊ TRÍ & DI CHUYỂN (50 Hz / 20 ms) ===
     * Ghi chú an toàn: KHÔNG tự lật dấu Encoder khi đang chạy */
    if (c->encoder_enabled) {
        if (++c->pos_counter >= POS_LOOP_DIVIDER) {
            c->pos_counter = 0;
            position_and_motion_loop(c, dt * (float)POS_LOOP_DIVIDER);
        }
    } else {
        float accel = (fabsf(c->v_cmd) > 0.001f) ? MOTION_ACCEL : MOTION_DECEL;
        float dv_max = accel * dt;
        c->v_ref += clampf(c->v_cmd - c->v_ref, -dv_max, dv_max);
        c->theta_ref = clampf((c->v_ref / MOTION_SPEED_MAX) * MOTION_NOENC_TILT_DEG,
                              -MOTION_TILT_MAX_DEG, MOTION_TILT_MAX_DEG);
    }

    /* === 3. VÒNG TRONG: CÂN BẰNG GÓC (200 Hz / 5 ms) === */
    c->theta_ref_filt += MOTION_THETA_LPF_ALPHA * (c->theta_ref - c->theta_ref_filt);
    c->reason = "BALANCE";
    int effective = c->motor->limit - c->motor->deadband;
    if (effective < 1) effective = 1;
    c->pid_angle.out_min = (float)-effective;
    c->pid_angle.out_max = (float)effective;
    PID_SetTarget(&c->pid_angle, 0.0f);
    c->output = PID_ComputeWithDerivative(&c->pid_angle, c->theta_ref_filt - tilt, tilt_rate, dt);

    /* === 4. RẼ VI SAI ƯU TIÊN CÂN BẰNG (Balance-Priority Differential Mixer) ===
     * Không bao giờ để lực rẽ lấn áp mô-men giữ thăng bằng hoặc gây bão hòa lệch 1 bánh */
    float turn_eff = (tilt_err > MOTION_TURN_MAX_TILT_DEG) ? 0.0f : c->filtered_turn_pwm;
    float headroom = (float)effective - fabsf(c->output);
    if (headroom < 0.0f) headroom = 0.0f;
    turn_eff = clampf(turn_eff, -headroom, headroom);

    int16_t pwm_left  = (int16_t)lroundf(c->output + turn_eff);
    int16_t pwm_right = (int16_t)lroundf(c->output - turn_eff);
    if (!Motor_SetSpeed(c->motor, pwm_left, pwm_right)) {
        BalanceController_EmergencyStop(c, "MOTOR_CUT");
        return false;
    }
    return true;
}
