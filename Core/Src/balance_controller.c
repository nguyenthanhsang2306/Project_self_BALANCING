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

static int32_t abs_i32(int32_t v) {
    return (v < 0) ? -v : v;
}

/* Đặt lại gốc vị trí khối tâm, khóa hướng yaw và bộ giám sát encoder */
static void reset_position(BalanceController_t *c) {
    Motor_ResetEncoders(c->motor);
    c->x = c->x_wheel = c->x_prev = 0.0f;
    c->v = c->v_wheel = c->v_wheel_prev = 0.0f;
    c->a_wheel_filt = 0.0f;
    c->x_ref = 0.0f;
    c->v_ref = 0.0f;
    c->theta_ref = 0.0f;
    c->theta_ref_filt = 0.0f;
    c->vel_integral = 0.0f;
    c->pos_dt_accum = 0.0f;
    c->pos_rate_accum = 0.0f;
    c->yaw_rate_ref = 0.0f;
    c->yaw_heading_err = 0.0f;
    c->was_moving = false;
    c->brake_ticks = 0U;
    c->pos_counter = 0;
    c->enc_left_active = false;
    c->enc_right_active = false;
    /* Cho phép vòng ngoài hoạt động ngay lập tức khi phần cứng encoder sẵn sàng
     * (bỏ chốt chặn mù 24 xung lúc mới ARM làm lỡ lệnh W/S đầu tiên). */
    c->enc_signal_active = (c->encoder_enabled && c->motor != NULL && c->motor->enc_ready);
    c->enc_fault_left = false;
    c->enc_fault_right = false;
    c->enc_no_signal = false;
    c->motor_stalled = false;
    c->enc_loss_l_ticks = 0U;
    c->enc_loss_r_ticks = 0U;
    c->motor_stall_ticks = 0U;
    c->enc_prev_l_total = 0;
    c->enc_prev_r_total = 0;
    if (c->imu != NULL) {
        ICM20602_SetLinearAccel(c->imu, 0.0f);
    }
    PID_Reset(&c->pid_pos);
    PID_SetTarget(&c->pid_pos, 0.0f);
}

/* Xóa triệt để mọi lệnh di chuyển và trạng thái lệnh gần nhất (dùng khi ngã / ARM / dừng khẩn) */
static void clear_motion(BalanceController_t *c) {
    c->v_cmd = 0.0f;
    c->v_ref = 0.0f;
    c->vel_integral = 0.0f;
    c->was_moving = false;
    c->brake_ticks = 0U;
    c->cmd_turn_pwm = 0.0f;
    c->filtered_turn_pwm = 0.0f;
    c->yaw_rate_ref = 0.0f;
    c->yaw_heading_err = 0.0f;
    c->last_speed_cmd_ms = 0U;
    c->last_turn_cmd_ms = 0U;
    c->last_pwm_left = 0;
    c->last_pwm_right = 0;
    c->last_cmd = 'X';
}

void BalanceController_Init(BalanceController_t *c, ICM20602_t *imu, Motor_System_t *motor) {
    memset(c, 0, sizeof(*c));
    c->imu = imu;
    c->motor = motor;
    c->nominal_balance_angle = DEFAULT_ZERO_DEG;
    c->control_sign = DEFAULT_CONTROL_SIGN;
    c->reason = "DISARMED";
    c->armed = false;
    c->fault_latched = false;
    c->encoder_enabled = (DEFAULT_ENC_ENABLE != 0 && motor != NULL && motor->enc_ready);
    c->enc_signal_active = c->encoder_enabled;
    c->enc_cpr = DEFAULT_ENC_CPR;
    c->speed_target_mag = MOTION_SPEED_MED;
    c->last_cmd = 'X';

    /* Vòng trong (200 Hz): PD góc thuần (DEFAULT_KI = 0.0f để tránh tích phân kép gây gằn bánh) */
    PID_Init(&c->pid_angle, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD,
             -(float)motor->limit, (float)motor->limit, ANGLE_INTEGRAL_LIMIT);

    /* Vòng ngoài (50 Hz): bám x_ref của khối tâm, bão hòa ±8° */
    PID_Init(&c->pid_pos, DEFAULT_KP_POS, DEFAULT_KI_POS, DEFAULT_KD_POS,
             -MAX_TARGET_ANGLE_DEG, MAX_TARGET_ANGLE_DEG, POS_INTEGRAL_LIMIT);
}

void BalanceController_SetZeroAngle(BalanceController_t *c, float zero_angle) {
    c->nominal_balance_angle = zero_angle;
}

void BalanceController_ClearFault(BalanceController_t *c) {
    c->fault_latched = false;
    c->enc_fault_left = false;
    c->enc_fault_right = false;
    c->enc_no_signal = false;
    c->motor_stalled = false;
}

void BalanceController_SetMotion(BalanceController_t *c, float speed_mps, float turn_pwm) {
    if (!c->armed || c->fault_latched) {
        clear_motion(c);
        return;
    }
    c->v_cmd = clampf(speed_mps, -MOTION_SPEED_MAX, MOTION_SPEED_MAX);
    c->cmd_turn_pwm = clampf(turn_pwm, -MOTION_TURN_PWM, MOTION_TURN_PWM);
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
        return;
    } else if (cmd == '2') {
        c->speed_target_mag = MOTION_SPEED_MED;
        if (c->v_cmd > 0.001f) c->v_cmd = c->speed_target_mag;
        else if (c->v_cmd < -0.001f) c->v_cmd = -c->speed_target_mag;
        return;
    } else if (cmd == '3') {
        c->speed_target_mag = MOTION_SPEED_FAST;
        if (c->v_cmd > 0.001f) c->v_cmd = c->speed_target_mag;
        else if (c->v_cmd < -0.001f) c->v_cmd = -c->speed_target_mag;
        return;
    } else if (cmd == 'x' || cmd == 'X' || cmd == ' ') {
        clear_motion(c);
        return;
    }

    /* Tuyệt đối bỏ qua lệnh di chuyển khi chưa ARM hoặc đang chốt lỗi an toàn */
    if (!c->armed || c->fault_latched) {
        return;
    }

    if (cmd == 'w' || cmd == 'W') {
        c->v_cmd = c->speed_target_mag;
        c->last_speed_cmd_ms = now;
        c->last_cmd = 'W';
    } else if (cmd == 's' || cmd == 'S') {
        c->v_cmd = -c->speed_target_mag;
        c->last_speed_cmd_ms = now;
        c->last_cmd = 'S';
    } else if (cmd == 'k' || cmd == 'K') {
        if (fabsf(c->v_cmd) > 0.001f) c->last_speed_cmd_ms = now;
        if (fabsf(c->cmd_turn_pwm) > 1.0f) c->last_turn_cmd_ms = now;
    } else if (cmd == 'v' || cmd == 'V') {
        c->v_cmd = 0.0f;
        c->last_speed_cmd_ms = 0U;
        if (fabsf(c->cmd_turn_pwm) < 1.0f) c->last_cmd = 'X';
    } else if (cmd == 'a' || cmd == 'A') {
        c->cmd_turn_pwm = -MOTION_TURN_PWM;
        c->last_turn_cmd_ms = now;
        c->last_cmd = 'A';
    } else if (cmd == 'd' || cmd == 'D') {
        c->cmd_turn_pwm = MOTION_TURN_PWM;
        c->last_turn_cmd_ms = now;
        c->last_cmd = 'D';
    } else if (cmd == 'c' || cmd == 'C') {
        c->cmd_turn_pwm = 0.0f;
        c->last_turn_cmd_ms = 0U;
        if (fabsf(c->v_cmd) < 0.001f) c->last_cmd = 'X';
    }
}

void BalanceController_EmergencyStop(BalanceController_t *c, const char *reason) {
    bool fl = c->enc_fault_left;
    bool fr = c->enc_fault_right;
    bool ns = c->enc_no_signal;
    bool ms = c->motor_stalled;
    Motor_Stop(c->motor);
    c->armed = false;
    c->fault_latched = true;    /* Chốt an toàn: yêu cầu lệnh ARM tường minh & đủ điều kiện mới mở lại */
    c->output = 0.0f;
    c->sat_flags = 0U;
    c->reason = reason;
    clear_motion(c);
    PID_Reset(&c->pid_angle);
    reset_position(c);
    /* Giữ nguyên cờ chẩn đoán lỗi encoder/kẹt bánh cho Telemetry STAT hiển thị đúng LOSS_L/R, NO_SIG, STALL */
    c->enc_fault_left = fl;
    c->enc_fault_right = fr;
    c->enc_no_signal = ns;
    c->motor_stalled = ms;
    if (fl || fr || ns) {
        c->enc_signal_active = false;
    }
}

/* Chỉ cho phép ARM khi có yêu cầu tường minh VÀ thỏa đầy đủ điều kiện cảm biến, encoder và góc dựng thẳng */
bool BalanceController_Arm(BalanceController_t *c) {
    if (c->fault_latched) {
        Motor_Stop(c->motor);
        return false;
    }
    uint32_t now = HAL_GetTick();
    if (!c->imu->ready || !c->imu->last_read_fresh ||
        c->imu->sample_age_ms > IMU_MAX_AGE_MS ||
        (now - c->imu->last_sample_ms) > IMU_MAX_AGE_MS ||
        !isfinite(c->imu->pitch_kalman) || !isfinite(c->imu->gyro_rate_pitch)) {
        Motor_Stop(c->motor);
        c->reason = "ARM_IMU";
        return false;
    }
    if (c->pid_angle.Kp <= 0.0f || !c->motor->ready) {
        Motor_Stop(c->motor);
        c->reason = "ARM_MOTOR";
        return false;
    }
    if (DEFAULT_ENC_ENABLE != 0 && !c->motor->enc_ready) {
        Motor_Stop(c->motor);
        c->reason = "ARM_ENC";
        return false;
    }

    float tilt_err = fabsf(c->imu->pitch_kalman - c->nominal_balance_angle);
    if (tilt_err > ARM_MAX_ANGLE_DEG) {
        Motor_Stop(c->motor);
        c->reason = "ARM_TILT";
        return false;
    }
    if (fabsf(c->imu->gyro_rate_pitch) > ARM_MAX_RATE_DPS) {
        Motor_Stop(c->motor);
        c->reason = "ARM_RATE";
        return false;
    }

    c->encoder_enabled = (DEFAULT_ENC_ENABLE != 0 && c->motor->enc_ready);
    PID_Reset(&c->pid_angle);
    reset_position(c);
    clear_motion(c);
    c->sat_flags = 0U;
    c->armed = Motor_Enable(c->motor);
    c->reason = c->armed ? "BALANCE" : "PWM_ERROR";
    return c->armed;
}

void BalanceController_SetPositionLoop(BalanceController_t *c, bool enable) {
    c->encoder_enabled = enable && c->motor->enc_ready;
    reset_position(c);
}

/* Vòng ngoài (~50 Hz, dùng tổng dt thực tế dtp):
 * - Hoạt động ngay từ chu kỳ đầu tiên sau khi ARM (không khóa mù 24 xung, giữ W/S ăn ngay tức thì).
 * - Vẫn giám sát độc lập lỗi mất xung 1 bánh (ENC_LOSS_L/R) và kẹt/đứt dây cả 2 bánh (ENC_NO_SIG / MOTOR_STALL).
 * - Bù vị trí & vận tốc khối tâm (x_com, v_com) khử hiện tượng pha không cực tiểu khi bắt đầu tiến/lùi.
 * - Gửi gia tốc ngang ước lượng của bánh sang bộ lọc Kalman để chống nghiêng giả khi tăng tốc / phanh.
 * - Khi phanh dừng hẳn: chốt x_ref = x_com và RESET sạch pid_pos.integral (không nạp góc phanh gây trôi ngược). */
static bool position_and_motion_loop(BalanceController_t *c, float dtp) {
    int32_t l_tot_i = c->motor->enc_left_total;
    int32_t r_tot_i = c->motor->enc_right_total;
    int32_t dl = abs_i32(l_tot_i - c->enc_prev_l_total);
    int32_t dr = abs_i32(r_tot_i - c->enc_prev_r_total);
    c->enc_prev_l_total = l_tot_i;
    c->enc_prev_r_total = r_tot_i;

    /* 1. Ghi nhận bánh xe đã từng phát xung (chỉ dùng phân loại NO_SIG vs STALL, KHÔNG chặn điều khiển) */
    if (!c->enc_left_active && (dl >= 2 || fabsf((float)l_tot_i) >= ENC_ACTIVE_MIN_PULSES)) {
        c->enc_left_active = true;
    }
    if (!c->enc_right_active && (dr >= 2 || fabsf((float)r_tot_i) >= ENC_ACTIVE_MIN_PULSES)) {
        c->enc_right_active = true;
    }

    /* 2. Giám sát lỗi mất xung bất đối xứng 1 kênh và kẹt/đứt dây đối xứng cả 2 kênh */
    int32_t abs_pwm_l = abs_i32((int32_t)c->last_pwm_left);
    int32_t abs_pwm_r = abs_i32((int32_t)c->last_pwm_right);

    if (abs_pwm_l >= ENC_LOSS_PWM_THRESH && dl < 2 && dr >= ENC_LOSS_OTHER_MIN_D) {
        if (++c->enc_loss_l_ticks >= ENC_LOSS_TICKS) {
            c->enc_fault_left = true;
        }
    } else {
        c->enc_loss_l_ticks = 0U;
    }

    if (abs_pwm_r >= ENC_LOSS_PWM_THRESH && dr < 2 && dl >= ENC_LOSS_OTHER_MIN_D) {
        if (++c->enc_loss_r_ticks >= ENC_LOSS_TICKS) {
            c->enc_fault_right = true;
        }
    } else {
        c->enc_loss_r_ticks = 0U;
    }

    if (abs_pwm_l >= MOTOR_STALL_PWM_THRESH && abs_pwm_r >= MOTOR_STALL_PWM_THRESH && dl < 2 && dr < 2) {
        if (++c->motor_stall_ticks >= MOTOR_STALL_TICKS) {
            if (!c->enc_left_active && !c->enc_right_active) {
                c->enc_no_signal = true;
            } else {
                c->motor_stalled = true;
            }
        }
    } else {
        c->motor_stall_ticks = 0U;
    }

    if (c->enc_fault_left || c->enc_fault_right) {
        c->enc_signal_active = false;
        const char *reason = c->enc_fault_left ? "ENC_LOSS_L" : "ENC_LOSS_R";
        BalanceController_EmergencyStop(c, reason);
        return false;
    }
    if (c->enc_no_signal) {
        c->enc_signal_active = false;
        BalanceController_EmergencyStop(c, "ENC_NO_SIG");
        return false;
    }
    if (c->motor_stalled) {
        BalanceController_EmergencyStop(c, "MOTOR_STALL");
        return false;
    }
    c->enc_signal_active = true;

    /* 3. Ramp v_ref về v_cmd với gia tốc giới hạn (khởi động / phanh êm) */
    float accel = (fabsf(c->v_cmd) > 0.001f) ? MOTION_ACCEL : MOTION_DECEL;
    float dv_max = accel * dtp;
    c->v_ref += clampf(c->v_cmd - c->v_ref, -dv_max, dv_max);
    if (fabsf(c->v_cmd) <= 0.001f && fabsf(c->v_ref) < 0.005f) {
        c->v_ref = 0.0f;
    }

    /* 4. Bù vị trí & vận tốc khối tâm (x_com, v_com) đồng bộ pha với vận tốc bánh xe */
    float counts = 0.5f * ((float)l_tot_i + (float)r_tot_i);
    float cpr = (c->enc_cpr > 1.0f) ? c->enc_cpr : DEFAULT_ENC_CPR;
    float x_wheel = counts / cpr * TWO_PI * WHEEL_RADIUS_M;
    float raw_v_wheel = (x_wheel - c->x_prev) / dtp;
    c->x_prev = x_wheel;
    c->x_wheel = x_wheel;

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

    /* 5. Ước lượng gia tốc tịnh tiến bánh xe (lọc thông thấp) cấp sang Kalman IMU */
    float raw_a_wheel = (c->v_wheel - c->v_wheel_prev) / dtp;
    c->v_wheel_prev = c->v_wheel;
    c->a_wheel_filt += 0.25f * (raw_a_wheel - c->a_wheel_filt);
    float a_imu_g = -(float)c->control_sign * (c->a_wheel_filt / GRAVITY_G);
    ICM20602_SetLinearAccel(c->imu, a_imu_g);

    /* 6. Xác định chế độ DI CHUYỂN (gồm giai đoạn phanh) hay ĐỨNG YÊN */
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
            /* Chuyển êm Đứng yên -> Di chuyển: khởi tạo tích phân vận tốc từ theta_ref hiện tại */
            c->vel_integral = clampf(c->theta_ref - p_term,
                                     -MOTION_VEL_INT_LIMIT, MOTION_VEL_INT_LIMIT);
        } else if (cmd_active) {
            float next_i = clampf(c->vel_integral + MOTION_KI_VEL * vel_err * dtp,
                                  -MOTION_VEL_INT_LIMIT, MOTION_VEL_INT_LIMIT);
            float cand = p_term + next_i;
            if (!((cand > MOTION_TILT_MAX_DEG && vel_err > 0.0f) ||
                  (cand < -MOTION_TILT_MAX_DEG && vel_err < 0.0f))) {
                c->vel_integral = next_i;
            }
        } else {
            /* Đang phanh cuối (v_ref == 0): xả mượt vel_integral về 0 thay vì tích lũy ngược chiều */
            float bleed = clampf(4.0f * dtp, 0.0f, 0.30f);
            c->vel_integral *= (1.0f - bleed);
        }

        c->x_ref = c->x;
        float raw_theta = p_term + c->vel_integral;
        c->theta_ref = clampf(raw_theta, -MOTION_TILT_MAX_DEG, MOTION_TILT_MAX_DEG);
        if (fabsf(raw_theta) >= MOTION_TILT_MAX_DEG) {
            c->sat_flags |= CTRL_SAT_THETA_REF;
        }
        c->was_moving = true;
    } else {
        if (c->was_moving) {
            /* Sửa dứt điểm lỗi xe trôi ngược sau khi dừng:
             * Chốt vị trí đứng mới tại x_com hiện tại và RESET pid_pos (integral = 0),
             * tuyệt đối KHÔNG nạp góc nghiêng phanh động (theta_ref âm) vào pid_pos.integral. */
            c->x_ref = c->x;
            c->vel_integral = 0.0f;
            PID_Reset(&c->pid_pos);
            c->was_moving = false;
        }

        if (fabsf(c->x - c->x_ref) > POS_RUNAWAY_M) {
            c->x_ref = c->x;
            c->pid_pos.prev_error = 0.0f;
        } else {
            c->x_ref = clampf(c->x_ref, c->x - MOTION_TRACK_ERR_MAX, c->x + MOTION_TRACK_ERR_MAX);
        }

        /* PID giữ vị trí khối tâm tại x_ref (giữ nguyên kể cả khi quay tại chỗ A/D) */
        PID_SetTarget(&c->pid_pos, c->x_ref);
        float th = PID_ComputeWithDerivative(&c->pid_pos, c->x, -c->v, dtp);
        c->theta_ref = clampf(th, -MAX_TARGET_ANGLE_DEG, MAX_TARGET_ANGLE_DEG);
        if (fabsf(th) >= MAX_TARGET_ANGLE_DEG) {
            c->sat_flags |= CTRL_SAT_THETA_REF;
        }
    }
    return true;
}

bool BalanceController_Update(BalanceController_t *c, float dt) {
    if (!c->armed || c->fault_latched) return false;

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

    /* Quy ước: θ > 0 = nghiêng về phía PWM dương (tiến) */
    float tilt      = -(float)c->control_sign * (angle - c->nominal_balance_angle);
    float tilt_rate = -(float)c->control_sign * rate;
    float tilt_err  = fabsf(tilt);

    /* === 1. AN TOÀN KHI NGÃ (> 35°): PWM = 0, STBY = LOW, CHỐT LỖI FALL_LATCH === */
    if (tilt_err > FALL_ANGLE_DEG) {
        BalanceController_EmergencyStop(c, "FALL_LATCH");
        return false;
    }

    /* Tự trả tốc độ tiến/lùi về 0 và trả thẳng lái nếu nhả nút hoặc mất lệnh quá timeout */
    if (fabsf(c->v_cmd) > 0.001f && (now_ms - c->last_speed_cmd_ms > MOTION_CMD_TIMEOUT_MS)) {
        c->v_cmd = 0.0f;
        if (fabsf(c->cmd_turn_pwm) < 1.0f) c->last_cmd = 'X';
    }
    if (fabsf(c->cmd_turn_pwm) > 1.0f && (now_ms - c->last_turn_cmd_ms > MOTION_TURN_TIMEOUT_MS)) {
        c->cmd_turn_pwm = 0.0f;
        if (fabsf(c->v_cmd) < 0.001f) c->last_cmd = 'X';
    }

    /* === 2. VÒNG NGOÀI: VỊ TRÍ KHỐI TÂM & DI CHUYỂN (~50 Hz, cộng dồn dt & tilt_rate thực tế) === */
    if (c->encoder_enabled) {
        c->pos_dt_accum += dt;
        c->pos_rate_accum += tilt_rate * dt;
        if (++c->pos_counter >= POS_LOOP_DIVIDER) {
            float dtp = c->pos_dt_accum;
            c->pos_counter = 0;
            c->pos_dt_accum = 0.0f;
            if (dtp >= 0.004f) {
                if (!position_and_motion_loop(c, dtp)) {
                    return false;
                }
            } else {
                c->pos_rate_accum = 0.0f;
            }
        }
    } else {
        /* Không có encoder hợp lệ -> chỉ giữ cân bằng góc (theta_ref = 0), không chạy hở */
        c->v_cmd = 0.0f;
        c->v_ref = 0.0f;
        c->theta_ref = 0.0f;
        c->pos_rate_accum = 0.0f;
        ICM20602_SetLinearAccel(c->imu, 0.0f);
    }

    /* === 3. VÒNG TRONG: CÂN BẰNG GÓC PD (200 Hz / 5 ms) === */
    c->theta_ref_filt += MOTION_THETA_LPF_ALPHA * (c->theta_ref - c->theta_ref_filt);
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

    /* === 4. KHÓA HƯỚNG ĐI THẲNG & RẼ VI SAI BẰNG GYRO Z (Balance-Priority Yaw Mixer) === */
    float yaw_cmd_dps = (MOTION_TURN_PWM > 1.0f)
        ? ((c->cmd_turn_pwm / MOTION_TURN_PWM) * MOTION_TURN_RATE_DPS)
        : 0.0f;
    c->yaw_rate_ref += MOTION_TURN_LPF_ALPHA * (yaw_cmd_dps - c->yaw_rate_ref);
    if (fabsf(yaw_cmd_dps) < 0.5f && fabsf(c->yaw_rate_ref) < 0.5f) {
        c->yaw_rate_ref = 0.0f;
    }

    float yaw_rate_dps = isfinite(c->imu->gyro_rate_yaw)
        ? ((float)DEFAULT_YAW_SIGN * c->imu->gyro_rate_yaw)
        : 0.0f;

    float headroom = (float)effective - fabsf(c->output);
    if (headroom < 0.0f) headroom = 0.0f;

    float turn_req = 0.0f;
    float turn_eff = 0.0f;
    if (tilt_err <= MOTION_TURN_MAX_TILT_DEG) {
        bool turning = (fabsf(yaw_cmd_dps) > 0.5f) || (fabsf(c->yaw_rate_ref) > 1.5f);
        if (turning) {
            /* Đang bẻ lái A/D: bám tốc độ góc đặt yaw_rate_ref và làm mới mốc hướng đứng thẳng */
            c->yaw_heading_err = 0.0f;
            float yaw_err = c->yaw_rate_ref - yaw_rate_dps;
            turn_req = MOTION_YAW_FF * c->yaw_rate_ref + MOTION_KP_YAW_RATE * yaw_err;
            turn_req = clampf(turn_req, -MOTION_TURN_PWM, MOTION_TURN_PWM);
            turn_eff = clampf(turn_req, -headroom, headroom);
        } else {
            /* Đi thẳng W/S hoặc đứng yên: tích phân -yaw_rate_dps để khóa góc hướng & bù lệch 2 motor N20 */
            bool driving_straight = c->was_moving || (fabsf(c->v_cmd) > 0.001f);
            float deadzone = driving_straight ? 0.10f : MOTION_YAW_DEADZONE_DPS;
            float rate_for_int = (fabsf(yaw_rate_dps) > deadzone) ? yaw_rate_dps : 0.0f;
            float next_heading = clampf(c->yaw_heading_err - rate_for_int * dt,
                                        -MOTION_YAW_HEADING_LIM, MOTION_YAW_HEADING_LIM);
            float cand_turn = MOTION_KI_YAW_HEADING * next_heading - MOTION_KP_YAW_RATE * yaw_rate_dps;
            float max_turn = (headroom < MOTION_TURN_PWM) ? headroom : MOTION_TURN_PWM;
            if (!((cand_turn > max_turn && (-rate_for_int) > 0.0f) ||
                  (cand_turn < -max_turn && (-rate_for_int) < 0.0f))) {
                c->yaw_heading_err = next_heading;
            }
            turn_req = clampf(MOTION_KI_YAW_HEADING * c->yaw_heading_err - MOTION_KP_YAW_RATE * yaw_rate_dps,
                              -MOTION_TURN_PWM, MOTION_TURN_PWM);
            turn_eff = clampf(turn_req, -headroom, headroom);
        }
    } else if (fabsf(c->cmd_turn_pwm) > 1.0f) {
        c->sat_flags |= CTRL_SAT_TURN_CLIP;
    }

    if (fabsf(turn_req) > 1.0f && fabsf(turn_eff - turn_req) > 1.0f) {
        c->sat_flags |= CTRL_SAT_TURN_CLIP;
    }
    c->filtered_turn_pwm = turn_eff;

    int16_t pwm_left  = (int16_t)lroundf(c->output + turn_eff);
    int16_t pwm_right = (int16_t)lroundf(c->output - turn_eff);
    c->last_pwm_left = pwm_left;
    c->last_pwm_right = pwm_right;
    if (!Motor_SetSpeed(c->motor, pwm_left, pwm_right)) {
        BalanceController_EmergencyStop(c, "MOTOR_CUT");
        return false;
    }
    return true;
}
