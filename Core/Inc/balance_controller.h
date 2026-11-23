#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include "mpu6050.h"
#include "motor.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

/* Cấu trúc Cascade PID cân bằng đứng yên tại chỗ (x_ref = 0):
 *
 *  x_ref=0 ─►(+)─► PID vị trí ─► θ_ref (±8°) ─►(+)─► PID góc ─► PWM ─► 2 Bánh
 *            (-)                                (-)
 *             ▲ x (encoder)                      ▲ θ (MPU6050)
 */
typedef struct {
    MPU6050_t *imu;
    Motor_System_t *motor;
    PID_t pid_angle;            /* Vòng trong (200 Hz): cân bằng góc nghiêng */
    PID_t pid_pos;              /* Vòng ngoài (50 Hz): giữ vị trí đứng yên tại x = 0 */
    float nominal_balance_angle;
    float output;               /* Giá trị PWM tính toán (-999 .. +999) */
    int8_t control_sign;
    bool armed;                 /* true = hệ thống đang sẵn sàng và cấp PWM */
    bool fall_waiting;          /* true = xe ngã, ngắt PWM tạm thời, chờ dựng thẳng dậy */
    bool fault_latched;         /* true = đã dừng khẩn cấp do lỗi/ngắt an toàn, khóa tới khi reset bằng B1 */
    bool encoder_enabled;       /* Cờ cho phép vòng ngoài giữ vị trí */
    bool enc_signal_active;     /* Tự động xác nhận encoder đã cắm và có xung thực */
    uint16_t enc_no_pulse_ticks;/* Đếm chu kỳ motor kéo mạnh nhưng encoder không đổi xung */
    int32_t enc_prev_abs_sum;   /* Tổng trị tuyệt đối xung kỳ trước để phát hiện kẹt/rút dây */
    const char *reason;

    /* Biến trạng thái vòng ngoài giữ vị trí */
    float enc_cpr;              /* Số xung 1 vòng bánh (đã x4) */
    float x;                    /* Độ lệch vị trí so với điểm đứng yên [m] */
    float v;                    /* Vận tốc đã lọc bậc 1 [m/s] */
    float x_prev;
    float theta_ref;            /* Góc nghiêng bù do vòng vị trí tạo ra [độ] */
    uint8_t pos_counter;
} BalanceController_t;

void BalanceController_Init(BalanceController_t *c, MPU6050_t *imu, Motor_System_t *motor);
void BalanceController_SetZeroAngle(BalanceController_t *c, float zero_angle);
void BalanceController_ClearFault(BalanceController_t *c);
void BalanceController_EmergencyStop(BalanceController_t *c, const char *reason);
bool BalanceController_Arm(BalanceController_t *c);
bool BalanceController_Update(BalanceController_t *c, float dt);
void BalanceController_SetPositionLoop(BalanceController_t *c, bool enable);

#endif /* BALANCE_CONTROLLER_H */
