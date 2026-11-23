/**
 * @file pid.h
 * @brief Bộ điều khiển PID số (Digital PID Controller) với Anti-Windup và Output Clamping
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float Kp;              // Hệ số tỉ lệ
    float Ki;              // Hệ số tích phân
    float Kd;              // Hệ số vi phân

    float setpoint;        // Giá trị đặt mong muốn
    float integral;        // Tích lũy sai số
    float prev_error;      // Sai số chu kỳ trước

    float out_min;         // Giới hạn đầu ra nhỏ nhất (vd: -1000)
    float out_max;         // Giới hạn đầu ra lớn nhất (vd: +1000)

    float integral_limit;  // Giới hạn chống tràn tích phân (Anti-windup)
} PID_t;

/* Khởi tạo bộ PID */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max, float integral_limit);

/* Đặt giá trị setpoint */
void PID_SetTarget(PID_t *pid, float setpoint);

/* Tính toán PID tiêu chuẩn: output = Kp*e + Ki*integral + Kd*(e - prev_e)/dt */
float PID_Compute(PID_t *pid, float measured_val, float dt);

/* 
 * Tính toán PID đặc biệt cho cân bằng:
 * Dùng trực tiếp vận tốc góc Gyro làm vi phân thay vì vi phân sai số góc.
 * Tránh lấy đạo hàm số của sai số góc; nhiễu gyro vẫn cần được lọc.
 */
float PID_ComputeWithDerivative(PID_t *pid, float measured_val, float derivative_val, float dt);

/* Reset trạng thái tích phân và sai số cũ */
void PID_Reset(PID_t *pid);

#endif /* PID_H_ */
