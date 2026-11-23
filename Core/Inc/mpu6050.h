#ifndef MPU6050_H_
#define MPU6050_H_
#include "stm32l4xx_hal.h"
#include <stdbool.h>
typedef struct { float angle,bias,P[2][2]; } Kalman_t;
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t address;
    uint8_t tilt_axis;
    int16_t raw_accel_x,raw_accel_y,raw_accel_z;
    int16_t raw_gyro_x,raw_gyro_y,raw_gyro_z;
    float ax,ay,az,gx,gy,gz;
    float gyro_x_offset,gyro_y_offset,gyro_z_offset;
    float pitch_acc,pitch_kalman,gyro_rate_pitch;
    Kalman_t kalman;
    uint32_t last_sample_ms,sample_count;
    bool ready;
    const char *error;
} MPU6050_t;
bool MPU6050_Init(MPU6050_t *,I2C_HandleTypeDef *);
bool MPU6050_Calibrate(MPU6050_t *,uint16_t samples);
bool MPU6050_ReadData(MPU6050_t *);
bool MPU6050_UpdateAngle(MPU6050_t *,float dt);
void MPU6050_SetAxis(MPU6050_t *,uint8_t axis);
#endif
