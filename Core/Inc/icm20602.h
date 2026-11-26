#ifndef ICM20602_H_
#define ICM20602_H_
#include "stm32l4xx_hal.h"
#include <stdbool.h>
typedef struct { float angle,bias,P[2][2]; } Kalman_t;
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t address;
    uint8_t whoami;
    uint8_t tilt_axis;
    int16_t raw_accel_x,raw_accel_y,raw_accel_z;
    int16_t raw_gyro_x,raw_gyro_y,raw_gyro_z;
    float ax,ay,az,gx,gy,gz;
    float gyro_x_offset,gyro_y_offset,gyro_z_offset;
    float pitch_acc,pitch_kalman,gyro_rate_pitch,gyro_rate_yaw;
    float lin_acc_g; /* Gia tốc ngang ước lượng dọc trục nghiêng [g] để bù trọng lực biểu kiến */
    Kalman_t kalman;
    uint32_t last_sample_ms,sample_count;
    uint32_t sample_age_ms,i2c_err_count;
    uint8_t consecutive_miss;
    bool last_data_ready;
    bool last_read_fresh;
    bool ready;
    const char *error;
} ICM20602_t;
bool ICM20602_Init(ICM20602_t *,I2C_HandleTypeDef *);
bool ICM20602_Calibrate(ICM20602_t *,uint16_t samples);
bool ICM20602_ReadData(ICM20602_t *);
bool ICM20602_UpdateAngle(ICM20602_t *,float dt);
void ICM20602_SetAxis(ICM20602_t *,uint8_t axis);
void ICM20602_SetLinearAccel(ICM20602_t *,float lin_acc_g);
#endif
