#ifndef MOTOR_H_
#define MOTOR_H_
#include "stm32l4xx_hal.h"
#include <stdbool.h>

typedef struct {
    TIM_HandleTypeDef *timer;
    TIM_HandleTypeDef *enc_left;
    TIM_HandleTypeDef *enc_right;
    volatile bool enabled,cut;
    bool ready;
    bool enc_ready;
    bool enc_left_ready;
    bool enc_right_ready;
    int16_t limit,deadband,deadband_ramp,left,right;
    int8_t left_sign,right_sign;
    int8_t enc_left_sign,enc_right_sign;
    uint16_t prev_cnt_left,prev_cnt_right;
    int16_t enc_left_delta,enc_right_delta;
    int32_t enc_left_total,enc_right_total;
} Motor_System_t;

bool Motor_Init(Motor_System_t *,TIM_HandleTypeDef *pwm_timer,TIM_HandleTypeDef *enc_l,TIM_HandleTypeDef *enc_r);
bool Motor_Enable(Motor_System_t *);
void Motor_EmergencyCut(Motor_System_t *); /* ISR-safe, latch cannot be cleared by SetSpeed */
void Motor_Stop(Motor_System_t *);
bool Motor_SetSpeed(Motor_System_t *,int16_t left,int16_t right);
bool Motor_SetManual(Motor_System_t *,int16_t left,int16_t right);
void Motor_ReadEncoders(Motor_System_t *);
void Motor_ResetEncoders(Motor_System_t *);

#endif
