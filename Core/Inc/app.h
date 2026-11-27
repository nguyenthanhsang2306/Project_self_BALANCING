#ifndef APP_H
#define APP_H
#include "stm32l4xx_hal.h"
void App_Init(I2C_HandleTypeDef *,UART_HandleTypeDef *,TIM_HandleTypeDef *pwm,TIM_HandleTypeDef *enc_l,TIM_HandleTypeDef *enc_r);
void App_Run(void);
void App_TimerTick(void);
#endif
