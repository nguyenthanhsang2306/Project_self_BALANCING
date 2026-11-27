#include "motor.h"
#include "board_config.h"
#include <string.h>

static void standby(bool on) {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,on?GPIO_PIN_SET:GPIO_PIN_RESET);
}

static void direction(int16_t left,int16_t right) {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,left>0?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,left<0?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,right>0?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,right<0?GPIO_PIN_SET:GPIO_PIN_RESET);
}

void Motor_EmergencyCut(Motor_System_t *s) {
    s->cut=true; s->enabled=false;
    __DMB();
    standby(false);
    if(s->ready) {
        __HAL_TIM_SET_COMPARE(s->timer,TIM_CHANNEL_1,0);
        __HAL_TIM_SET_COMPARE(s->timer,TIM_CHANNEL_2,0);
        s->timer->Instance->EGR=TIM_EGR_UG;
    }
}

void Motor_Stop(Motor_System_t *s) {
    uint32_t key=__get_PRIMASK(); __disable_irq();
    Motor_EmergencyCut(s);
    direction(0,0); s->left=s->right=0;
    standby(false);
    __set_PRIMASK(key);
}

bool Motor_Init(Motor_System_t *s,TIM_HandleTypeDef *timer,TIM_HandleTypeDef *enc_l,TIM_HandleTypeDef *enc_r) {
    memset(s,0,sizeof(*s)); s->cut=true; standby(false);
    s->limit=DEFAULT_PWM_LIMIT;
    s->deadband=DEFAULT_DEADBAND;
    s->deadband_ramp=MOTOR_DEADBAND_RAMP;
    s->left_sign=DEFAULT_LEFT_SIGN; s->right_sign=DEFAULT_RIGHT_SIGN;
    s->enc_left_sign=DEFAULT_ENC_LEFT_SIGN; s->enc_right_sign=DEFAULT_ENC_RIGHT_SIGN;

    if(!timer || timer->Instance!=TIM1) return false;
    uint32_t clock=HAL_RCC_GetPCLK2Freq();
    if(RCC->CFGR&RCC_CFGR_PPRE2) clock*=2U;
    uint32_t frequency=clock/(timer->Instance->PSC+1U)/(timer->Instance->ARR+1U);
    if(frequency!=10000U || timer->Instance->ARR!=999U ||
       (timer->Instance->CR1&(TIM_CR1_DIR|TIM_CR1_CMS))) return false;
    s->timer=timer;
    __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(timer,TIM_CHANNEL_2,0);
    if(HAL_TIM_PWM_Start(timer,TIM_CHANNEL_1)!=HAL_OK) return false;
    if(HAL_TIM_PWM_Start(timer,TIM_CHANNEL_2)!=HAL_OK) {
        (void)HAL_TIM_PWM_Stop(timer,TIM_CHANNEL_1); return false;
    }

    /* Kích hoạt MOE (Main Output Enable) cho TIM1 để xuất xung ra chân PA8 & PA9 */
    __HAL_TIM_MOE_ENABLE(timer);

    /* Khởi tạo Encoder và kiểm tra chặt trạng thái từng kênh */
    if(enc_l && enc_r) {
        s->enc_left = enc_l;
        s->enc_right = enc_r;
        s->enc_left_ready  = (HAL_TIM_Encoder_Start(enc_l, TIM_CHANNEL_ALL) == HAL_OK);
        s->enc_right_ready = (HAL_TIM_Encoder_Start(enc_r, TIM_CHANNEL_ALL) == HAL_OK);
        if(s->enc_left_ready) {
            s->prev_cnt_left = (uint16_t)__HAL_TIM_GET_COUNTER(enc_l);
        }
        if(s->enc_right_ready) {
            s->prev_cnt_right = (uint16_t)__HAL_TIM_GET_COUNTER(enc_r);
        }
        s->enc_ready = (s->enc_left_ready && s->enc_right_ready);
    }

    s->ready=true; Motor_Stop(s); return true;
}

void Motor_ReadEncoders(Motor_System_t *s) {
    if(!s->enc_ready) return;
    uint16_t curr_l = (uint16_t)__HAL_TIM_GET_COUNTER(s->enc_left);
    uint16_t curr_r = (uint16_t)__HAL_TIM_GET_COUNTER(s->enc_right);

    /* Phép trừ số nguyên 16-bit có dấu tự động xử lý tràn số Modulo 65536 */
    int16_t diff_l = (int16_t)(curr_l - s->prev_cnt_left);
    int16_t diff_r = (int16_t)(curr_r - s->prev_cnt_right);

    s->prev_cnt_left = curr_l;
    s->prev_cnt_right = curr_r;

    s->enc_left_delta = diff_l * s->enc_left_sign;
    s->enc_right_delta = diff_r * s->enc_right_sign;

    s->enc_left_total += s->enc_left_delta;
    s->enc_right_total += s->enc_right_delta;
}

void Motor_ResetEncoders(Motor_System_t *s) {
    s->enc_left_delta = 0;
    s->enc_right_delta = 0;
    s->enc_left_total = 0;
    s->enc_right_total = 0;
    if(s->enc_ready) {
        s->prev_cnt_left = (uint16_t)__HAL_TIM_GET_COUNTER(s->enc_left);
        s->prev_cnt_right = (uint16_t)__HAL_TIM_GET_COUNTER(s->enc_right);
    }
}

bool Motor_Enable(Motor_System_t *s) {
    uint32_t key=__get_PRIMASK(); __disable_irq();
    if(!s->ready) { __set_PRIMASK(key); return false; }
    s->cut=false; s->enabled=true;
    __DMB();
    standby(true);
    __set_PRIMASK(key);
    return true;
}

static int16_t output(int16_t value,int16_t limit,int16_t deadband,int16_t deadband_ramp,int8_t sign) {
    if(!value || (deadband>0 && value>=-2 && value<=2)) return 0;
    int32_t magnitude=value<0?-(int32_t)value:value;
    /* Bù vùng chết mềm tuyến tính quanh điểm 0 (|value| < deadband_ramp):
     * - Hạ độ dốc quanh 0 để nhiễu gyro không đập qua lại độ rơ hộp số GA12-N20.
     * - Từ |value| >= deadband_ramp, cấp đủ 100% deadband để thắng ma sát tĩnh. */
    int32_t ramp = (deadband_ramp > 0) ? deadband_ramp : 1;
    int32_t db = (magnitude < ramp) ? ((deadband * magnitude) / ramp) : deadband;
    magnitude += db;
    if(magnitude>limit) magnitude=limit;
    return (int16_t)((value<0?-magnitude:magnitude)*sign);
}

static bool apply(Motor_System_t *s,int16_t left,int16_t right,int16_t limit,int16_t deadband,int16_t ramp) {
    uint32_t key=__get_PRIMASK(); __disable_irq();
    if(!s->ready || !s->enabled || s->cut) { __set_PRIMASK(key); return false; }
    left=output(left,limit,deadband,ramp,s->left_sign);
    right=output(right,limit,deadband,ramp,s->right_sign);

    bool changed=((left>0)-(left<0))!=((s->left>0)-(s->left<0)) ||
                 ((right>0)-(right<0))!=((s->right>0)-(s->right<0));
    if(changed) { direction(left,right); }
    /* Cập nhật trực tiếp CCR1/CCR2 trên nền PWM 10 kHz; KHÔNG ghi TIM_EGR_UG khi đảo chiều */
    __HAL_TIM_SET_COMPARE(s->timer,TIM_CHANNEL_1,left<0?-left:left);
    __HAL_TIM_SET_COMPARE(s->timer,TIM_CHANNEL_2,right<0?-right:right);
    s->left=left; s->right=right;
    standby(true);
    __set_PRIMASK(key); return true;
}

bool Motor_SetSpeed(Motor_System_t *s,int16_t left,int16_t right) {
    return apply(s,left,right,s->limit,s->deadband,s->deadband_ramp);
}

bool Motor_SetManual(Motor_System_t *s,int16_t left,int16_t right) {
    return apply(s,left,right,MANUAL_MAX_PWM,0,1);
}
