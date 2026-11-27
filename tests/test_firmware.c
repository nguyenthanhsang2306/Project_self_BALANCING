#include "app.h"
#include "balance_controller.h"
#include "board_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

TIM_TypeDef mock_tim1={.PSC=7,.ARR=999};
TIM_TypeDef mock_tim2={.PSC=0,.ARR=65535};
TIM_TypeDef mock_tim3={.PSC=0,.ARR=65535};
RCC_TypeDef mock_rcc;
GPIO_TypeDef mock_a,mock_b,mock_c;
uint32_t SystemCoreClock=80000000U;
static uint32_t tick;
static I2C_HandleTypeDef bus;
static UART_HandleTypeDef uart;
static TIM_HandleTypeDef timer={&mock_tim1};
static TIM_HandleTypeDef enc_l_tim={&mock_tim2};
static TIM_HandleTypeDef enc_r_tim={&mock_tim3};
static bool i2c_error,write_error,moving,stale,second_pwm_fail;
static uint8_t reg_1a,reg_1d;
static int read_count,pwm_stop_count,standby_low_count;
static float tilt_deg, gz_dps;
static uint8_t *rx_destination;
static char serial_output[65536];
static size_t output_size;
extern ICM20602_t imu;
extern Motor_System_t motor_sys;
extern BalanceController_t controller;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick+=ms; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint16_t pin,uint32_t value) {
    if(port==GPIOC && pin==GPIO_PIN_0 && !value) standby_low_count++;
    if(value) port->outputs|=pin; else port->outputs&=~pin;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint16_t pin) {
    if(port==GPIOC && pin==GPIO_PIN_13) return GPIO_PIN_SET; /* B1 unpressed */
    return (port->outputs & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *port,uint16_t pin) {
    port->outputs ^= pin;
}
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 80000000U; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *t,uint32_t c) {
    if(c==TIM_CHANNEL_2 && second_pwm_fail) return HAL_ERROR;
    t->Instance->CR1|=TIM_CR1_CEN;
    t->Instance->BDTR|=TIM_BDTR_MOE;
    t->Instance->CCER|=c==TIM_CHANNEL_1?TIM_CCER_CC1E:TIM_CCER_CC2E;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *t,uint32_t c) {
    (void)t; (void)c; pwm_stop_count++; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *t,uint32_t c) {
    (void)t; (void)c; return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *b,uint16_t a,uint16_t r,uint16_t m,uint8_t *p,uint16_t n,uint32_t wait) {
    (void)b; (void)a; (void)m; (void)wait;
    if(write_error) return HAL_ERROR;
    if(p && n>=1U) {
        if(r==0x1A) reg_1a=p[0];
        else if(r==0x1D) reg_1d=p[0];
    }
    return HAL_OK;
}
static void pack(uint8_t *b,int16_t value) { b[0]=(uint16_t)value>>8; b[1]=(uint8_t)value; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *b,uint16_t a,uint16_t r,uint16_t m,uint8_t *p,uint16_t n,uint32_t wait) {
    (void)b; (void)a; (void)m; (void)wait;
    if(i2c_error) return HAL_ERROR;
    memset(p,0,n);
    if(r==0x75) p[0]=0x12; /* ICM-20602 WHO_AM_I */
    else if(r==0x3A && n==15) {
        p[0]=stale?0:1;
        float angle=tilt_deg*0.01745329252f;
        pack(p+1,(int16_t)lroundf(-sinf(angle)*16384));
        pack(p+3,(int16_t)lroundf(sinf(angle)*16384));
        pack(p+5,(int16_t)lroundf(cosf(angle)*16384));
#if IMU_USE_FIXED_GYRO_BIAS
        pack(p+9,(int16_t)(lroundf(IMU_GYRO_X_OFFSET_LSB)+(moving?((read_count++&1)?400:-400):0)));
        pack(p+11,(int16_t)(lroundf(IMU_GYRO_Y_OFFSET_LSB)+(moving?((read_count&1)?400:-400):0)));
        pack(p+13,(int16_t)lroundf(IMU_GYRO_Z_OFFSET_LSB+gz_dps*65.5f));
#else
        pack(p+9,(int16_t)(66+(moving?((read_count++&1)?400:-400):0)));
        pack(p+11,(int16_t)(66+(moving?((read_count&1)?400:-400):0)));
        pack(p+13,(int16_t)lroundf(gz_dps*65.5f));
#endif
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n) {
    if(output_size+n<sizeof(serial_output)) {
        memcpy(serial_output+output_size,p,n); output_size+=n; serial_output[output_size]=0;
    }
    HAL_UART_TxCpltCallback(u); return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n) {
    (void)u; assert(n==1); rx_destination=p; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }
static void raw_rx(const char *s) {
    while(*s) { *rx_destination=(uint8_t)*s++; HAL_UART_RxCpltCallback(&uart); }
}
static void send(const char *s) { raw_rx(s); for(int i=0;i<16;i++) App_Run(); }
static void step(void) { tick+=5; App_TimerTick(); App_Run(); }

static void test_sensor(void) {
    ICM20602_t s={0};
    write_error=true; assert(!ICM20602_Init(&s,&bus)); write_error=false;
    assert(ICM20602_Init(&s,&bus));
    assert(s.whoami==0x12);
    assert(reg_1a==ICM20602_GYRO_DLPF_CFG && reg_1d==ICM20602_ACCEL_DLPF_CFG);
    assert(reg_1a==2U && reg_1d==3U);
    moving=true; assert(!ICM20602_Calibrate(&s,300)); assert(!s.ready); moving=false;
    assert(ICM20602_Init(&s,&bus)); assert(ICM20602_Calibrate(&s,300));
#if IMU_USE_FIXED_GYRO_BIAS
    assert(fabsf(s.gyro_x_offset-IMU_GYRO_X_OFFSET_LSB)<1e-4f);
    assert(fabsf(s.gyro_y_offset-IMU_GYRO_Y_OFFSET_LSB)<1e-4f);
    assert(fabsf(s.gyro_z_offset-IMU_GYRO_Z_OFFSET_LSB)<1e-4f);
    assert(s.kalman.bias==0.0f);
    assert(ICM20602_ReadData(&s));
    assert(fabsf(s.gy-((float)s.raw_gyro_y-IMU_GYRO_Y_OFFSET_LSB)/65.5f)<1e-4f);
    assert(fabsf(s.gy)<0.02f);
#else
    assert(fabsf(s.gyro_x_offset-66)<0.01f);
#endif
    for(int i=0;i<12000;i++) { tick+=5; assert(ICM20602_UpdateAngle(&s,.005f)); }
    assert(fabsf(s.pitch_kalman)<0.05f);
    tilt_deg=20;
    for(int i=0;i<2000;i++) { tick+=5; assert(ICM20602_UpdateAngle(&s,.005f)); }
    assert(fabsf(s.pitch_kalman-20)<0.2f);
    /* IMU_MAX_CONSEC_MISS transient stale ticks recover; next fails with IMU_STALE */
    stale=true;
    for(unsigned k=0;k<IMU_MAX_CONSEC_MISS;k++) { tick+=5; assert(ICM20602_UpdateAngle(&s,.005f)); assert(!s.last_read_fresh); }
    tick+=5; assert(!ICM20602_UpdateAngle(&s,.005f)); assert(!s.ready);
    stale=false; tilt_deg=0;
    puts("PASS ICM-20602: WHO_AM_I=0x12, config errors, moving calibration, bias, 60s stationary, tilt, miss predict & stale fault");
}
static void test_motor_and_controller(void) {
    Motor_System_t m;
    second_pwm_fail=true; assert(!Motor_Init(&m,&timer,&enc_l_tim,&enc_r_tim)); assert(pwm_stop_count==1);
    assert(!(mock_c.outputs&GPIO_PIN_0)); second_pwm_fail=false;
    assert(Motor_Init(&m,&timer,&enc_l_tim,&enc_r_tim)); assert(!Motor_SetSpeed(&m,100,100));
    assert(m.left_sign==DEFAULT_LEFT_SIGN && m.right_sign==DEFAULT_RIGHT_SIGN);
    assert(m.enc_left_sign==DEFAULT_ENC_LEFT_SIGN && m.enc_right_sign==DEFAULT_ENC_RIGHT_SIGN);
    assert(m.deadband_ramp==MOTOR_DEADBAND_RAMP);

    /* Explicit unit signs + soft deadband ramp check */
    m.left_sign=1; m.right_sign=1; m.enc_left_sign=1; m.enc_right_sign=1;
    assert(Motor_Enable(&m)); m.limit=300; m.deadband=35; m.deadband_ramp=35;
    assert(Motor_SetSpeed(&m,10,-10)); /* inside ramp: |db| = 10 -> total 20 */
    assert(mock_tim1.CCR1==20 && mock_tim1.CCR2==20);
    assert(Motor_SetSpeed(&m,500,-100)); assert(mock_tim1.CCR1==300 && mock_tim1.CCR2==135);
    assert(mock_a.outputs&GPIO_PIN_10); assert(!(mock_b.outputs&GPIO_PIN_6));
    assert(!(mock_b.outputs&GPIO_PIN_7));
    assert(Motor_SetManual(&m,500,-999));
    assert(mock_tim1.CCR1==500 && mock_tim1.CCR2==999);
    int blanks=standby_low_count;
    mock_tim1.EGR=0;
    for(int j=0;j<200;j++) assert(Motor_SetSpeed(&m,100+j,-100-j));
    assert(standby_low_count==blanks && mock_tim1.EGR==0);
    assert(mock_c.outputs&GPIO_PIN_0);
    /* Sign flip in apply() MUST NOT trigger TIM_EGR_UG or drop STBY */
    assert(Motor_SetSpeed(&m,-200,200));
    assert(standby_low_count==blanks && mock_tim1.EGR==0);
    assert(!(mock_a.outputs&GPIO_PIN_10) && (mock_b.outputs&GPIO_PIN_6));
    Motor_EmergencyCut(&m); assert(!Motor_SetSpeed(&m,200,200)); assert(!(mock_c.outputs&GPIO_PIN_0));
    assert(mock_tim1.CCR1==0 && mock_tim1.CCR2==0 && mock_tim1.EGR==TIM_EGR_UG);
    Motor_Stop(&m); assert(mock_tim1.CCR1==0 && mock_tim1.CCR2==0);

    /* Encoder modulo delta arithmetic */
    mock_tim2.CNT=65530; mock_tim3.CNT=10;
    Motor_ResetEncoders(&m);
    mock_tim2.CNT=4;
    mock_tim3.CNT=25;
    Motor_ReadEncoders(&m);
    assert(m.enc_left_delta==10 && m.enc_left_total==10);
    assert(m.enc_right_delta==15 && m.enc_right_total==15);

    /* Balance controller + FALL_WAIT STBY=LOW + rebase preserving integral + Yaw Hold */
    ICM20602_t s={.ready=true,.last_read_fresh=true,.last_sample_ms=tick,.pitch_kalman=DEFAULT_ZERO_DEG,.error="OK"};
    BalanceController_t c; BalanceController_Init(&c,&s,&m);
    assert(c.control_sign==DEFAULT_CONTROL_SIGN && c.yaw_sign==DEFAULT_YAW_SIGN);
    c.control_sign=1;
    c.pid_angle.Kp=0; assert(!BalanceController_Arm(&c));
    c.pid_angle.Kp=10; c.pid_angle.Ki=0; c.pid_angle.Kd=0;
    c.encoder_enabled=false; c.yaw_kp=0.5f; c.yaw_kd=1.0f;
    assert(BalanceController_Arm(&c));
    s.pitch_kalman=DEFAULT_ZERO_DEG-5.0f;
    assert(BalanceController_Update(&c,.005f)); assert(fabsf(c.output-50.0f)<0.1f);
    assert(c.pid_angle.integral==0.0f); /* Ki==0 short-circuits integral */

    /* Fall >35 deg -> FALL_WAIT pulls STBY=LOW, <=15 deg -> resumes STBY=HIGH */
    s.pitch_kalman=DEFAULT_ZERO_DEG+40.0f; assert(BalanceController_Update(&c,.005f));
    assert(c.fall_waiting && c.output==0 && mock_tim1.CCR1==0 && !(mock_c.outputs&GPIO_PIN_0));
    s.pitch_kalman=DEFAULT_ZERO_DEG+22.0f; assert(BalanceController_Update(&c,.005f));
    assert(c.fall_waiting && c.output==0 && !(mock_c.outputs&GPIO_PIN_0));
    s.pitch_kalman=DEFAULT_ZERO_DEG+10.0f; assert(BalanceController_Update(&c,.005f));
    assert(!c.fall_waiting && fabsf(c.output+100.0f)<0.1f && (mock_c.outputs&GPIO_PIN_0));

    /* Yaw Hold + CoM Position Hold + Rebase preserving integral */
    s.pitch_kalman=DEFAULT_ZERO_DEG; s.gyro_rate_yaw=-20.0f; /* CW with yaw_sign=-1 -> yaw_rate_dps=+20 */
    c.encoder_enabled=true;
    for(int step_i=0;step_i<10;step_i++) {
        tick+=5; s.last_sample_ms=tick;
        assert(BalanceController_Update(&c,.005f));
    }
    assert(c.turn_output < -15.0f); /* negative feedback opposes +yaw_rate */
    s.gyro_rate_yaw=0.0f;
    c.pid_pos.integral=2.8f;
    mock_tim2.CNT+=7000; mock_tim3.CNT+=7000; /* >0.60m */
    Motor_ReadEncoders(&m);
    for(int k=0;k<4;k++) { tick+=5; s.last_sample_ms=tick; assert(BalanceController_Update(&c,.005f)); }
    assert(fabsf(c.pid_pos.integral) > 2.5f && fabsf(c.pid_pos.integral) <= POS_INTEGRAL_LIMIT); /* preserved on runaway rebase */

    tick+=IMU_MAX_AGE_MS+5U; /* > IMU_MAX_AGE_MS */
    assert(!BalanceController_Update(&c,.005f)); assert(!m.enabled);
    PID_t p; PID_Init(&p,100,1,0,-300,300,100);
    PID_SetTarget(&p,10);
    for(int i=0;i<1000;i++) PID_ComputeWithDerivative(&p,0,0,.005f);
    assert(p.integral==0); assert(PID_ComputeWithDerivative(&p,11,0,.005f)<0);
    puts("PASS motor/PID: soft deadband ramp, zero EGR flicker, FALL_WAIT STBY=LOW, Yaw Hold, CoM rebase, anti-windup");
}
static void test_app(void) {
    tilt_deg=0; gz_dps=0;
    App_Init(&bus,&uart,&timer,&enc_l_tim,&enc_r_tim); step();
    assert(imu.ready && !motor_sys.enabled); /* stays OFF after calibration */
    for(int j=0;j<100;j++) App_Run();
    size_t quiet_size=output_size;
    for(int j=0;j<100;j++) step();
    assert(output_size==quiet_size);
    send("STATUS\r"); assert(output_size>quiet_size);
    assert(strstr(serial_output,"WHO=0x12"));
    assert(strstr(serial_output,"RAMP=48"));
    assert(strstr(serial_output,"YPID="));
    assert(strstr(serial_output,"YSIGN=-1"));
    send("LOG 1\r");
    quiet_size=output_size;
    for(int j=0;j<50;j++) step();
    assert(output_size>quiet_size);
    send("LOG 0\r");
    send("PID 0 0 0\r");
    send("ARM\r"); assert(!controller.armed);
    send("PID nan 0 0\r"); assert(controller.pid_angle.Kp==0);
    send("PID 10 0.0001 0.005\r");
    send("STATUS\r");
    assert(strstr(serial_output,"PID=10.0000,0.0001,0.0050"));
    send("?\r"); assert(strstr(serial_output,"PID_PANEL PROTO=3 FW=Project_self_tuning-v4.0"));

    /* Live PPID, YPID, RAMP, ZERO, SETZERO, and sign commands while stopped */
    send("PPID 3.0 0.4 4.0\r");
    assert(fabsf(controller.pid_pos.Kp-3.0f)<0.01f && fabsf(controller.pid_pos.Kd-4.0f)<0.01f);
    send("YPID 0.6 1.2\r");
    assert(fabsf(controller.yaw_kp-0.6f)<0.01f && fabsf(controller.yaw_kd-1.2f)<0.01f);
    send("YPID 0 0\r");
    send("RAMP 25\r"); assert(motor_sys.deadband_ramp==25);
    send("EDIR 1 1\r"); assert(motor_sys.enc_left_sign==1 && motor_sys.enc_right_sign==1);
    send("YSIGN 1\r"); assert(controller.yaw_sign==1);
    send("ENC 0\r"); assert(!controller.encoder_enabled);
    send("ZERO 0\r");
    send("PID 30 0 0\r");
    send("LIMIT 300\r"); send("DEAD 0\r");
    send("SIGN 1\r"); send("DIR 1 -1\r");
    send("ARM\r");
    assert(controller.armed);
    /* Trying to change signs while balancing upright is rejected */
    send("DIR -1 1\r"); assert(motor_sys.left_sign==1 && motor_sys.right_sign==-1);
    tilt_deg=12;
    for(int j=0;j<1000;j++) step();
    assert(controller.armed && motor_sys.left==-300 && motor_sys.right==300);
    assert(mock_tim1.CCR1==300 && mock_tim1.CCR2==300);
    assert(mock_c.outputs&GPIO_PIN_0);
    send("DIAG\r");
    assert(strstr(serial_output,"DIAG MODE=BALANCE"));
    assert(strstr(serial_output,"CCR1=300 CCR2=300 CEN=1 MOE=1 CH1=1 CH2=1 STBY=1 ENABLE=1 CUT=0"));
    tilt_deg=-12;
    for(int j=0;j<1000;j++) step();
    assert(controller.armed && motor_sys.left==300 && motor_sys.right==-300);
    send("X"); assert(mock_tim1.CCR1==0 && mock_tim1.CCR2==0);
    tilt_deg=2.25f; send("C\r"); step();
    send("SETZERO\r");
    assert(fabsf(controller.nominal_balance_angle-2.25f)<0.1f);
    tilt_deg=0; send("C\r"); step(); send("ZERO 0\r");
    send("DIR 1 1\r"); send("PID 10 0 0\r");
    puts("PASS balance end-to-end: v4.0 status, SETZERO, PPID/YPID/RAMP/EDIR/YSIGN, ARM, signed PWM 300, X cut");

    send("M 150 0\r"); assert(motor_sys.enabled && mock_tim1.CCR1==150);
    send("PID 99 0 0\r"); assert(controller.pid_angle.Kp==10);
    raw_rx("X"); assert(!(mock_c.outputs&GPIO_PIN_0));
    assert(mock_tim1.CCR1==0 && mock_tim1.CCR2==0);
    raw_rx("M 250 250\r"); App_Run(); assert(!motor_sys.enabled);
    send("DIR 1 -1\r"); send("M 0 150\r"); assert(motor_sys.right==-150);
    for(int i=0;i<401;i++) step();
    assert(!motor_sys.enabled);
    send("ARM\r"); assert(controller.armed);
    i2c_error=true;
    for(unsigned k=0;k<IMU_MAX_CONSEC_MISS;k++) { step(); assert(motor_sys.enabled && imu.ready); }
    step(); assert(!motor_sys.enabled && !imu.ready); /* miss > IMU_MAX_CONSEC_MISS triggers I2C_READ halt */
    i2c_error=false;
    send("ARM\r"); assert(!controller.armed);
    send("M 500 500\r");
    assert(motor_sys.enabled && mock_tim1.CCR1==500 && mock_tim1.CCR2==500);
    for(int j=0;j<10;j++) step();
    assert(motor_sys.enabled);
    for(int j=0;j<391;j++) step();
    assert(!motor_sys.enabled);
    send("DIR 1 1\r"); send("M -999 -500\r");
    assert(motor_sys.enabled && mock_tim1.CCR1==999 && mock_tim1.CCR2==500);
    assert(mock_a.outputs&GPIO_PIN_10);
    assert(!(mock_b.outputs&GPIO_PIN_7));
    send("X"); assert(!motor_sys.enabled);
    send("M 1000 0\r"); assert(!motor_sys.enabled);
    send("C\r"); step(); assert(imu.ready && !motor_sys.enabled);
    send("M 500 500\r"); i2c_error=true;
    for(unsigned k=0;k<=IMU_MAX_CONSEC_MISS;k++) step();
    assert(motor_sys.enabled && !imu.ready); /* losing IMU does not cancel bench test */
    for(unsigned j=0;j<(401U-(IMU_MAX_CONSEC_MISS+1U));j++) step();
    assert(!motor_sys.enabled && mock_tim1.CCR1==0 && mock_tim1.CCR2==0);
    i2c_error=false; send("C\r"); step();
    send("ARM\r"); assert(controller.armed);
    tick+=CONTROL_WATCHDOG_MS+10U; App_TimerTick(); assert(!motor_sys.enabled && !(mock_c.outputs&GPIO_PIN_0)); App_Run();
    send("C\r"); step(); send("M 150 150\r"); assert(motor_sys.enabled);
    HAL_UART_ErrorCallback(&uart); assert(!motor_sys.enabled); App_Run();
    send("M 150 150\r"); assert(motor_sys.enabled);
    for(int i=0;i<130;i++) raw_rx("a");
    assert(!motor_sys.enabled); App_Run();
    send("C\r"); step();
    send("M 150 150\r"); for(int k=0;k<4;k++){ tick+=5; App_TimerTick(); } App_Run();
    assert(!motor_sys.enabled && strcmp(controller.reason,"DEADLINE_MISS")==0);
    puts("PASS app: quiet/log/status, v4.0 tuning commands, manual timeout, X flush, I2C miss & recovery, watchdog, deadline");
}
int main(void) {
    test_sensor(); test_motor_and_controller(); test_app();
    puts("ALL FIRMWARE TESTS PASS (simulated HAL, not hardware)"); return 0;
}
