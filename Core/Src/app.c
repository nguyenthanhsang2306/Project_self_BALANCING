#include "app.h"
#include "board_config.h"
#include "balance_controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>

/* Globals can be watched in CubeIDE. Use UART commands to change settings. */
ICM20602_t imu;
Motor_System_t motor_sys;
BalanceController_t controller;
static I2C_HandleTypeDef *bus;
static UART_HandleTypeDef *serial;
static uint8_t rx_byte,rx_queue[128],tx_queue[2048];
static volatile uint16_t rx_head,rx_tail,tx_head,tx_tail,tx_length;
static volatile uint8_t tx_busy,stop_requested,serial_fault,deadline_fault;
static volatile uint8_t scheduling;
static volatile uint32_t pending,last_control_ms;
static uint32_t last_cycles,last_log,manual_started;
static char command[80];
static unsigned command_length;
static bool discard_line;
static bool telemetry_enabled; /* Quiet on boot; STATUS is always available. */
#define PROTOCOL_GREETING "PID_PANEL PROTO=3 FW=Project_self_tuning-v4.0\r\n"
typedef enum { STOPPED,MANUAL,BALANCING,FAULT } Mode;
static Mode mode;
static const char *reason="BOOT";

static uint32_t cycles(void) {
#ifdef UNIT_TEST
    return HAL_GetTick()*80000U;
#else
    return DWT->CYCCNT;
#endif
}
static void say(const char *text) {
    size_t n=strlen(text);
    unsigned available=(tx_tail+sizeof(tx_queue)-tx_head-1U)%sizeof(tx_queue);
    if(n>available) return;
    for(size_t i=0;i<n;i++) {
        tx_queue[tx_head]=(uint8_t)text[i]; __DMB();
        tx_head=(tx_head+1U)%sizeof(tx_queue);
    }
}
static void service_tx(void) {
    if(tx_busy || tx_head==tx_tail) return;
    unsigned n=tx_head>tx_tail?(unsigned)(tx_head-tx_tail):sizeof(tx_queue)-tx_tail;
    if(n>64) n=64;
    tx_length=(uint16_t)n; tx_busy=1;
    if(HAL_UART_Transmit_IT(serial,&tx_queue[tx_tail],tx_length)!=HAL_OK) tx_busy=0;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) {
    if(uart!=serial) return;
    tx_tail=(tx_tail+tx_length)%sizeof(tx_queue); __DMB(); tx_busy=0;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
    if(uart!=serial) return;
    if(rx_byte=='x' || rx_byte=='X') {
        Motor_EmergencyCut(&motor_sys); stop_requested=1;
    } else {
        uint16_t next=(rx_head+1U)%sizeof(rx_queue);
        if(next==rx_tail) {
#ifdef UNIT_TEST
            Motor_EmergencyCut(&motor_sys); serial_fault=1;
#endif
        } else {
            rx_queue[rx_head]=rx_byte; __DMB(); rx_head=next;
        }
    }
    if(HAL_UART_Receive_IT(serial,&rx_byte,1)!=HAL_OK) {
#ifdef UNIT_TEST
        Motor_EmergencyCut(&motor_sys); serial_fault=1;
#endif
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
    if(uart==serial) {
#ifdef UNIT_TEST
        Motor_EmergencyCut(&motor_sys); serial_fault=1;
#else
        /* Xóa các cờ lỗi phần cứng UART do rung lắc cáp USB/nhiễu chổi than và tiếp tục nhận */
        __HAL_UART_CLEAR_OREFLAG(serial);
        __HAL_UART_CLEAR_NEFLAG(serial);
        __HAL_UART_CLEAR_FEFLAG(serial);
        __HAL_UART_CLEAR_PEFLAG(serial);
        (void)HAL_UART_Receive_IT(serial,&rx_byte,1);
#endif
    }
}
void App_TimerTick(void) {
    if(!scheduling) return;
    if(pending<255U) pending++;
    if(motor_sys.enabled && HAL_GetTick()-last_control_ms>CONTROL_WATCHDOG_MS) {
        Motor_EmergencyCut(&motor_sys); deadline_fault=1;
    }
}
static void flush_rx(void) {
    uint32_t key=__get_PRIMASK(); __disable_irq();
    rx_tail=rx_head; command_length=0; discard_line=false;
    __set_PRIMASK(key);
}
static void halt(const char *why,bool fault) {
    Mode next=fault?FAULT:STOPPED;
    bool changed=mode!=next || strcmp(reason,why)!=0;
    BalanceController_EmergencyStop(&controller,why);
    mode=next; reason=why;
    if(changed) { say("EVENT MOTOR_OFF REASON="); say(why); say("\r\n"); }
}
static void number(char *out,size_t size,float value) {
    if(!isfinite(value)) { snprintf(out,size,"NaN"); return; }
    long v=lroundf(value*100.0f);
    unsigned long mag=(unsigned long)(v<0?-v:v);
    snprintf(out,size,"%s%lu.%02lu",v<0?"-":"",mag/100UL,mag%100UL);
}
static void setting_number(char *out,size_t size,float value) {
    if(!isfinite(value)) { snprintf(out,size,"NaN"); return; }
    long v=lroundf(value*10000.0f);
    unsigned long mag=(unsigned long)(v<0?-v:v);
    snprintf(out,size,"%s%lu.%04lu",v<0?"-":"",mag/10000UL,mag%10000UL);
}
static const char *mode_name(void) {
    switch(mode) { case MANUAL:return "MANUAL"; case BALANCING:return "BALANCE";
        case FAULT:return "FAULT"; default:return "STOP"; }
}
static void status(bool settings) {
    char text[384],a[20],r[20],gzs[20],p[20],i[20],d[20],z[20],sp[20],si[20],sd[20],yp[20],yd[20],xs[20],tr[20],vs[20],cp[20];
    const char *active_reason = (mode==BALANCING && controller.fall_waiting) ? "FALL_WAIT" :
                                ((mode==BALANCING && controller.reason)?controller.reason:reason);
    number(a,sizeof(a),imu.pitch_kalman);
    number(r,sizeof(r),imu.gyro_rate_pitch);
    number(gzs,sizeof(gzs),imu.gyro_rate_yaw);
    number(xs,sizeof(xs),controller.x*100.0f);
    number(tr,sizeof(tr),controller.theta_ref);
    number(vs,sizeof(vs),controller.v*100.0f);
    snprintf(text,sizeof(text),
             "MODE=%s IMU=%s ANG=%s RATE=%s GZ=%s A=%d B=%d TURN=%d ENC=%ld,%ld X=%s V=%s TREF=%s REASON=%s N=%lu AGE=%lu\r\n",
             mode_name(),imu.error,a,r,gzs,motor_sys.left,motor_sys.right,
             (int)lroundf(controller.turn_output),
             (long)motor_sys.enc_left_total,(long)motor_sys.enc_right_total,xs,vs,tr,
             active_reason,(unsigned long)imu.sample_count,(unsigned long)(HAL_GetTick()-imu.last_sample_ms));
    say(text);
    if(settings) {
        setting_number(p,sizeof(p),controller.pid_angle.Kp); setting_number(i,sizeof(i),controller.pid_angle.Ki);
        setting_number(d,sizeof(d),controller.pid_angle.Kd); setting_number(z,sizeof(z),controller.nominal_balance_angle);
        setting_number(sp,sizeof(sp),controller.pid_pos.Kp); setting_number(si,sizeof(si),controller.pid_pos.Ki);
        setting_number(sd,sizeof(sd),controller.pid_pos.Kd);
        setting_number(yp,sizeof(yp),controller.yaw_kp); setting_number(yd,sizeof(yd),controller.yaw_kd);
        number(cp,sizeof(cp),controller.enc_cpr);
        snprintf(text,sizeof(text),
                 "PID=%s,%s,%s ZERO=%s LIMIT=%d DEAD=%d RAMP=%d DIR=%d,%d SIGN=%d AXIS=%c PPID=%s,%s,%s YPID=%s,%s YSIGN=%d CPR=%s EDIR=%d,%d ENC=%u ADDR=0x%02X WHO=0x%02X TESTMAX=999\r\n",
                 p,i,d,z,motor_sys.limit,motor_sys.deadband,motor_sys.deadband_ramp,
                 motor_sys.left_sign,motor_sys.right_sign,controller.control_sign,
                 imu.tilt_axis==0?'X':'Y',sp,si,sd,yp,yd,controller.yaw_sign,cp,
                 motor_sys.enc_left_sign,motor_sys.enc_right_sign,(unsigned)controller.encoder_enabled,
                 imu.address>>1,imu.whoami);
        say(text);
    }
}
/* Read-only snapshot. Register values are not a measurement of the pin waveform. */
static void diagnostics(void) {
    char text[260],out[20],err[20];
    uint32_t key=__get_PRIMASK(); __disable_irq();
    uint32_t cr1=TIM1->CR1,ccer=TIM1->CCER,bdtr=TIM1->BDTR;
    uint32_t a=TIM1->CCR1,b=TIM1->CCR2,stby=GPIOC->ODR&GPIO_PIN_0;
    bool enabled=motor_sys.enabled,cut=motor_sys.cut;
    int32_t el_tot=motor_sys.enc_left_total,er_tot=motor_sys.enc_right_total;
    __set_PRIMASK(key);
    number(out,sizeof(out),controller.output);
    number(err,sizeof(err),controller.nominal_balance_angle-imu.pitch_kalman);
    snprintf(text,sizeof(text),"DIAG MODE=%s ERR=%s OUT=%s CCR1=%lu CCR2=%lu CEN=%u MOE=%u CH1=%u CH2=%u STBY=%u ENABLE=%u CUT=%u ENCTOT=%ld,%ld\r\n",
        mode_name(),err,out,(unsigned long)a,(unsigned long)b,
        (unsigned)((cr1&TIM_CR1_CEN)!=0),(unsigned)((bdtr&TIM_BDTR_MOE)!=0),
        (unsigned)((ccer&TIM_CCER_CC1E)!=0),(unsigned)((ccer&TIM_CCER_CC2E)!=0),
        (unsigned)(stby!=0),(unsigned)enabled,(unsigned)cut,(long)el_tot,(long)er_tot);
    say(text);
}
static void calibrate(void) {
    halt("CALIB",false); scheduling=0; flush_rx();
    say("CALIB: keep still, ICM-20602 Z up. Wait ~1 second.\r\n");
    uint32_t start=HAL_GetTick();
    while((tx_busy || tx_head!=tx_tail) && HAL_GetTick()-start<250U) service_tx();
    bool ok=false;
    for(unsigned attempt=0; attempt<5U && !ok; attempt++) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        ok = ICM20602_Init(&imu,bus) && ICM20602_Calibrate(&imu,IMU_CALIB_SAMPLES);
        if(!ok) HAL_Delay(200);
    }
    halt(ok?"READY":imu.error,!ok);
    flush_rx(); /* No command typed during calibration is allowed to arm later. */
    uint32_t key=__get_PRIMASK(); __disable_irq();
    pending=0; last_cycles=0; last_control_ms=HAL_GetTick(); scheduling=1;
    __set_PRIMASK(key);
    say(ok?"CALIB OK. Motors OFF. Send ARM to balance, X to stop.\r\n":"CALIB FAILED. Fix wiring/hold still, send C.\r\n");
}
static bool parse_value(const char *token,float *value) {
    char *end; errno=0; *value=strtof(token,&end);
    return end!=token && *end==0 && !errno && isfinite(*value);
}
static bool integer_in(float value,int low,int high) {
    return value>=low && value<=high && value==truncf(value);
}
static void execute(char *line) {
    char *token[6]; unsigned count=0;
    for(char *t=strtok(line," \t");t;t=strtok(NULL," \t")) {
        if(count==6) { say("ERR too many arguments\r\n"); return; }
        token[count++]=t;
    }
    if(!count) return;
    for(char *c=token[0];*c;c++) *c=(char)toupper((unsigned char)*c);
    if(count==1 && !strcmp(token[0],"?")) {
        say(PROTOCOL_GREETING);
        say("X stop. C calib. STATUS report. DIAG PWM. ARM balance. SETZERO capture tilt.\r\n");
        say("M a b: motor test -999..999 (2s). LOG 0/1: quiet/stream 200ms.\r\n");
        say("Live: PID kp ki kd; PPID kp ki kd; YPID kp kd; ZERO deg; LIMIT; DEAD; RAMP; ENC 0/1; CPR n.\r\n");
        say("Stopped: DIR a b; SIGN +1/-1; EDIR a b; YSIGN +1/-1; AXIS 0(X)/1(Y).\r\n");
        say("Settings RAM only: copy chosen values into board_config.h.\r\n"); return;
    }
    if(count==1 && !strcmp(token[0],"STATUS")) { status(true); return; }
    if(count==1 && !strcmp(token[0],"DIAG")) { status(false); diagnostics(); return; }
    if(count==1 && !strcmp(token[0],"C")) { calibrate(); return; }
    if(count==1 && !strcmp(token[0],"SETZERO")) {
        if(mode==MANUAL) { say("ERR send X first\r\n"); return; }
        if(!imu.ready || !isfinite(imu.pitch_kalman) || fabsf(imu.pitch_kalman)>15.0f) {
            say("ERR SETZERO requires ready IMU within +-15deg\r\n"); return;
        }
        uint32_t key=__get_PRIMASK(); __disable_irq();
        controller.nominal_balance_angle=imu.pitch_kalman;
        __set_PRIMASK(key);
        status(true); return;
    }
    if(count==1 && !strcmp(token[0],"ARM")) {
        if(mode==MANUAL || mode==BALANCING) { say("ERR send X first\r\n"); return; }
        uint32_t key=__get_PRIMASK(); __disable_irq();
        bool ok=!stop_requested && !serial_fault && !deadline_fault && BalanceController_Arm(&controller);
        if(ok) { mode=BALANCING; reason="ARMED"; last_control_ms=HAL_GetTick(); }
        __set_PRIMASK(key);
        say(ok?"ARMED. X stops immediately.\r\n":"ARM REJECTED: fresh IMU, Kp>0, ready motor required.\r\n"); return;
    }
    bool hw_locked = (mode==MANUAL || (mode==BALANCING && !controller.fall_waiting));
    if(!strcmp(token[0],"AXIS") && count==2) {
        if(hw_locked) { say("ERR send X before changing AXIS\r\n"); return; }
        char a=(char)toupper((unsigned char)token[1][0]);
        if(token[1][1]==0 && (a=='0' || a=='X')) { ICM20602_SetAxis(&imu,0); status(true); return; }
        if(token[1][1]==0 && (a=='1' || a=='Y')) { ICM20602_SetAxis(&imu,1); status(true); return; }
        say("ERR AXIS 0 (X) or AXIS 1 (Y)\r\n"); return;
    }
    float v[5]={0};
    for(unsigned j=1;j<count;j++) if(!parse_value(token[j],&v[j-1])) {
        say("ERR numeric arguments required\r\n"); return;
    }
    if(!strcmp(token[0],"LOG") && count==2 && integer_in(v[0],0,1)) {
        telemetry_enabled=(v[0]!=0);
        say(telemetry_enabled?"LOG ON\r\n":"LOG OFF\r\n"); return;
    }
    if(!strcmp(token[0],"M") && count==3) {
        if(!integer_in(v[0],-MANUAL_MAX_PWM,MANUAL_MAX_PWM) ||
           !integer_in(v[1],-MANUAL_MAX_PWM,MANUAL_MAX_PWM)) { say("ERR M range -999..999\r\n"); return; }
        if(v[0]==0 && v[1]==0) { halt("STOP",false); say("STOP\r\n"); return; }
        if(mode==BALANCING || !motor_sys.ready) { say("ERR stop balance / check PWM\r\n"); return; }
        uint32_t key=__get_PRIMASK(); __disable_irq();
        bool ok=!stop_requested && !serial_fault && !deadline_fault;
        if(ok) ok=Motor_Enable(&motor_sys) && Motor_SetManual(&motor_sys,(int16_t)v[0],(int16_t)v[1]);
        if(ok) { mode=MANUAL; reason="TEST"; manual_started=HAL_GetTick(); last_control_ms=HAL_GetTick(); }
        __set_PRIMASK(key);
        say(ok?"MANUAL for 2 seconds. X stops.\r\n":"M REJECTED\r\n"); return;
    }
    if(mode==MANUAL) { say("ERR send X before changing settings\r\n"); return; }
    bool is_sign_cmd = (!strcmp(token[0],"DIR") || !strcmp(token[0],"SIGN") ||
                        !strcmp(token[0],"EDIR") || !strcmp(token[0],"YSIGN"));
    if(hw_locked && is_sign_cmd) {
        say("ERR send X before changing signs\r\n"); return;
    }
    bool ok=false;
    uint32_t key=__get_PRIMASK(); __disable_irq();
    if(!strcmp(token[0],"PID") && count==4 && v[0]>=0 && v[0]<=200 &&
       v[1]>=0 && v[1]<=50 && v[2]>=0 && v[2]<=20) {
        controller.pid_angle.Kp=v[0]; controller.pid_angle.Ki=v[1]; controller.pid_angle.Kd=v[2];
        if(mode!=BALANCING || v[1]<=1e-6f) PID_Reset(&controller.pid_angle);
        ok=true;
    } else if(!strcmp(token[0],"ZERO") && count==2 && fabsf(v[0])<=15.0f) {
        controller.nominal_balance_angle=v[0]; ok=true;
    } else if(!strcmp(token[0],"LIMIT") && count==2 && integer_in(v[0],1,ABSOLUTE_PWM_LIMIT) && v[0]>motor_sys.deadband) {
        motor_sys.limit=(int16_t)v[0]; ok=true;
    } else if(!strcmp(token[0],"DEAD") && count==2 && integer_in(v[0],0,250) && v[0]<motor_sys.limit) {
        motor_sys.deadband=(int16_t)v[0]; ok=true;
    } else if(!strcmp(token[0],"RAMP") && count==2 && integer_in(v[0],1,100)) {
        motor_sys.deadband_ramp=(int16_t)v[0]; ok=true;
    } else if(!strcmp(token[0],"DIR") && count==3 && fabsf(v[0])==1 && fabsf(v[1])==1) {
        motor_sys.left_sign=(int8_t)v[0]; motor_sys.right_sign=(int8_t)v[1]; ok=true;
    } else if(!strcmp(token[0],"SIGN") && count==2 && fabsf(v[0])==1) {
        controller.control_sign=(int8_t)v[0]; ok=true;
    } else if((!strcmp(token[0],"PPID") || !strcmp(token[0],"SPID")) && (count==3 || count==4) &&
              v[0]>=0 && v[0]<=50.0f && v[1]>=0 && v[1]<=10.0f && v[2]>=0 && v[2]<=50.0f) {
        controller.pid_pos.Kp=v[0]; controller.pid_pos.Ki=v[1];
        if(count==4) controller.pid_pos.Kd=v[2];
        if(mode!=BALANCING || v[1]<=1e-6f) PID_Reset(&controller.pid_pos);
        ok=true;
    } else if(!strcmp(token[0],"YPID") && count==3 && v[0]>=0 && v[0]<=5.0f && v[1]>=0 && v[1]<=10.0f) {
        controller.yaw_kp=v[0]; controller.yaw_kd=v[1]; ok=true;
    } else if(!strcmp(token[0],"YSIGN") && count==2 && fabsf(v[0])==1) {
        controller.yaw_sign=(int8_t)v[0]; ok=true;
    } else if(!strcmp(token[0],"EDIR") && count==3 && fabsf(v[0])==1 && fabsf(v[1])==1) {
        motor_sys.enc_left_sign=(int8_t)v[0]; motor_sys.enc_right_sign=(int8_t)v[1];
        BalanceController_SetPositionLoop(&controller, controller.encoder_enabled); ok=true;
    } else if(!strcmp(token[0],"ENC") && count==2 && integer_in(v[0],0,1)) {
        BalanceController_SetPositionLoop(&controller, v[0]!=0); ok=true;
    } else if(!strcmp(token[0],"CPR") && count==2 && v[0]>=10.0f && v[0]<=100000.0f) {
        controller.enc_cpr=v[0];
        BalanceController_SetPositionLoop(&controller, controller.encoder_enabled); ok=true;
    }
    __set_PRIMASK(key);
    if(ok) status(true); else say("ERR command/range. Send ?\r\n");
}

void App_Init(I2C_HandleTypeDef *i2c,UART_HandleTypeDef *uart,TIM_HandleTypeDef *pwm,TIM_HandleTypeDef *enc_l,TIM_HandleTypeDef *enc_r) {
    bus=i2c; serial=uart;
#ifndef UNIT_TEST
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT=0; DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    __HAL_DBGMCU_FREEZE_TIM6();
#endif
    bool motor_ok=Motor_Init(&motor_sys,pwm,enc_l,enc_r);
    BalanceController_Init(&controller,&imu,&motor_sys);
    if(HAL_UART_Receive_IT(serial,&rx_byte,1)!=HAL_OK) serial_fault=1;
    say(PROTOCOL_GREETING);
    say("200Hz ICM-20602 / TIM1 10kHz / motors OFF / BIN2=PA10-D2\r\n");
    if(!motor_ok) say("PWM CONFIG ERROR: expected TIM1 CH1+CH2 PSC=7 ARR=999, clock=80MHz\r\n");
    if(motor_sys.enc_ready) say("ENCODER READY: Left=TIM2(PA0/PA1), Right=TIM3(PA6/PA7)\r\n");
    else say("ENCODER DISABLED: running angle-only mode\r\n");
    telemetry_enabled=false;
    calibrate(); status(true);
    say(PROTOCOL_GREETING); /* A host connecting during calibration can identify us. */
    say("QUIET MODE. STATUS reads once; LOG 1 starts streaming.\r\n");
}

void App_Run(void) {
    if(stop_requested || serial_fault || deadline_fault) {
        uint32_t key=__get_PRIMASK(); __disable_irq();
        uint8_t uart_error=serial_fault,late=deadline_fault;
        stop_requested=serial_fault=deadline_fault=0;
        __set_PRIMASK(key);
        halt(uart_error?"UART_ERROR":(late?"CONTROL_WATCHDOG":"STOP"),uart_error||late);
        flush_rx();
        if(uart_error) {
            (void)HAL_UART_AbortReceive(serial);
            if(HAL_UART_Receive_IT(serial,&rx_byte,1)!=HAL_OK) serial_fault=1;
        }
        say("MOTOR STOPPED.\r\n");
    }
#ifndef UNIT_TEST
    /* Tự động kích hoạt lại nhận ngắt UART nếu bị nhỡ do xung đột khóa HAL_LOCK khi đang truyền TX */
    if(serial && serial->RxState==HAL_UART_STATE_READY) {
        (void)HAL_UART_Receive_IT(serial,&rx_byte,1);
    }
#endif
    uint32_t key=__get_PRIMASK(); __disable_irq();
    uint32_t due=pending; pending=0; __set_PRIMASK(key);
    if(due) {
        if(due > CONTROL_MAX_MISSED_TICKS && (mode==BALANCING || mode==MANUAL)) {
            halt("DEADLINE_MISS", true);
            last_cycles = cycles();
        } else {
            uint32_t now_cycles=cycles();
            float dt=last_cycles?(float)(now_cycles-last_cycles)/(float)SystemCoreClock:0.005f;
            last_cycles=now_cycles;
            if(motor_sys.enc_ready) {
                Motor_ReadEncoders(&motor_sys);
            }
            if(imu.ready) {
                if(!isfinite(dt) || dt<0.002f || dt>0.015f) dt=0.005f;
                if(!ICM20602_UpdateAngle(&imu,dt)) {
                    if(mode==BALANCING) halt(imu.error, true);
                } else {
                    last_control_ms=HAL_GetTick();
                    if(mode==BALANCING && !BalanceController_Update(&controller,dt)) {
                        halt(controller.reason,false);
                    }
                }
            }
        }
    }
    if(due==1U && mode==MANUAL) last_control_ms=HAL_GetTick();
    if(mode==MANUAL && HAL_GetTick()-manual_started>=MANUAL_TIMEOUT_MS) halt("TEST_TIMEOUT",false);
    /* Bounded UART work; all control and I2C run in main, never in TIM6 IRQ. */
    for(unsigned j=0;j<16 && rx_tail!=rx_head && !stop_requested && !serial_fault && !deadline_fault;j++) {
        char c=(char)rx_queue[rx_tail]; rx_tail=(rx_tail+1U)%sizeof(rx_queue);
        if(c=='\r' || c=='\n') {
            if(command_length && !discard_line) { command[command_length]=0; execute(command); }
            command_length=0; discard_line=false;
        } else if((c=='\b' || c==127) && !discard_line) {
            if(command_length) command_length--;
        } else if(!discard_line) {
            if(command_length<sizeof(command)-1U) command[command_length++]=c;
            else { discard_line=true; halt("COMMAND_TOO_LONG",true); say("ERR command too long; stopped\r\n"); }
        }
    }
    if(telemetry_enabled && HAL_GetTick()-last_log>=200U) { last_log=HAL_GetTick(); status(false); }
    static uint32_t b1_low_ms = 0;
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
        if(b1_low_ms == 0) b1_low_ms = HAL_GetTick();
        else if(HAL_GetTick() - b1_low_ms > 60U) {
            uint32_t wait_start = HAL_GetTick();
            while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET &&
                  (HAL_GetTick() - wait_start) < 3000U) {
                HAL_Delay(10);
            }
            b1_low_ms = 0;
            calibrate();
            status(true);
        }
    } else {
        b1_low_ms = 0;
    }

    if(mode == BALANCING) {
        if(controller.fall_waiting) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, ((HAL_GetTick() / 200U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        }
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    service_tx();
}
