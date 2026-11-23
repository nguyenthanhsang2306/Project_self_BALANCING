#include "mpu6050.h"
#include "board_config.h"
#include <math.h>
#include <string.h>
#define DEG 57.2957795131f
#if IMU_TILT_AXIS != 0 && IMU_TILT_AXIS != 1
#error IMU_TILT_AXIS must be 0 or 1
#endif

static bool fail(MPU6050_t *s,const char *error) {
    s->ready=false; s->error=error; return false;
}
static float wrap(float a) {
    while(a>180.0f) a-=360.0f;
    while(a< -180.0f) a+=360.0f;
    return a;
}
static float tilt(const MPU6050_t *s,float ax,float ay,float az) {
    if(s->tilt_axis==0) {
        (void)ax; return atan2f(ay,az)*DEG;
    } else {
        (void)ay; return atan2f(-ax,az)*DEG;
    }
}

/* Giải phóng bus I2C1 khi MPU6050 giữ SDA ở mức LOW (sau khi nhấn Reset hoặc nhiễu dây)
 * và khởi tạo lại ngoại vi I2C1 với Timing ổn định cho dây cắm jumper. */
static void i2c_bus_recover(I2C_HandleTypeDef *bus) {
#ifndef UNIT_TEST
    if(!bus || bus->Instance!=I2C1) return;
    GPIO_InitTypeDef g={0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Phát 9 xung clock trên PB8 (SCL) để giải phóng trạng thái kẹt SDA (PB9) của MPU6050 */
    g.Pin=GPIO_PIN_8;
    g.Mode=GPIO_MODE_OUTPUT_OD;
    g.Pull=GPIO_PULLUP;
    g.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&g);

    g.Pin=GPIO_PIN_9;
    g.Mode=GPIO_MODE_INPUT;
    g.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB,&g);

    for(unsigned i=0;i<9U;i++) {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
        for(volatile unsigned d=0;d<400U;d++) { __NOP(); }
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
        for(volatile unsigned d=0;d<400U;d++) { __NOP(); }
    }

    /* Phát điều kiện STOP (SDA từ LOW -> HIGH khi SCL = HIGH) */
    g.Pin=GPIO_PIN_9;
    g.Mode=GPIO_MODE_OUTPUT_OD;
    g.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB,&g);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
    for(volatile unsigned d=0;d<400U;d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
    for(volatile unsigned d=0;d<400U;d++) { __NOP(); }
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
    for(volatile unsigned d=0;d<400U;d++) { __NOP(); }

    /* Reset cứng thanh ghi I2C1 và cấu hình lại chân PB8/PB9 AF4_I2C1 + Timing chuẩn 200kHz */
    (void)HAL_I2C_DeInit(bus);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    for(volatile unsigned d=0;d<200U;d++) { __NOP(); }
    __HAL_RCC_I2C1_RELEASE_RESET();

    g.Pin=GPIO_PIN_8|GPIO_PIN_9;
    g.Mode=GPIO_MODE_AF_OD;
    g.Pull=GPIO_PULLUP;
    g.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate=GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB,&g);

    bus->Init.Timing=0x10425F73U;
    (void)HAL_I2C_Init(bus);
    (void)HAL_I2CEx_ConfigAnalogFilter(bus,I2C_ANALOGFILTER_ENABLE);
    (void)HAL_I2CEx_ConfigDigitalFilter(bus,4);
#else
    (void)bus;
#endif
}

static bool write_reg(MPU6050_t *s,uint8_t reg,uint8_t value) {
#ifdef UNIT_TEST
    return HAL_I2C_Mem_Write(s->hi2c,s->address,reg,I2C_MEMADD_SIZE_8BIT,
                             &value,1,IMU_IO_TIMEOUT_MS)==HAL_OK;
#else
    for(unsigned attempt=0;attempt<3U;attempt++) {
        if(HAL_I2C_Mem_Write(s->hi2c,s->address,reg,I2C_MEMADD_SIZE_8BIT,
                             &value,1,IMU_IO_TIMEOUT_MS)==HAL_OK) return true;
        i2c_bus_recover(s->hi2c);
    }
    return false;
#endif
}

static bool read_regs(MPU6050_t *s,uint8_t reg,uint8_t *buf,uint16_t len) {
#ifdef UNIT_TEST
    return HAL_I2C_Mem_Read(s->hi2c,s->address,reg,I2C_MEMADD_SIZE_8BIT,
                            buf,len,IMU_IO_TIMEOUT_MS)==HAL_OK;
#else
    for(unsigned attempt=0;attempt<2U;attempt++) {
        if(HAL_I2C_Mem_Read(s->hi2c,s->address,reg,I2C_MEMADD_SIZE_8BIT,
                            buf,len,4U)==HAL_OK) return true;
        i2c_bus_recover(s->hi2c);
    }
    return false;
#endif
}

void MPU6050_SetAxis(MPU6050_t *s,uint8_t axis) {
    s->tilt_axis=(axis!=0)?1U:0U;
    if(s->ready) {
        s->pitch_acc=tilt(s,s->ax,s->ay,s->az);
        s->pitch_kalman=s->kalman.angle=s->pitch_acc;
        s->kalman.bias=0;
        memset(s->kalman.P,0,sizeof(s->kalman.P));
        s->gyro_rate_pitch=(s->tilt_axis==0)?s->gx:s->gy;
    }
}

bool MPU6050_Init(MPU6050_t *s,I2C_HandleTypeDef *bus) {
    uint8_t saved_axis=s->address?s->tilt_axis:(uint8_t)IMU_TILT_AXIS;
    memset(s,0,sizeof(*s)); s->hi2c=bus; s->tilt_axis=saved_axis; s->error="NOT_CALIBRATED";
    if(!bus) return fail(s,"NO_I2C");

#ifndef UNIT_TEST
    /* Đảm bảo MPU6050 đã ổn định nguồn sau khi bật điện và làm sạch bus I2C */
    HAL_Delay(30);
    i2c_bus_recover(bus);
    const unsigned max_tries=5U;
#else
    const unsigned max_tries=1U;
#endif

    for(unsigned try_idx=0;try_idx<max_tries && !s->address;try_idx++) {
        for(unsigned addr=0x68;addr<=0x69;addr++) {
            uint8_t id=0;
            if(HAL_I2C_Mem_Read(bus,(uint16_t)(addr<<1),0x75,I2C_MEMADD_SIZE_8BIT,
                               &id,1,IMU_IO_TIMEOUT_MS)==HAL_OK &&
               (id==0x68 || id==0x69 || id==0x70 || id==0x72 || id==0x98)) {
                s->address=(uint16_t)(addr<<1);
                break;
            }
        }
#ifndef UNIT_TEST
        if(!s->address) {
            i2c_bus_recover(bus);
            HAL_Delay(20);
        }
#endif
    }

    if(!s->address) return fail(s,"IMU_NOT_FOUND");
    if(!write_reg(s,0x6B,0x80)) return fail(s,"IMU_RESET");
    HAL_Delay(100);
    /* PLL gyro X, DLPF 42/44Hz, data 1kHz, +/-500dps, +/-2g, data ready. */
    const uint8_t regs[][2]={{0x6B,1},{0x6C,0},{0x1A,3},{0x19,0},
        {0x1B,8},{0x1C,0},{0x23,0},{0x6A,0},{0x37,0},{0x38,1}};
    for(unsigned i=0;i<sizeof(regs)/sizeof(regs[0]);i++)
        if(!write_reg(s,regs[i][0],regs[i][1])) return fail(s,"IMU_CONFIG");
    HAL_Delay(100); return true;
}

bool MPU6050_ReadData(MPU6050_t *s) {
    uint8_t b[14];
#ifdef UNIT_TEST
    if(!read_regs(s,0x3B,b,14)) return fail(s,"I2C_READ");
#else
    if(!read_regs(s,0x3B,b,14)) {
        if(!s->ready) s->error="I2C_READ";
        return false;
    }
#endif
    s->raw_accel_x=(int16_t)((uint16_t)b[0]<<8|b[1]);
    s->raw_accel_y=(int16_t)((uint16_t)b[2]<<8|b[3]);
    s->raw_accel_z=(int16_t)((uint16_t)b[4]<<8|b[5]);
    s->raw_gyro_x=(int16_t)((uint16_t)b[8]<<8|b[9]);
    s->raw_gyro_y=(int16_t)((uint16_t)b[10]<<8|b[11]);
    s->raw_gyro_z=(int16_t)((uint16_t)b[12]<<8|b[13]);
    s->ax=s->raw_accel_x/16384.0f; s->ay=s->raw_accel_y/16384.0f; s->az=s->raw_accel_z/16384.0f;
    s->gx=(s->raw_gyro_x-s->gyro_x_offset)/65.5f;
    s->gy=(s->raw_gyro_y-s->gyro_y_offset)/65.5f;
    s->gz=(s->raw_gyro_z-s->gyro_z_offset)/65.5f;
    return true;
}

#ifdef UNIT_TEST
static bool fresh(MPU6050_t *s) {
    uint8_t status=0;
    return read_regs(s,0x3A,&status,1) && (status&1U);
}
#endif

static float clamp_val(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

bool MPU6050_Calibrate(MPU6050_t *s,uint16_t samples) {
    s->ready=false;
    if(samples<20 || samples>2000) samples=100;
    float mean[6]={0};
    s->gyro_x_offset=s->gyro_y_offset=s->gyro_z_offset=0;
    HAL_Delay(20);
    for(unsigned i=0;i<samples;i++) {
        if(!MPU6050_ReadData(s)) return false;
        float values[6]={s->ax,s->ay,s->az,s->gx,s->gy,s->gz};
        for(unsigned k=0;k<6;k++) {
            mean[k]+=(values[k]-mean[k])/(float)(i+1U);
        }
        HAL_Delay(2);
    }
    /* Giới hạn bù trôi tĩnh Gyro trong ngưỡng vật lý ±8 deg/s để dù tay đang cầm xe vẫn không bị lệch offset,
     * phần trôi dư sẽ được bộ lọc Kalman (kalman.bias) tự động bù liên tục ở tần số 200Hz. */
    s->gyro_x_offset=clamp_val(mean[3], -8.0f, 8.0f)*65.5f;
    s->gyro_y_offset=clamp_val(mean[4], -8.0f, 8.0f)*65.5f;
    s->gyro_z_offset=clamp_val(mean[5], -8.0f, 8.0f)*65.5f;
    memset(&s->kalman,0,sizeof(s->kalman));
    s->pitch_acc=s->pitch_kalman=s->kalman.angle=tilt(s,mean[0],mean[1],mean[2]);
    s->gyro_rate_pitch=0; s->last_sample_ms=HAL_GetTick();
    s->ready=true; s->error="OK"; return true;
}

bool MPU6050_UpdateAngle(MPU6050_t *s,float dt) {
    if(!s->ready) return false;
    if(!isfinite(dt) || dt<0.001f || dt>0.050f) dt = 0.005f;
#ifdef UNIT_TEST
    if(!fresh(s)) return fail(s,"IMU_NO_NEW_DATA");
    if(!MPU6050_ReadData(s)) return false;
#else
    if(!MPU6050_ReadData(s)) {
        /* Chống văng FAULT khi có gai nhiễu I2C thoáng qua (< 80ms) lúc motor đảo chiều:
         * Tiếp tục dự đoán góc nghiêng bằng vận tốc góc gần nhất cho đến khi bus I2C hồi phục. */
        if(HAL_GetTick() - s->last_sample_ms <= 80U) {
            s->kalman.angle = wrap(s->kalman.angle + dt * (s->gyro_rate_pitch - s->kalman.bias));
            s->pitch_kalman = s->kalman.angle;
            return true;
        }
        return fail(s,"I2C_READ");
    }
#endif
    if(s->gx >  480.0f) s->gx =  480.0f; else if(s->gx < -480.0f) s->gx = -480.0f;
    if(s->gy >  480.0f) s->gy =  480.0f; else if(s->gy < -480.0f) s->gy = -480.0f;
    if(s->gz >  480.0f) s->gz =  480.0f; else if(s->gz < -480.0f) s->gz = -480.0f;
    s->pitch_acc=tilt(s,s->ax,s->ay,s->az);
    float raw_rate=(s->tilt_axis==0)?s->gx:s->gy;
    s->gyro_rate_pitch=(s->sample_count==0)
        ? raw_rate
        : (GYRO_LPF_ALPHA*raw_rate + (1.0f-GYRO_LPF_ALPHA)*s->gyro_rate_pitch);
    Kalman_t *k=&s->kalman;
    k->angle=wrap(k->angle+dt*(raw_rate-k->bias));
    k->P[0][0]+=dt*(dt*k->P[1][1]-k->P[0][1]-k->P[1][0]+0.001f);
    k->P[0][1]-=dt*k->P[1][1]; k->P[1][0]-=dt*k->P[1][1]; k->P[1][1]+=0.003f*dt;
    float norm=sqrtf(s->ax*s->ax+s->ay*s->ay+s->az*s->az);
    if(norm>0.80f && norm<1.20f) {
        float denom=k->P[0][0]+0.03f;
        float k0=k->P[0][0]/denom,k1=k->P[1][0]/denom;
        float innovation=wrap(s->pitch_acc-k->angle);
        k->angle=wrap(k->angle+k0*innovation); k->bias+=k1*innovation;
        float p00=k->P[0][0],p01=k->P[0][1];
        k->P[0][0]-=k0*p00; k->P[0][1]-=k0*p01;
        k->P[1][0]-=k1*p00; k->P[1][1]-=k1*p01;
    }
    if(!isfinite(k->angle) || !isfinite(k->bias)) return fail(s,"IMU_NAN");
    s->pitch_kalman=k->angle; s->last_sample_ms=HAL_GetTick(); s->sample_count++;
    return true;
}
