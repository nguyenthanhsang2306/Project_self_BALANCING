#include "icm20602.h"
#include "board_config.h"
#include <math.h>
#include <string.h>
#define DEG 57.2957795131f
#if IMU_TILT_AXIS != 0 && IMU_TILT_AXIS != 1
#error IMU_TILT_AXIS must be 0 or 1
#endif

static bool fail(ICM20602_t *s,const char *error) {
    s->ready=false; s->error=error; return false;
}
static float wrap(float a) {
    while(a>180.0f) a-=360.0f;
    while(a< -180.0f) a+=360.0f;
    return a;
}
static float tilt(const ICM20602_t *s,float ax,float ay,float az) {
    if(s->tilt_axis==0) {
        (void)ax; return atan2f(ay,az)*DEG;
    } else {
        (void)ay; return atan2f(-ax,az)*DEG;
    }
}

#ifndef UNIT_TEST
static void dwt_delay_us(uint32_t us) {
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) { }
}
#endif

/* Giải phóng bus I2C1 khi ICM-20602 giữ SDA ở mức LOW (sau khi nhấn Reset hoặc nhiễu dây)
 * và khởi tạo lại ngoại vi I2C1 với Timing ổn định cho dây cắm jumper (~0.15 ms). */
#ifndef UNIT_TEST
static void i2c_bus_recover(I2C_HandleTypeDef *bus) {
    if(!bus || bus->Instance!=I2C1) return;
    GPIO_InitTypeDef g={0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Phát 9 xung clock 100 kHz trên PB8 (SCL) để giải phóng trạng thái kẹt SDA (PB9) */
    g.Pin=GPIO_PIN_8;
    g.Mode=GPIO_MODE_OUTPUT_OD;
    g.Pull=GPIO_PULLUP;
    g.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&g);

    g.Pin=GPIO_PIN_9;
    g.Mode=GPIO_MODE_INPUT;
    g.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB,&g);

    for(unsigned i=0;i<16U;i++) {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
        dwt_delay_us(10U);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
        dwt_delay_us(10U);
        if(i>=8U && HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)==GPIO_PIN_SET) {
            break;
        }
    }

    /* Phát điều kiện STOP (SDA từ LOW -> HIGH khi SCL = HIGH) */
    g.Pin=GPIO_PIN_9;
    g.Mode=GPIO_MODE_OUTPUT_OD;
    g.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB,&g);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
    dwt_delay_us(10U);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
    dwt_delay_us(10U);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
    dwt_delay_us(10U);

    /* Reset cứng thanh ghi I2C1 và cấu hình lại chân PB8/PB9 AF4_I2C1 + Timing chuẩn 100kHz */
    (void)HAL_I2C_DeInit(bus);
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    dwt_delay_us(2U);
    __HAL_RCC_I2C1_RELEASE_RESET();

    g.Pin=GPIO_PIN_8|GPIO_PIN_9;
    g.Mode=GPIO_MODE_AF_OD;
    g.Pull=GPIO_PULLUP;
    g.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate=GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB,&g);

    bus->Init.Timing=IMU_I2C_TIMING;
    (void)HAL_I2C_Init(bus);
    (void)HAL_I2CEx_ConfigAnalogFilter(bus,I2C_ANALOGFILTER_ENABLE);
    (void)HAL_I2CEx_ConfigDigitalFilter(bus,IMU_I2C_DNF);
}
#endif

#ifndef UNIT_TEST
static bool i2c_abort_transfer(I2C_TypeDef *i2c, uint32_t err_mask) {
    i2c->ICR = err_mask | I2C_ICR_STOPCF;
    i2c->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES |
                  I2C_CR2_RELOAD | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);
    return false;
}

/* Đọc thanh ghi I2C ở mức thanh ghi với giới hạn thời gian cứng bằng DWT (us).
 * Loại bỏ hoàn toàn bẫy chờ 25 ms (I2C_TIMEOUT_BUSY & I2C_TIMEOUT_STOPF) trong STM32 HAL,
 * đảm bảo tuyệt đối không bao giờ treo vòng 200 Hz gây DEADLINE_MISS khi nhiễu PWM/EMI trên dây I2C. */
static bool i2c_mem_read_bounded_us(I2C_HandleTypeDef *bus, uint16_t dev_addr, uint8_t reg,
                                    uint8_t *buf, uint16_t len, uint32_t timeout_us) {
    if (!bus || !bus->Instance || !buf || len == 0U || len > 255U) return false;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    I2C_TypeDef *i2c = bus->Instance;
    if ((i2c->ISR & I2C_ISR_BUSY) != 0U || bus->State != HAL_I2C_STATE_READY) {
        return false;
    }

    const uint32_t err_mask = I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR;
    const uint32_t max_cycles = (SystemCoreClock / 1000000U) * timeout_us;
    const uint32_t t0 = DWT->CYCCNT;

    i2c->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;

    /* Pha 1: Gửi địa chỉ slave (Write) + 1 byte địa chỉ thanh ghi (SOFTEND) */
    i2c->CR2 = ((uint32_t)dev_addr & I2C_CR2_SADD) |
               (1U << I2C_CR2_NBYTES_Pos) |
               I2C_CR2_START;

    while ((i2c->ISR & I2C_ISR_TXIS) == 0U) {
        if ((i2c->ISR & err_mask) != 0U || (uint32_t)(DWT->CYCCNT - t0) > max_cycles) {
            return i2c_abort_transfer(i2c, err_mask);
        }
    }
    i2c->TXDR = reg;

    while ((i2c->ISR & I2C_ISR_TC) == 0U) {
        if ((i2c->ISR & err_mask) != 0U || (uint32_t)(DWT->CYCCNT - t0) > max_cycles) {
            return i2c_abort_transfer(i2c, err_mask);
        }
    }

    /* Pha 2: Repeated START (Read) đọc liên tiếp len byte với AUTOEND */
    i2c->CR2 = ((uint32_t)dev_addr & I2C_CR2_SADD) |
               (((uint32_t)len << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) |
               I2C_CR2_RD_WRN |
               I2C_CR2_AUTOEND |
               I2C_CR2_START;

    for (uint16_t i = 0U; i < len; i++) {
        while ((i2c->ISR & I2C_ISR_RXNE) == 0U) {
            if ((i2c->ISR & err_mask) != 0U || (uint32_t)(DWT->CYCCNT - t0) > max_cycles) {
                return i2c_abort_transfer(i2c, err_mask);
            }
        }
        buf[i] = (uint8_t)i2c->RXDR;
    }

    /* Pha 3: Chờ cờ STOPF (tự sinh bởi AUTOEND) trong cùng ngân sách timeout_us */
    while ((i2c->ISR & I2C_ISR_STOPF) == 0U) {
        if ((i2c->ISR & err_mask) != 0U || (uint32_t)(DWT->CYCCNT - t0) > max_cycles) {
            return i2c_abort_transfer(i2c, err_mask);
        }
    }
    i2c->ICR = I2C_ICR_STOPCF;
    i2c->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES |
                  I2C_CR2_RELOAD | I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);
    return true;
}
#endif

static bool write_reg(ICM20602_t *s,uint8_t reg,uint8_t value) {
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

static bool read_regs(ICM20602_t *s,uint8_t reg,uint8_t *buf,uint16_t len) {
#ifdef UNIT_TEST
    return HAL_I2C_Mem_Read(s->hi2c,s->address,reg,I2C_MEMADD_SIZE_8BIT,
                            buf,len,IMU_IO_TIMEOUT_MS)==HAL_OK;
#else
    /* Khi đã vào vòng điều khiển 200Hz (s->ready == true):
     * Đọc có giới hạn cứng bằng DWT. Nếu dính gai EMI thoáng qua, phục hồi bus và chỉ thử đọc lại
     * 1 lần khi còn đủ ngân sách thời gian < IMU_RETRY_BUDGET_US để tổng chu kỳ luôn < 5 ms. */
    if(s->ready) {
        if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        }
        uint32_t t0 = DWT->CYCCNT;
        if(i2c_mem_read_bounded_us(s->hi2c, s->address, reg, buf, len, IMU_RUNTIME_TIMEOUT_US)) {
            return true;
        }
        i2c_bus_recover(s->hi2c);
        uint32_t budget_cycles = (SystemCoreClock / 1000000U) * IMU_RETRY_BUDGET_US;
        if ((uint32_t)(DWT->CYCCNT - t0) < budget_cycles &&
            i2c_mem_read_bounded_us(s->hi2c, s->address, reg, buf, len, IMU_RUNTIME_TIMEOUT_US)) {
            return true;
        }
        return false;
    }
    for(unsigned attempt=0;attempt<2U;attempt++) {
        if(HAL_I2C_Mem_Read(s->hi2c,s->address,reg,I2C_MEMADD_SIZE_8BIT,
                            buf,len,IMU_IO_TIMEOUT_MS)==HAL_OK) return true;
        i2c_bus_recover(s->hi2c);
    }
    return false;
#endif
}

static float clamp_f(float v,float lo,float hi) {
    return v<lo?lo:(v>hi?hi:v);
}

void ICM20602_SetLinearAccel(ICM20602_t *s,float lin_acc_g) {
    if(!s) return;
    if(!isfinite(lin_acc_g)) {
        s->lin_acc_g = 0.0f;
        return;
    }
    s->lin_acc_g = clamp_f(lin_acc_g, -IMU_MAX_LIN_ACC_G, IMU_MAX_LIN_ACC_G);
}

void ICM20602_SetAxis(ICM20602_t *s,uint8_t axis) {
    s->tilt_axis=(axis!=0)?1U:0U;
    if(s->ready) {
        s->pitch_acc=tilt(s,s->ax,s->ay,s->az);
        s->pitch_kalman=s->kalman.angle=s->pitch_acc;
        s->kalman.bias=0;
        memset(s->kalman.P,0,sizeof(s->kalman.P));
        s->gyro_rate_pitch=(s->tilt_axis==0)?s->gx:s->gy;
        s->gyro_rate_yaw=s->gz;
    }
}

bool ICM20602_Init(ICM20602_t *s,I2C_HandleTypeDef *bus) {
    uint8_t saved_axis=s->address?s->tilt_axis:(uint8_t)IMU_TILT_AXIS;
    memset(s,0,sizeof(*s)); s->hi2c=bus; s->tilt_axis=saved_axis; s->error="NOT_CALIBRATED";
    if(!bus) return fail(s,"NO_I2C");

#ifndef UNIT_TEST
    /* Đảm bảo ICM-20602 đã ổn định nguồn sau khi bật điện và làm sạch bus I2C */
    HAL_Delay(30);
    i2c_bus_recover(bus);
    const unsigned max_tries=5U;
#else
    const unsigned max_tries=1U;
#endif

    for(unsigned try_idx=0;try_idx<max_tries && !s->address;try_idx++) {
#ifndef UNIT_TEST
        IWDG->KR = 0xAAAAU;
#endif
        for(unsigned addr=0x68;addr<=0x69;addr++) {
            uint8_t id=0;
            if(HAL_I2C_Mem_Read(bus,(uint16_t)(addr<<1),0x75,I2C_MEMADD_SIZE_8BIT,
                               &id,1,IMU_IO_TIMEOUT_MS)==HAL_OK &&
               (id==0x12U || id==0x11U)) {
                s->address=(uint16_t)(addr<<1);
                s->whoami=id;
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

    /* 1. DEVICE_RESET (0x6B = 0x80) -> chờ 100 ms */
    if(!write_reg(s,0x6B,0x80)) return fail(s,"IMU_RESET");
#ifndef UNIT_TEST
    IWDG->KR = 0xAAAAU;
#endif
    HAL_Delay(100);

    /* 2. SIGNAL_PATH_RESET (0x68 = 0x03): xóa sạch đường lọc số Accel + Temp theo datasheet ICM-20602 */
    if(!write_reg(s,0x68,0x03)) return fail(s,"IMU_RESET");
#ifndef UNIT_TEST
    IWDG->KR = 0xAAAAU;
#endif
    HAL_Delay(15);

    /* 3. PWR_MGMT_1 (0x6B = 0x01): thoát Sleep, chọn Auto-PLL Clock -> chờ 15 ms ổn định PLL */
    if(!write_reg(s,0x6B,0x01)) return fail(s,"IMU_CONFIG");
#ifndef UNIT_TEST
    IWDG->KR = 0xAAAAU;
#endif
    HAL_Delay(15);

    /* 4. Bảng cấu hình chuẩn ICM-20602:
     *    0x70=0x00 (I2C_IF bật I2C), 0x6C=0x00 (PWR_MGMT_2), 0x1A=2 (Gyro DLPF 92 Hz / 3.9 ms),
     *    0x1D=3 (Accel DLPF 44.8 Hz), 0x19=0x00 (ODR 1 kHz), 0x1B=0x08 (+/-500 dps), 0x1C=0x00 (+/-2g),
     *    0x23=0x00 (FIFO_EN), 0x6A=0x00 (USER_CTRL), 0x37=0x00 (INT_PIN_CFG), 0x38=0x01 (DATA_RDY_INT_EN) */
    const uint8_t regs[][2]={
        {0x70,0x00},
        {0x6C,0x00},
        {0x1A,(uint8_t)ICM20602_GYRO_DLPF_CFG},
        {0x1D,(uint8_t)ICM20602_ACCEL_DLPF_CFG},
        {0x19,0x00},
        {0x1B,0x08},
        {0x1C,0x00},
        {0x23,0x00},
        {0x6A,0x00},
        {0x37,0x00},
        {0x38,0x01}
    };
    for(unsigned i=0;i<sizeof(regs)/sizeof(regs[0]);i++)
        if(!write_reg(s,regs[i][0],regs[i][1])) return fail(s,"IMU_CONFIG");
#if IMU_USE_FIXED_GYRO_BIAS
    s->gyro_x_offset = IMU_GYRO_X_OFFSET_LSB;
    s->gyro_y_offset = IMU_GYRO_Y_OFFSET_LSB;
    s->gyro_z_offset = IMU_GYRO_Z_OFFSET_LSB;
#endif
#ifndef UNIT_TEST
    IWDG->KR = 0xAAAAU;
#endif
    HAL_Delay(100); return true;
}

/* Đọc đồng thời 15 byte từ 0x3A (INT_STATUS + 14 byte ACCEL/TEMP/GYRO) trong 1 giao dịch I2C
 * để kiểm tra cờ DATA_RDY_INT đồng nhất trên cả bản chạy thật lẫn UNIT_TEST. */
bool ICM20602_ReadData(ICM20602_t *s) {
    uint8_t b[15];
    if(!read_regs(s,0x3A,b,15)) {
        s->last_data_ready = false;
        if(!s->ready) s->error="I2C_READ";
        return false;
    }
    s->last_data_ready = ((b[0] & 0x01U) != 0U);
    s->raw_accel_x=(int16_t)((uint16_t)b[1]<<8|b[2]);
    s->raw_accel_y=(int16_t)((uint16_t)b[3]<<8|b[4]);
    s->raw_accel_z=(int16_t)((uint16_t)b[5]<<8|b[6]);
    s->raw_gyro_x=(int16_t)((uint16_t)b[9]<<8|b[10]);
    s->raw_gyro_y=(int16_t)((uint16_t)b[11]<<8|b[12]);
    s->raw_gyro_z=(int16_t)((uint16_t)b[13]<<8|b[14]);
    s->ax=s->raw_accel_x/16384.0f; s->ay=s->raw_accel_y/16384.0f; s->az=s->raw_accel_z/16384.0f;
    s->gx=(s->raw_gyro_x-s->gyro_x_offset)/65.5f;
    s->gy=(s->raw_gyro_y-s->gyro_y_offset)/65.5f;
    s->gz=(s->raw_gyro_z-s->gyro_z_offset)/65.5f;
    return true;
}

bool ICM20602_Calibrate(ICM20602_t *s,uint16_t samples) {
    s->ready=false;
    s->last_read_fresh=false;
    s->consecutive_miss=0U;
    s->sample_age_ms=0U;
    if(samples<20 || samples>2000) samples=100;
    float mean[6]={0};
    float vmin[6]={0}, vmax[6]={0};
#if IMU_USE_FIXED_GYRO_BIAS
    s->gyro_x_offset = IMU_GYRO_X_OFFSET_LSB;
    s->gyro_y_offset = IMU_GYRO_Y_OFFSET_LSB;
    s->gyro_z_offset = IMU_GYRO_Z_OFFSET_LSB;
#else
    s->gyro_x_offset = s->gyro_y_offset = s->gyro_z_offset = 0.0f;
#endif
    HAL_Delay(20);
    for(unsigned i=0;i<samples;i++) {
#ifndef UNIT_TEST
        IWDG->KR = 0xAAAAU;
#endif
        if(!ICM20602_ReadData(s)) return fail(s,"I2C_READ");
        float values[6]={s->ax,s->ay,s->az,s->gx,s->gy,s->gz};
        for(unsigned k=0;k<6;k++) {
            mean[k]+=(values[k]-mean[k])/(float)(i+1U);
            if(i==0U) { vmin[k]=vmax[k]=values[k]; }
            else {
                if(values[k]<vmin[k]) vmin[k]=values[k];
                if(values[k]>vmax[k]) vmax[k]=values[k];
            }
        }
        HAL_Delay(2);
    }

    /* Kiểm tra xe PHẢI đứng yên trong lúc calib (không chấp nhận dữ liệu rung/lắc) */
    for(unsigned k=0;k<3;k++) {
        if((vmax[k]-vmin[k]) > IMU_CALIB_MAX_ACC_SPREAD_G) {
            return fail(s,"IMU_MOVING");
        }
    }
    for(unsigned k=3;k<6;k++) {
        if((vmax[k]-vmin[k]) > IMU_CALIB_MAX_GYRO_SPREAD_DPS) {
            return fail(s,"IMU_MOVING");
        }
        if(fabsf(mean[k]) > IMU_CALIB_MAX_BIAS_DPS) {
            return fail(s,"IMU_MOVING");
        }
    }
    float acc_norm = sqrtf(mean[0]*mean[0] + mean[1]*mean[1] + mean[2]*mean[2]);
    if(!isfinite(acc_norm) || acc_norm < 0.80f || acc_norm > 1.20f) {
        return fail(s,"IMU_ACC_ERR");
    }

#if IMU_USE_FIXED_GYRO_BIAS
    /* Khóa nguyên 3 offset gyro LSB đo từ 12000 mẫu (không ghi đè bằng trung bình 300 mẫu).
     * Chỉ trừ 1 lần trong ICM20602_ReadData(); kalman.bias = 0 sẽ tự hấp thụ trôi nhiệt dư. */
    s->gyro_x_offset = IMU_GYRO_X_OFFSET_LSB;
    s->gyro_y_offset = IMU_GYRO_Y_OFFSET_LSB;
    s->gyro_z_offset = IMU_GYRO_Z_OFFSET_LSB;
#else
    s->gyro_x_offset = mean[3] * 65.5f;
    s->gyro_y_offset = mean[4] * 65.5f;
    s->gyro_z_offset = mean[5] * 65.5f;
#endif
    memset(&s->kalman,0,sizeof(s->kalman));
    s->pitch_acc=s->pitch_kalman=s->kalman.angle=tilt(s,mean[0],mean[1],mean[2]);
    s->gyro_rate_pitch=0.0f;
    s->gyro_rate_yaw=0.0f;
    s->lin_acc_g=0.0f;
    s->last_sample_ms=HAL_GetTick();
    s->sample_age_ms=0U;
    s->consecutive_miss=0U;
    s->last_read_fresh=true;
    s->ready=true; s->error="OK"; return true;
}

bool ICM20602_UpdateAngle(ICM20602_t *s,float dt) {
    if(!s->ready) return false;
    if(!isfinite(dt) || dt<0.001f || dt>0.050f) dt = 0.005f;
    uint32_t now = HAL_GetTick();

    bool io_ok = ICM20602_ReadData(s);
    if(!io_ok || !s->last_data_ready) {
        s->last_read_fresh = false;
        s->i2c_err_count++;
        if(s->consecutive_miss < 255U) s->consecutive_miss++;
        s->sample_age_ms = now - s->last_sample_ms;

        /* Chỉ cho phép dự đoán tối đa IMU_MAX_CONSEC_MISS (3 chu kỳ = 15 ms) và tuổi mẫu <= 20 ms */
        if(s->consecutive_miss <= IMU_MAX_CONSEC_MISS && s->sample_age_ms <= IMU_MAX_AGE_MS) {
            s->kalman.angle = wrap(s->kalman.angle + dt * (s->gyro_rate_pitch - s->kalman.bias));
            s->pitch_kalman = s->kalman.angle;
            return true;
        }
        return fail(s, !io_ok ? "I2C_READ" : "IMU_STALE");
    }

    if(s->gx >  480.0f) s->gx =  480.0f; else if(s->gx < -480.0f) s->gx = -480.0f;
    if(s->gy >  480.0f) s->gy =  480.0f; else if(s->gy < -480.0f) s->gy = -480.0f;
    if(s->gz >  480.0f) s->gz =  480.0f; else if(s->gz < -480.0f) s->gz = -480.0f;

    float raw_rate=(s->tilt_axis==0)?s->gx:s->gy;
    s->gyro_rate_pitch=(s->sample_count==0)
        ? raw_rate
        : (GYRO_LPF_ALPHA*raw_rate + (1.0f-GYRO_LPF_ALPHA)*s->gyro_rate_pitch);
    s->gyro_rate_yaw=(s->sample_count==0)
        ? s->gz
        : (GYRO_LPF_ALPHA*s->gz + (1.0f-GYRO_LPF_ALPHA)*s->gyro_rate_yaw);

    Kalman_t *k=&s->kalman;
    k->angle=wrap(k->angle+dt*(raw_rate-k->bias));
    k->P[0][0]+=dt*(dt*k->P[1][1]-k->P[0][1]-k->P[1][0]+0.001f);
    k->P[0][1]-=dt*k->P[1][1]; k->P[1][0]-=dt*k->P[1][1]; k->P[1][1]+=0.003f*dt;

    /* Khử gia tốc tịnh tiến ngang (a_lin_g) khỏi vector gia tốc kế trước khi tính góc trọng trường
     * và tăng hiệp phương sai đo R thích nghi + khóa cập nhật bias gyro khi xe đang tăng/giảm tốc. */
    float a_lin = isfinite(s->lin_acc_g) ? clamp_f(s->lin_acc_g, -IMU_MAX_LIN_ACC_G, IMU_MAX_LIN_ACC_G) : 0.0f;
    float phi_rad = k->angle * (1.0f / DEG);
    float ax_g = s->ax, ay_g = s->ay, az_g = s->az;
    if(s->tilt_axis==0) {
        ay_g = s->ay + a_lin * cosf(phi_rad);
        az_g = s->az - a_lin * sinf(phi_rad);
    } else {
        ax_g = s->ax - a_lin * cosf(phi_rad);
        az_g = s->az - a_lin * sinf(phi_rad);
    }
    s->pitch_acc = tilt(s, ax_g, ay_g, az_g);

    float grav_norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    if(grav_norm > 0.75f && grav_norm < 1.25f) {
        float norm_dev = fabsf(grav_norm - 1.0f);
        float abs_alin = fabsf(a_lin);
        float r_meas = IMU_KALMAN_R_BASE
                     + IMU_KALMAN_R_LIN_SCALE * (abs_alin * abs_alin)
                     + IMU_KALMAN_R_NORM_SCALE * (norm_dev * norm_dev);
        float denom = k->P[0][0] + r_meas;
        float k0 = k->P[0][0] / denom;
        float k1 = k->P[1][0] / denom;
        bool freeze_bias = (abs_alin > IMU_BIAS_FREEZE_ACC_G) || (norm_dev > IMU_BIAS_FREEZE_NORM_G);
        if(freeze_bias) {
            k1 = 0.0f;
        }
        float innovation = wrap(s->pitch_acc - k->angle);
        k->angle = wrap(k->angle + k0 * innovation);
        k->bias += k1 * innovation;
        float p00 = k->P[0][0], p01 = k->P[0][1];
        k->P[0][0] -= k0 * p00;
        k->P[0][1] -= k0 * p01;
        if(freeze_bias) {
            /* Bảo toàn ma trận hiệp phương sai đối xứng xác định dương (Joseph form khi k1 = 0) */
            k->P[1][0] = k->P[0][1];
        } else {
            k->P[1][0] -= k1 * p00;
            k->P[1][1] -= k1 * p01;
        }
    }
    if(!isfinite(k->angle) || !isfinite(k->bias)) return fail(s,"IMU_NAN");
    s->pitch_kalman=k->angle;
    s->last_sample_ms=now;
    s->sample_age_ms=0U;
    s->consecutive_miss=0U;
    s->last_read_fresh=true;
    s->sample_count++;
    return true;
}
