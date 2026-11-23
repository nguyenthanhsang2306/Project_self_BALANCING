/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Self-Balancing & Remote Motion Controller (STM32L476RG)
  *                   Nhận lệnh qua ngắt UART Ring Buffer (USART2 & LPUART1)
  *                   Điều khiển 200 Hz bằng TIM6 và gửi Telemetry non-blocking
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "board_config.h"
#include "mpu6050.h"
#include "motor.h"
#include "balance_controller.h"
#include <math.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;
UART_HandleTypeDef hlpuart1;

static MPU6050_t imu;
static Motor_System_t motor_sys;
static BalanceController_t controller;
static uint32_t last_cycles = 0U;
static volatile bool recalib_requested = false;

/* Ring buffer nhận lệnh qua ngắt RXNE cho cả USART2 và LPUART1 */
#define RX_RING_SIZE 128U
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0U;
static volatile uint16_t rx_tail = 0U;

/* Buffer truyền Telemetry non-blocking đồng thời ra USART2 và LPUART1 */
static char tx_buf[128];
static int tx_len = 0;
static int tx_idx_u2 = 0;
static int tx_idx_lp = 0;
static uint32_t last_cmd_blink_ms = 0U;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);

/* Ngắt nhận RXNE cho cả USART2 (PA3/D0) và LPUART1 (PB10/D6) */
void Motion_UART_RxIRQ(USART_TypeDef *uart)
{
  if (!uart) return;
  uint32_t isr = uart->ISR;
  if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE))
  {
    uart->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_PECF;
  }
  while (uart->ISR & USART_ISR_RXNE)
  {
    uint8_t b = (uint8_t)(uart->RDR & 0xFFU);
    uint16_t next = (uint16_t)((rx_head + 1U) % RX_RING_SIZE);
    if (next != rx_tail)
    {
      rx_ring[rx_head] = b;
      rx_head = next;
    }
  }
}

static void SelfBalance_CalibrateAndArm(bool capture_upright_zero)
{
  BalanceController_EmergencyStop(&controller, "CALIB");
  bool ok = false;
  for (unsigned attempt = 0; attempt < 10U && !ok; attempt++)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    ok = MPU6050_Init(&imu, &hi2c1);
    if (ok)
    {
      MPU6050_SetAxis(&imu, IMU_TILT_AXIS);
      ok = MPU6050_Calibrate(&imu, IMU_CALIB_SAMPLES);
    }
    if (!ok) HAL_Delay(30);
  }
  if (ok)
  {
    float zero_deg = DEFAULT_ZERO_DEG;
    if (B1_CAPTURE_ZERO_ENABLE && capture_upright_zero && isfinite(imu.pitch_kalman) &&
        fabsf(imu.pitch_kalman - DEFAULT_ZERO_DEG) <= B1_ZERO_CAPTURE_DEG)
    {
      zero_deg = imu.pitch_kalman;
    }
    BalanceController_SetZeroAngle(&controller, zero_deg);
    BalanceController_ClearFault(&controller);
    (void)BalanceController_Arm(&controller);
  }
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
  last_cycles = DWT->CYCCNT;
}

static void Motion_HandleByte(uint8_t b)
{
  char c = (char)b;
  if (c == 'w' || c == 'W' ||
      c == 's' || c == 'S' ||
      c == 'v' || c == 'V' ||
      c == 'a' || c == 'A' ||
      c == 'd' || c == 'D' ||
      c == 'c' || c == 'C' ||
      c == 'x' || c == 'X' || c == ' ' ||
      c == '1' || c == '2' || c == '3')
  {
    BalanceController_ProcessCommandChar(&controller, c);
    last_cmd_blink_ms = HAL_GetTick();
  }
  else if (c == 'k' || c == 'K')
  {
    /* Gói Keep-Alive làm mới timeout và giữ đèn LD2 nháy nhanh khi đang đè nút */
    BalanceController_ProcessCommandChar(&controller, c);
    if (controller.last_cmd != 'X')
    {
      last_cmd_blink_ms = HAL_GetTick();
    }
  }
  else if (c == '!')
  {
    /* Ngắt khẩn cấp chốt an toàn */
    BalanceController_EmergencyStop(&controller, "WEB_ESTOP");
  }
  else if ((c == 'i' || c == 'I') &&
           (!controller.armed || controller.fall_waiting || controller.fault_latched))
  {
    /* Chỉ cho phép calib IMU khi xe chưa ARM, đang FALL_WAIT hoặc đã khóa lỗi; chặn khi đang BALANCE */
    recalib_requested = true;
    last_cmd_blink_ms = HAL_GetTick();
  }
  else if ((c == 'z' || c == 'Z') &&
           (!controller.armed || controller.fall_waiting || controller.fault_latched) &&
           imu.ready && isfinite(imu.pitch_kalman) && fabsf(imu.pitch_kalman) <= 15.0f)
  {
    BalanceController_SetZeroAngle(&controller, imu.pitch_kalman);
    last_cmd_blink_ms = HAL_GetTick();
  }
  else if ((c == 'r' || c == 'R') && !controller.armed && imu.ready)
  {
    BalanceController_ClearFault(&controller);
    (void)BalanceController_Arm(&controller);
    last_cmd_blink_ms = HAL_GetTick();
  }
}

static void Motion_PollUART(void)
{
  while (rx_tail != rx_head)
  {
    uint8_t b = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % RX_RING_SIZE);
    Motion_HandleByte(b);
  }

  /* Đẩy từng byte Telemetry ra USART2 (PA2/D1) và LPUART1 (PC1/A4) khi cờ TXE rảnh (non-blocking) */
  if (tx_idx_u2 < tx_len && (USART2->ISR & USART_ISR_TXE))
  {
    USART2->TDR = (uint8_t)tx_buf[tx_idx_u2++];
  }
  if (tx_idx_lp < tx_len && (LPUART1->ISR & USART_ISR_TXE))
  {
    LPUART1->TDR = (uint8_t)tx_buf[tx_idx_lp++];
  }
}

static void buf_append_str(char *dst, int *pos, int max_len, const char *s)
{
  while (s && *s && *pos < max_len - 1)
  {
    dst[(*pos)++] = *s++;
  }
}

static void buf_append_int(char *dst, int *pos, int max_len, int32_t val)
{
  char tmp[16];
  int i = 0;
  if (val < 0)
  {
    if (*pos < max_len - 1) dst[(*pos)++] = '-';
    val = -val;
  }
  do
  {
    tmp[i++] = (char)('0' + (val % 10));
    val /= 10;
  } while (val > 0 && i < (int)sizeof(tmp));
  while (i > 0 && *pos < max_len - 1)
  {
    dst[(*pos)++] = tmp[--i];
  }
}

static void buf_append_tenths(char *dst, int *pos, int max_len, float val)
{
  if (!isfinite(val)) val = 0.0f;
  int32_t scaled = (int32_t)lroundf(val * 10.0f);
  if (scaled < 0)
  {
    if (*pos < max_len - 1) dst[(*pos)++] = '-';
    scaled = -scaled;
  }
  buf_append_int(dst, pos, max_len, scaled / 10);
  if (*pos < max_len - 1) dst[(*pos)++] = '.';
  buf_append_int(dst, pos, max_len, scaled % 10);
}

/* Đóng gói Telemetry vào tx_buf (0 ms blocking trong vòng lặp 200 Hz) */
static void Motion_SendTelemetry(void)
{
  if (tx_idx_u2 < tx_len || tx_idx_lp < tx_len)
  {
    return;
  }
  int pos = 0;
  /* STAT:<pitch>,<theta_ref>,<v_cmd_cms>,<cmd>,<reason>,<v_real_cms>,<zero_deg>,<output>,<enc_l>,<enc_r>,<enc_state> */
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), "STAT:");
  buf_append_tenths(tx_buf, &pos, (int)sizeof(tx_buf), imu.pitch_kalman);
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), ",");
  buf_append_tenths(tx_buf, &pos, (int)sizeof(tx_buf), controller.theta_ref);
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), ",");
  buf_append_int(tx_buf, &pos, (int)sizeof(tx_buf), (int32_t)lroundf(controller.v_cmd * 100.0f));
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), ",");
  if (pos < (int)sizeof(tx_buf) - 1)
  {
    tx_buf[pos++] = controller.last_cmd ? controller.last_cmd : 'X';
  }
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), ",");
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), controller.reason ? controller.reason : "IDLE");
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), ",");
  buf_append_int(tx_buf, &pos, (int)sizeof(tx_buf), (int32_t)lroundf(controller.v * 100.0f));
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), ",");
  buf_append_tenths(tx_buf, &pos, (int)sizeof(tx_buf), controller.nominal_balance_angle);
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), ",");
  int32_t out_i = isfinite(controller.output) ? (int32_t)lroundf(controller.output) : 0;
  buf_append_int(tx_buf, &pos, (int)sizeof(tx_buf), out_i);
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), ",");
  buf_append_int(tx_buf, &pos, (int)sizeof(tx_buf), motor_sys.enc_left_total);
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), ",");
  buf_append_int(tx_buf, &pos, (int)sizeof(tx_buf), motor_sys.enc_right_total);
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), ",");
  const char *enc_st = !controller.encoder_enabled ? "DIS" :
                       (controller.enc_signal_active ? "ACT" : "WAIT");
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), enc_st);
  buf_append_str(tx_buf, &pos, (int)sizeof(tx_buf), "\r\n");
  tx_buf[pos] = '\0';

  tx_len = pos;
  tx_idx_u2 = 0;
  tx_idx_lp = 0;
  Motion_PollUART();
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* Khởi tạo mỗi ngoại vi đúng 1 lần duy nhất */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __HAL_DBGMCU_FREEZE_TIM6();

  (void)Motor_Init(&motor_sys, &htim1, &htim2, &htim3);
  BalanceController_Init(&controller, &imu, &motor_sys);

  /* Hiệu chuẩn nhanh MPU6050 và sẵn sàng cân bằng */
  SelfBalance_CalibrateAndArm(false);

  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
  (void)HAL_TIM_Base_Start(&htim6);
  last_cycles = DWT->CYCCNT;

  uint32_t b1_low_ms = 0U;
  uint32_t last_telem_ms = HAL_GetTick();

  while (1)
  {
    Motion_PollUART();

    if (recalib_requested)
    {
      recalib_requested = false;
      if (!controller.armed || controller.fall_waiting || controller.fault_latched)
      {
        SelfBalance_CalibrateAndArm(false);
      }
    }

    /* 1. Vòng điều khiển 200 Hz: chỉ kích bởi cờ phần cứng TIM6 (không trộn HAL_GetTick) */
    if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

      uint32_t now_cycles = DWT->CYCCNT;
      float dt = last_cycles ? ((float)(now_cycles - last_cycles) / (float)SystemCoreClock) : 0.005f;
      last_cycles = now_cycles;

      if (isfinite(dt) && dt >= 0.001f)
      {
        if (dt > 0.030f) dt = 0.030f;

        if (motor_sys.enc_ready)
        {
          Motor_ReadEncoders(&motor_sys);
        }

        if (imu.ready)
        {
          if (MPU6050_UpdateAngle(&imu, dt))
          {
            /* Chỉ điều khiển khi đang ARM và KHÔNG bị chốt lỗi an toàn */
            if (controller.armed && !controller.fault_latched)
            {
              (void)BalanceController_Update(&controller, dt);
            }
          }
          else if (controller.armed)
          {
            BalanceController_EmergencyStop(&controller, "I2C_ERR");
          }
        }
      }

      /* 2. Gửi phản hồi trạng thái sang ESP32-C6 mỗi 120ms (non-blocking) */
      if (HAL_GetTick() - last_telem_ms >= 120U)
      {
        last_telem_ms = HAL_GetTick();
        Motion_SendTelemetry();
      }
    }

    /* 3. Nút nhấn xanh B1 (PC13): cắt motor NGAY LẬP TỨC trước khi chờ nhả nút */
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
    {
      if (b1_low_ms == 0U)
      {
        b1_low_ms = HAL_GetTick();
      }
      else if (HAL_GetTick() - b1_low_ms > 50U)
      {
        BalanceController_EmergencyStop(&controller, "B1_CALIB");
        while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
        {
          HAL_Delay(10);
        }
        b1_low_ms = 0U;
        SelfBalance_CalibrateAndArm(true);
      }
    }
    else
    {
      b1_low_ms = 0U;
    }

    /* 4. Đèn xanh LD2 (PA5):
     * - Nháy rất nhanh (40ms): Đang nhận lệnh di chuyển W/A/S/D từ ESP32-C6
     * - Sáng đứng: Xe đang đứng cân bằng (BALANCE)
     * - Nháy 200ms: Đang chờ dựng thẳng <= 18° (FALL_WAIT)
     * - Tắt: Lỗi / Đang chốt an toàn (bấm B1 để mở lại) */
    bool cmd_active = (controller.last_cmd == 'W' || controller.last_cmd == 'S' ||
                       controller.last_cmd == 'A' || controller.last_cmd == 'D') ||
                      (last_cmd_blink_ms != 0U && (HAL_GetTick() - last_cmd_blink_ms < 120U));
    if (cmd_active)
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ((HAL_GetTick() / 40U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    else if (controller.armed && !controller.fault_latched)
    {
      if (controller.fall_waiting)
      {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ((HAL_GetTick() / 200U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
      else
      {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      }
    }
    else
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) Error_Handler();

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10425F73;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 4) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();

  USART2->CR1 |= USART_CR1_RXNEIE;
}

static void MX_LPUART1_UART_Init(void)
{
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK) Error_Handler();

  LPUART1->CR1 |= USART_CR1_RXNEIE;
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, LD2_Pin | BIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, AIN1_Pin | AIN2_Pin | BIN1_Pin | RESERVED_LOW_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STBY_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin | BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = AIN1_Pin | AIN2_Pin | BIN1_Pin | RESERVED_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
  __disable_irq();
  while (1) { }
}
