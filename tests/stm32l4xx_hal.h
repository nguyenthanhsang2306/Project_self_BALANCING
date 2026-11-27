#ifndef MOCK_HAL_H
#define MOCK_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK=0,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET=0, GPIO_PIN_SET=1 } GPIO_PinState;
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { uint32_t PSC,ARR,CR1,CCR1,CCR2,EGR,CCER,BDTR,CNT; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { uint32_t CFGR; } RCC_TypeDef;
typedef struct { union { uint32_t outputs; uint32_t ODR; }; } GPIO_TypeDef;
extern TIM_TypeDef mock_tim1;
extern RCC_TypeDef mock_rcc;
extern GPIO_TypeDef mock_a,mock_b,mock_c;
extern uint32_t SystemCoreClock;
#define TIM1 (&mock_tim1)
#define RCC (&mock_rcc)
#define GPIOA (&mock_a)
#define GPIOB (&mock_b)
#define GPIOC (&mock_c)
#define RCC_CFGR_PPRE2 0x3800U
#define TIM_CR1_DIR 0x10U
#define TIM_CR1_CMS 0x60U
#define TIM_EGR_UG 1U
#define TIM_CR1_CEN 1U
#define TIM_BDTR_MOE 0x8000U
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC2E 0x10U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_ALL 0xFFFFU
#define GPIO_PIN_0 1U
#define GPIO_PIN_4 16U
#define GPIO_PIN_5 32U
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
#define GPIO_PIN_8 256U
#define GPIO_PIN_9 512U
#define GPIO_PIN_10 1024U
#define GPIO_PIN_13 8192U
#define I2C_MEMADD_SIZE_8BIT 1U
#define __HAL_TIM_SET_COMPARE(t,c,v) ((c)==TIM_CHANNEL_1?((t)->Instance->CCR1=(v)):((t)->Instance->CCR2=(v)))
#define __HAL_TIM_GET_COUNTER(t) ((t)->Instance->CNT)
#define __HAL_TIM_MOE_ENABLE(t) ((t)->Instance->BDTR|=TIM_BDTR_MOE)
#define __DMB() ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *,uint16_t,uint32_t);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *,uint16_t);
void HAL_GPIO_TogglePin(GPIO_TypeDef *,uint16_t);
uint32_t HAL_RCC_GetPCLK2Freq(void);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *,uint16_t,uint16_t,uint16_t,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *,uint16_t,uint16_t,uint16_t,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
#endif
