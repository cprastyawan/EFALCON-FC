/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	uint32_t Roll;
	uint32_t Pitch;
	uint32_t Throttle;
	uint32_t Yaw;
	uint32_t SWC;
	uint32_t SWB;
} RC_t;

typedef struct {
	float x;
	float y;
}Point_t;

typedef struct {
	float xV;
	float yV;
}Velocity_t;

typedef enum {
	Front = 0,
	Back = 1,
	Left = 2,
	Right = 3,
	Top = 4,
	Bottom = 5
}Orientation;

typedef struct {
	float error;
	float preverror;

	float derivative;
	float sumIntegral;
	float setPoint;

	float output;

	float kp;
	float kd;
	float ki;

	float timesampling;
} PIDType_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define map(x, in_min, in_max, out_min, out_max) (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
#define constrain(x, min, max) (x < min ? min : x > max ? max : x)
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define US_TRIG_Pin GPIO_PIN_2
#define US_TRIG_GPIO_Port GPIOE
#define US_ECHO_Pin GPIO_PIN_5
#define US_ECHO_GPIO_Port GPIOE
#define LORA_RST_Pin GPIO_PIN_6
#define LORA_RST_GPIO_Port GPIOE
#define LORA_NSS_Pin GPIO_PIN_13
#define LORA_NSS_GPIO_Port GPIOC
#define ADC_ANGIN2_Pin GPIO_PIN_0
#define ADC_ANGIN2_GPIO_Port GPIOC
#define ADC_BATERAI3_Pin GPIO_PIN_1
#define ADC_BATERAI3_GPIO_Port GPIOC
#define LORA_MISO_Pin GPIO_PIN_2
#define LORA_MISO_GPIO_Port GPIOC
#define LORA_MOSI_Pin GPIO_PIN_3
#define LORA_MOSI_GPIO_Port GPIOC
#define SBUS_RX_Pin GPIO_PIN_0
#define SBUS_RX_GPIO_Port GPIOA
#define PWM_OUT5_Pin GPIO_PIN_1
#define PWM_OUT5_GPIO_Port GPIOA
#define PWM_OUT6_Pin GPIO_PIN_2
#define PWM_OUT6_GPIO_Port GPIOA
#define PWM_OUT7_Pin GPIO_PIN_3
#define PWM_OUT7_GPIO_Port GPIOA
#define ADC_BATERAI2_Pin GPIO_PIN_4
#define ADC_BATERAI2_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define ADC_ANGIN1_Pin GPIO_PIN_4
#define ADC_ANGIN1_GPIO_Port GPIOC
#define IMU_CS_Pin GPIO_PIN_5
#define IMU_CS_GPIO_Port GPIOC
#define REMOTE_CH3_Pin GPIO_PIN_0
#define REMOTE_CH3_GPIO_Port GPIOB
#define ADC_BATERAI1_Pin GPIO_PIN_1
#define ADC_BATERAI1_GPIO_Port GPIOB
#define BMP_CS_Pin GPIO_PIN_7
#define BMP_CS_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOE
#define PWM_OUT3_Pin GPIO_PIN_10
#define PWM_OUT3_GPIO_Port GPIOB
#define PWM_OUT4_Pin GPIO_PIN_11
#define PWM_OUT4_GPIO_Port GPIOB
#define LORA_DIO1_Pin GPIO_PIN_12
#define LORA_DIO1_GPIO_Port GPIOB
#define REMOTE_PPM_Pin GPIO_PIN_14
#define REMOTE_PPM_GPIO_Port GPIOB
#define LORA_DIO2_Pin GPIO_PIN_10
#define LORA_DIO2_GPIO_Port GPIOD
#define LORA_DIO3_Pin GPIO_PIN_11
#define LORA_DIO3_GPIO_Port GPIOD
#define REMOTE_CH4_Pin GPIO_PIN_12
#define REMOTE_CH4_GPIO_Port GPIOD
#define REMOTE_CH5_Pin GPIO_PIN_13
#define REMOTE_CH5_GPIO_Port GPIOD
#define REMOTE_CH6_Pin GPIO_PIN_14
#define REMOTE_CH6_GPIO_Port GPIOD
#define GPS_TX_Pin GPIO_PIN_9
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_10
#define GPS_RX_GPIO_Port GPIOA
#define GPS_INT_Pin GPIO_PIN_11
#define GPS_INT_GPIO_Port GPIOA
#define PWM_OUT1_Pin GPIO_PIN_15
#define PWM_OUT1_GPIO_Port GPIOA
#define PWM_OUT2_Pin GPIO_PIN_3
#define PWM_OUT2_GPIO_Port GPIOB
#define REMOTE_CH1_Pin GPIO_PIN_4
#define REMOTE_CH1_GPIO_Port GPIOB
#define REMOTE_CH2_Pin GPIO_PIN_5
#define REMOTE_CH2_GPIO_Port GPIOB
#define PWM_OUT8_Pin GPIO_PIN_8
#define PWM_OUT8_GPIO_Port GPIOB
#define PWM_BUZZER_Pin GPIO_PIN_9
#define PWM_BUZZER_GPIO_Port GPIOB
#define RTC_NC_Pin GPIO_PIN_1
#define RTC_NC_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
