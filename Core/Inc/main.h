/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

//#define DC_Pin GPIO_PIN_0
//#define DC_GPIO_Port GPIOB
//#define RESET_Pin GPIO_PIN_1
//#define RESET_GPIO_Port GPIOC
extern SPI_HandleTypeDef hspi1;  // SPI1 настроен через CubeMX
extern SPI_HandleTypeDef hspi2;  // SPI1 настроен через CubeMX

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define DCDC_DT     (1.0f / 10000.0f)
#define DCDC_VIN    12.0f
#define DCDC_RLOAD  10.0f
#define DCDC_C      470000e-6f

typedef struct {
    float Kp;       // Пропорциональный коэффициент
    float Ki;       // Интегральный коэффициент
    float integral; // Накопленная интегральная ошибка
    float out_min;  // Минимальный выход
    float out_max;  // Максимальный выход
} PI_Controller;

// Внешние переменные
//extern float Vout;
//extern float Vref;
//extern float Device_temp;
//extern float duty_last;
//extern PI_Controller pi;

// Прототипы функций
void PI_Init(PI_Controller* pi, float Kp, float Ki, float out_min, float out_max);
float PI_Update(PI_Controller* pi, float setpoint, float measurement, float dt);

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_Pin GPIO_PIN_2
#define CS_GPIO_Port GPIOC
#define DC_Pin GPIO_PIN_3
#define DC_GPIO_Port GPIOC
#define RESET_Pin GPIO_PIN_0
#define RESET_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_8
#define SPI2_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
