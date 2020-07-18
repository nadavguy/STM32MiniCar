/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum { false, true } bool;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
extern char USBTXArray[1024];
extern uint8_t UART5RXArray[128];

extern bool isNewMagDataAvailable;

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
extern void aaa(void);
extern double CurrentTime(void);



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Flash_Reset_Pin GPIO_PIN_1
#define Flash_Reset_GPIO_Port GPIOA
#define USB_TX_Pin GPIO_PIN_2
#define USB_TX_GPIO_Port GPIOA
#define USB_RX_Pin GPIO_PIN_3
#define USB_RX_GPIO_Port GPIOA
#define MS5611_CS_Pin GPIO_PIN_4
#define MS5611_CS_GPIO_Port GPIOA
#define PWM1_J1_P1_Pin GPIO_PIN_5
#define PWM1_J1_P1_GPIO_Port GPIOA
#define Green_LED_Pin GPIO_PIN_12
#define Green_LED_GPIO_Port GPIOD
#define Red_LED_Pin GPIO_PIN_13
#define Red_LED_GPIO_Port GPIOD
#define Blue_LED_Pin GPIO_PIN_15
#define Blue_LED_GPIO_Port GPIOD
#define PWM3_J5_P1_Pin GPIO_PIN_8
#define PWM3_J5_P1_GPIO_Port GPIOA
#define PWM4_J5_P4_Pin GPIO_PIN_10
#define PWM4_J5_P4_GPIO_Port GPIOA
#define PWM2_J1_P4_Pin GPIO_PIN_11
#define PWM2_J1_P4_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
extern unsigned int sysTickCounter;
extern s32 comres;
extern u8 power_mode;
extern s16 accel_datax;
extern s16 accel_datay;
extern s16 accel_dataz;
extern double d_euler_data_h;
extern double d_euler_data_r;
extern double d_euler_data_p;
extern struct bno055_gravity_double_t d_gravity_xyz;
extern struct bno055_mag_double_t d_mag_xyz;
extern struct bno055_euler_double_t d_euler_hpr;
extern double Roll;
extern double Pitch;
extern double Yaw;

extern QSPI_HandleTypeDef hqspi;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
