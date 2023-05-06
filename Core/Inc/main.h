/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include <stdlib.h>
#include "qfplib-m3.h"
#include <math.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
float measure_calc_routine_duration();
void Read_Command();
void Set_DIR(uint8_t);
uint32_t Get_dt_us();
void MPU6050_Read();
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
volatile enum task {rise_up, equilibrium, test};
enum task state;
volatile enum onoff {on, off};
enum onoff run;
int counter, counter2;

// Data from MPU6050 Library
float accelxy_div, accelz_div, gyro_div;
float AccX, AccY, AccZ, GyX, GyY, GyZ;
float accPitch, accRoll, gyroAngleDelta, gyroAngle;
float AccY2, AccZ2, denominator, numerator, aux;

float targetAngle, filteredAngle;

uint32_t startTime, readingTime, prevTime, dt_us, elapsedTime;
float dt, calc_routine_duration;

float kp;
float ki;
float kd;

float error, dError, sumError;
float lastError;
float pid, control, control_mapped;

int is_angle_positive;
float CONTROL_ANGLE_LIMIT;
float END_RISE_UP_ANGLE;
float DITHERING_RATE;
float ANGLE_FILTER;
float GYRO_FIXRATE;
float DELAY_TIME;
float GYRO_ANGLE_DELTA_LIMIT;

uint32_t duty; // goes into CCR Register

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENA_Pin GPIO_PIN_0
#define ENA_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_1
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_2
#define IN2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

//Constants
#define DUTY_MAX 1999 //TIM2_ARR
#define RISE_UP_DUTY 1900
#define riseup_time1 3000000
#define riseup_time2 4000000

#define ICF 0.025 //integral clamping factor
#define PI 3.1416

//MPU6050 OFFSETS
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
