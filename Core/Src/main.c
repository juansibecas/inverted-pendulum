/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* Var Init*/

  targetAngle = 0.0;
  lastError = 0.0;

  kp = 0;
  ki = 0;
  kd = 0;

  duty = 0;
  counter = 0;
  counter2 = 0;

  CONTROL_ANGLE_LIMIT = 10;
  END_RISE_UP_ANGLE = 4;
  DITHERING_RATE = 0;
  ANGLE_FILTER = 0.95; // 0.9-0.95 less filter delay - more noise
  GYRO_FIXRATE = 0.5;
  GYRO_ANGLE_DELTA_LIMIT = 0.1;

  //taken from mpu6050.c
  accelxy_div = 16384.0;
  accelz_div = 14418.0;
  gyro_div = 131.0;

  //taken from previous calibration
  AccErrorX = 0.07;
  AccErrorY = -2.49;
  GyroErrorX = -4.79;
  GyroErrorY = 0.02;
  GyroErrorZ = -0.05;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  run = on;
  state = equilibrium;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  //START TIM1
  HAL_TIM_Base_Start(&htim3);

  while (MPU6050_Init(&hi2c1) == 1);

  //START UART
  HAL_UART_Receive_IT(&huart3, UART3_rxBuffer, 1);

  //START PWM
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Program Started\r\n");
  HAL_Delay(2000);
  elapsedTime = 0;
  startTime = 0;
  prevTime = 0;
  __HAL_TIM_SET_COUNTER(&htim3,0);  // Restart the counter value

  while (1){

	  // Read data
	  if (I2C1->SR1 != 0x0){ //BERR or ARLO Flagged0
		  printf("SR1 cleared\r\n");
		  I2C1->SR1 = 0x0;
	  }

	  MPU6050_Read();

	  // Calculating Pitch from accelerometer data

	  //qfp form
	  AccY2 = qfp_fmul(AccY, AccY);
	  AccZ2 = qfp_fmul(AccZ, AccZ);
	  denominator = qfp_fsqrt(qfp_fadd(AccY, AccZ));
	  numerator = qfp_fmul(-1, AccX);
	  aux = qfp_fatan2(numerator, denominator);
	  accPitch = qfp_fsub(qfp_fmul(aux, qfp_fdiv(180, PI)), AccErrorY);

	  //math.h form
	  //accPitch = (atan(-1 * AccX / sqrt(pow(AccY,2) + pow(AccZ,2))) * 180 / PI) - AccErrorY;

	  if(counter == 0){
		  if(accPitch > 0){
			  is_angle_positive = 1;
		  } else{
			  is_angle_positive = 0;
		  }
	  }
	  // Correct the outputs with the calculated error values
	  //qfp form
	  GyX = qfp_fsub(GyX, GyroErrorX);
	  GyY = qfp_fsub(GyY, GyroErrorY);
	  GyZ = qfp_fsub(GyZ, GyroErrorZ);

	  // COMPLEMENTARY FILTER
	  gyroAngleDelta = qfp_fmul(-1, GyZ);
	  gyroAngleDelta = qfp_fmul(gyroAngleDelta, dt);
	  gyroAngleDelta = qfp_fmul(gyroAngleDelta, GYRO_FIXRATE);


	  if(counter == 0){
		  gyroAngle = accPitch;
	  }

	  if(abs(gyroAngleDelta) > GYRO_ANGLE_DELTA_LIMIT){
		  gyroAngleDelta = 0;
	  }
	  gyroAngle = qfp_fadd(gyroAngle, gyroAngleDelta);

	  if(isnan(accPitch)) {
		  accPitch = 0;
	  }


	  if(counter == 0){
		  filteredAngle = accPitch;
	  } else{
		  filteredAngle = qfp_fmul(ANGLE_FILTER, qfp_fadd(filteredAngle, gyroAngleDelta));
		  aux = qfp_fsub(1, ANGLE_FILTER);
		  filteredAngle = qfp_fadd(filteredAngle,  qfp_fmul(aux, accPitch));
	  }

	  counter++;
	  switch(run){
	  case on:
		  // Time Measurement
		  dt_us = Get_dt_us();
		  elapsedTime += dt_us;
		  //divide by 1e6 from us to s
		  dt = qfp_fdiv(dt_us, 1000000);

		  switch(state){
		  case equilibrium:

			  //  Turn off if it falls off
			  //if(abs(filteredAngle) > CONTROL_ANGLE_LIMIT){
			  //	printf("Angle too large, turning off\r\n");
			  //	run = off;
			  //}

			  // DITHERING for more responsiveness
			  aux = qfp_fmul(DITHERING_RATE, dt);
			  if (filteredAngle < targetAngle){
				  targetAngle = qfp_fadd(targetAngle, aux);
			  } else{
				  targetAngle = qfp_fsub(targetAngle, aux);
			  }

			  //clamp dithering
			  if (targetAngle < -5) targetAngle = -5;
			  else if(targetAngle > 5) targetAngle = 5;

			  // PROPORTIONAL
			  error = qfp_fsub(targetAngle, filteredAngle);

			  // DERIVATIVE
			  dError = qfp_fdiv(qfp_fsub(error, lastError), dt);
			  lastError = error;
			  if(counter2 == 0){
				  dError = 0;
			  }
			  // INTEGRAL
			  sumError = qfp_fadd(sumError, qfp_fmul(error, dt));
			  // SATURATION for integral windup
			  if (sumError < -DUTY_MAX*ICF) sumError = -DUTY_MAX*ICF;
			  else if(sumError > DUTY_MAX*ICF) sumError = DUTY_MAX*ICF;

			  // CONTROL
			  pid = qfp_fadd(qfp_fadd(qfp_fmul(kp, error), qfp_fmul(kd, dError)), qfp_fmul(ki, sumError));
			  // SATURATION for duty cycle
			  // constrained by TIM2_ARR/DUTY_MAX
			  control = pid;
			  if (control < -DUTY_MAX) control = -DUTY_MAX;
			  else if(control > DUTY_MAX) control = DUTY_MAX;

			  //Set direction
			  if(control > 0.0){
				  Set_DIR(1);
			  } else{
				  Set_DIR(0);
			  }

			  duty = (uint32_t) abs(control); // first take the sign away and then cast to uint32_t

			  // ENA Pin - Timer 2 Channel 1
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);

			  counter2++;

			  break;

		  case rise_up: //3 seconds to start spinning and 1 more to switch dir
			  if (elapsedTime < riseup_time1){ // elapsed time in microseconds

				  if (is_angle_positive){
					  Set_DIR(1);
				  } else{
					  Set_DIR(0);
				  }
				  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, RISE_UP_DUTY);

			  } else if (elapsedTime < riseup_time2){

				  //  Switch direction
				  if (is_angle_positive){
					  Set_DIR(0);
				  } else{
					  Set_DIR(1);
				  }

				  //  End rise up sequence if it got up
				  if(abs(filteredAngle) < END_RISE_UP_ANGLE){
					  printf("Rise up success\r\n");
					  state = equilibrium;
				  }

			  } else{
				  printf("Rise up sequence failed, turning off\r\n");
				  run = off;
			  }

			  break;

		  case test:
			  //debug only
			  Set_DIR(1);
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, DUTY_MAX);
			  HAL_Delay(5000);
			  Set_DIR(0);
			  HAL_Delay(5000);
			  break;

		  default:
			  break;

		  }
		  break;
	  case off:
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		  //Blink
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  HAL_Delay(500);
		  printf("Waiting...\r\n");

		  //  Reset variables

		  targetAngle = 0.0;
		  lastError = 0.0;
		  dError = 0.0;
		  sumError = 0.0;

		  duty = 0;
		  counter = 0;
		  counter2 = 0;

		  elapsedTime = 0;
		  break;

	  default:
		  break;

	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

float measure_calc_routine_duration(){
	return (float)HAL_GetTick()/counter2;

}

void MPU6050_Read(){
	MPU6050_Read_Accel(&hi2c1, &MPU6050);
	MPU6050_Read_Gyro(&hi2c1, &MPU6050);

	//cast raw values to int32_t, make them fix16_t and divide by corrector value from mpu6050.c
	AccX = qfp_fdiv((float)MPU6050.Accel_X_RAW, accelxy_div);
	AccY = qfp_fdiv((float)MPU6050.Accel_Y_RAW, accelxy_div);
	AccZ = qfp_fdiv((float)MPU6050.Accel_Z_RAW, accelz_div);
	GyX = qfp_fdiv((float)MPU6050.Gyro_X_RAW, gyro_div);
	GyY = qfp_fdiv((float)MPU6050.Gyro_Y_RAW, gyro_div);
	GyZ = qfp_fdiv((float)MPU6050.Gyro_Z_RAW, gyro_div);

}

void Set_DIR(uint8_t dir){
	if(dir == 1){
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, SET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, RESET);
	} else{
		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, RESET);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, SET);
	}
}

uint32_t Get_dt_us(){
	uint32_t dt_us;
	dt_us = __HAL_TIM_GET_COUNTER(&htim3)*10;  // TIM3 clock 0.1MHz which is 10us per tick
	__HAL_TIM_SET_COUNTER(&htim3, 0);  // Restart the counter value
	return dt_us;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

