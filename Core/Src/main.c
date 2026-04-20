/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "arm_math.h"
#include "micros.h"
#include "motor_driver.h"
#include "encoder.h"
#include "motor_pid.h"
#include "uart_comm.h"
#include "cmd_parser.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_MOTORS 4
#define PER_REV 1940
#define MAX_RPM 160
#define KP 3.0f
#define KI 0.5f
#define KD 0.2f
#define ALPHA 0.5f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
motor_t motors[N_MOTORS] = {
  { .htim = &htim8, .channel = TIM_CHANNEL_1, .dir_port = M1_DIR_GPIO_Port, .dir_pin = M1_DIR_Pin, .max_pwm = TIM8_ARR },
  { .htim = &htim8, .channel = TIM_CHANNEL_2, .dir_port = M2_DIR_GPIO_Port, .dir_pin = M2_DIR_Pin, .max_pwm = TIM8_ARR },
  { .htim = &htim8, .channel = TIM_CHANNEL_3, .dir_port = M3_DIR_GPIO_Port, .dir_pin = M3_DIR_Pin, .max_pwm = TIM8_ARR },
  { .htim = &htim8, .channel = TIM_CHANNEL_4, .dir_port = M4_DIR_GPIO_Port, .dir_pin = M4_DIR_Pin, .max_pwm = TIM8_ARR }
};

encoder_t encoders[N_MOTORS] = {
  { .htim = &htim1, .per_rev = PER_REV },
  { .htim = &htim2, .per_rev = PER_REV },
  { .htim = &htim3, .per_rev = PER_REV },
  { .htim = &htim4, .per_rev = PER_REV }
};

motor_pid_t motor_pids[N_MOTORS] = {
  { .motor = &motors[0], .encoder = &encoders[0], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA },
  { .motor = &motors[1], .encoder = &encoders[1], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA },
  { .motor = &motors[2], .encoder = &encoders[2], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA },
  { .motor = &motors[3], .encoder = &encoders[3], .max_rpm = MAX_RPM, .kp = KP, .ki = KI, .kd = KD, .alpha = ALPHA }
};

RotationDirection_t dirs[N_MOTORS] = { ROTATION_CCW, ROTATION_CCW, ROTATION_CCW, ROTATION_CCW };
int8_t motor_sign[N_MOTORS] = { 1, -1, 1, -1};
float poses[N_MOTORS];
float32_t set_speed[N_MOTORS];
uint32_t time, time2;
char rx_buff[100], out[128];
Command cmd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
    uart_rx_byte_callback();
}

void execute_command(const Command *cmd)
{
  CmdType type = cmd->cmd;
  switch (type)
  {
  case CMD_GET_POS:
  {
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      poses[i] = encoder_get_rotations(&encoders[i]) * (float)motor_sign[i];
    }
    sprintf(out, "CMD_GET_POS;%.3f;%.3f;%.3f;%.3f\r", poses[0], poses[1], poses[2], poses[3]);
    break;
  }
  case CMD_SET_SPEED:
  {
    sprintf(out, "CMD_SET_SPEED;%hd;%hd;%hd;%hd\r", cmd->speeds[0], cmd->speeds[1], cmd->speeds[2], cmd->speeds[3]);
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      int16_t dir = cmd->speeds[i] * motor_sign[i];
      dirs[i] = (dir >= 0) ? ROTATION_CCW : ROTATION_CW;
      set_speed[i] = fabsf(cmd->speeds[i]);
    }
    break;
  }
  case CMD_SET_PID:
  {
    sprintf(out, "CMD_SET_PID;%.2f;%.2f;%.2f;\r", cmd->set_pid.kp, cmd->set_pid.ki, cmd->set_pid.kd);
    for (uint8_t i = 0; i < N_MOTORS; i++)
    {
      motor_pid_set_pid(&motor_pids[i], cmd->set_pid.kp, cmd->set_pid.ki, cmd->set_pid.kd);
    }
    break;
  }
  default:
    break;
  }
  uart_send_str(out);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  micros_tim_init(&htim5);
  uart_init(&huart2);

  for (uint8_t i = 0; i < N_MOTORS; i++)
  {
    motor_init(&motors[i]);
    encoder_init(&encoders[i]);
    motor_pid_init(&motor_pids[i]);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  time = micros();
  time2 = micros();
  while (1)
  {
    if (micros() - time > 10000)
    {
      if (uart_is_line())
      {
        uart_get_line(rx_buff, sizeof(rx_buff));
        if (parse_command(rx_buff, &cmd))
        {
          execute_command(&cmd);
        }
        else
        {
          uart_send_str("ERR\r");
        }
      }
      time = micros();
    }

    if (micros() - time2 > 10000)
    {
      for (uint8_t i = 0; i < N_MOTORS; i++)
      {
        //motor_pid_update(&motor_pids[i], set_speed[i], dirs[i]);
        for (uint8_t i = 0; i < N_MOTORS; i++)
        {
          poses[i] = encoder_get_rotations(&encoders[i]) * (float)motor_sign[i];
        }
        sprintf(out, "CMD_GET_POS;%.3f;%.3f;%.3f;%.3f\r\n", poses[0], poses[1], poses[2], poses[3]);
        printf(out);
      }
      time2 = micros();
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
