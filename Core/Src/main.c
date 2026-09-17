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
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MAX 3599.0f
#define INTEGRAL_OUT_MAX 100.0f
#define ENCODER_COUNTS_PER_TURN 1456.0f
#define SPEED_FILTER_ALPHA 0.3f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t receiveData;
volatile uint8_t uart_event = 0;

char commandBuffer[32];
uint8_t commandIndex = 0;

char controlMode = 0;
float targetValue = 0.0f;
volatile uint8_t control_event = 0;
uint32_t controlCycleCount = 0;

uint16_t encoderCurrent = 0, encoderPrevious = 0;
int16_t encoderDelta = 0;
int32_t encoderTotal = 0;
float rawSpeedRPM = 0.0f;
float actualSpeedRPM = 0.0f, relativeTurns = 0.0f;

char vofaBuffer[64];
int vofaLength = 0;

float Target, Actual, Out;
float Kp = 0.45f, Ki = 0.055f, Kd = 0.1f;
float Error0, Error1, ErrorInt = 0.0f;

/* 位置环以编码器计数为误差单位，参数从0开始调节 */
float positionKp = 0.10f, positionKi = 0.003f, positionKd = 0.15f;
float positionError0 = 0.0f, positionError1 = 0.0f, positionErrorInt = 0.0f;

volatile int32_t knobCount = 0;
volatile uint8_t tuneMode = 0; // 0表示比例参数，1表示积分参数

int32_t lastKnobCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Motor_SetOutput(float output);

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &receiveData, 1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (knobCount != lastKnobCount)
    {
      int32_t knobDelta = knobCount - lastKnobCount;
      lastKnobCount = knobCount;

      if (controlMode == 'P')
      {
        if (tuneMode == 0)
        {
          /* 位置模式：每格调整positionKp 0.001 */
          positionKp += (float)knobDelta * 0.001f;

          if (positionKp < 0.0f)
          {
            positionKp = 0.0f;
          }
          else if (positionKp > 0.2f)
          {
            positionKp = 0.2f;
          }
        }
        else
        {
          /* 位置模式：每格调整positionKi 0.0005 */
          positionKi += (float)knobDelta * 0.0005f;

          if (positionKi < 0.0f)
          {
            positionKi = 0.0f;
          }
          else if (positionKi > 0.1f)
          {
            positionKi = 0.1f;
          }
        }
      }
      else
      {
        if (tuneMode == 0)
        {
          /* 速度模式：每格调整Kp 0.01 */
          Kp += (float)knobDelta * 0.01f;

          if (Kp < 0.0f)
          {
            Kp = 0.0f;
          }
          else if (Kp > 5.0f)
          {
            Kp = 5.0f;
          }
        }
        else
        {
          /* 速度模式：每格调整Ki 0.001 */
          Ki += (float)knobDelta * 0.001f;

          if (Ki < 0.0f)
          {
            Ki = 0.0f;
          }
          else if (Ki > 1.0f)
          {
            Ki = 1.0f;
          }
        }
      }
    }

    // 判断中断接受是否发生
    if (uart_event == 1)
    {
      // 判断是速度还是位置指令
      if (commandBuffer[0] == 'S' && commandBuffer[1] == ',')
      {
        controlMode = 'S';
        // 将字符串转换为浮点数，从第三个字符开始
        targetValue = (float)atof(&commandBuffer[2]);
        ErrorInt = 0.0f;
        Error0 = 0.0f;
        Error1 = 0.0f;
      }
      else if (commandBuffer[0] == 'P' && commandBuffer[1] == ',')
      {
        controlMode = 'P';
        targetValue = (float)atof(&commandBuffer[2]);
        positionError0 = targetValue * ENCODER_COUNTS_PER_TURN - (float)encoderTotal;
        positionError1 = positionError0;
        positionErrorInt = 0.0f;
      }
      uart_event = 0;
    }

    // 判断计时器中断是否发生
    if (control_event == 1)
    {
      control_event = 0;
      // 读取编码器数值
      encoderCurrent = __HAL_TIM_GET_COUNTER(&htim2);
      // 计算编码器差值（用来计算速度）
      encoderDelta = (int16_t)(encoderCurrent - encoderPrevious);
      encoderPrevious = encoderCurrent;
      // 计算编码器总数值（用来计算位置）
      encoderTotal += encoderDelta;
      // 计算实际速度和相对转数
      // 13个脉冲/圈 × 28减速比 × 4倍频，40ms = 1/1500分钟
      rawSpeedRPM = (float)encoderDelta * 1500.0f / ENCODER_COUNTS_PER_TURN;

      /* 一阶低通滤波：30%采用本次数据，70%保留上次结果 */
      actualSpeedRPM += SPEED_FILTER_ALPHA * (rawSpeedRPM - actualSpeedRPM);

      relativeTurns = (float)encoderTotal / ENCODER_COUNTS_PER_TURN;

      // 以下为速度PI算法
      if (controlMode == 'S')
      {
        if(fabsf(Target)<115.0f)
        {
          Kp = 0.20f;
		  Ki = 0.025f;
          
        }
		else
		{
			Kp = 0.45f;
			Ki = 0.055f;
		}
        Target = targetValue;
        Actual = actualSpeedRPM;

        /* 目标速度为0时立即清除积分并停止电机 */
        if (Target == 0.0f)
        {
          Error0 = 0.0f;
          Error1 = 0.0f;
          ErrorInt = 0.0f;
          Out = 0.0f;
          Motor_SetOutput(0.0f);
        }
        else
        {
          Error1 = Error0;
          Error0 = Target - Actual;

          /* Ki不为0时才进行误差积分 */
          if (Ki != 0.0f)
          {
            ErrorInt += Error0;

            /* 限制积分项 Ki * ErrorInt 在-100～100之间 */
            if (ErrorInt > INTEGRAL_OUT_MAX / Ki)
            {
              ErrorInt = INTEGRAL_OUT_MAX / Ki;
            }
            else if (ErrorInt < -INTEGRAL_OUT_MAX / Ki)
            {
              ErrorInt = -INTEGRAL_OUT_MAX / Ki;
            }
          }
          else
          {
            ErrorInt = 0.0f;
          }

         if (Ki != 0.0f && fabsf(Error0) < 100.0f)
        {
          ErrorInt += Error0;
        }
        else
        {
          ErrorInt = 0.0f;
        }

          /* 位置式PI */
          Out = Kp * Error0 + Ki * ErrorInt + Kd * (Error0 - Error1);

          /* 输出限制为-100～100 */
          if (Out > 100.0f)
          {
            Out = 100.0f;
          }
          else if (Out < -100.0f)
          {
            Out = -100.0f;
          }

          Motor_SetOutput(Out);
        }
      }
      // 以下为位置PD算法
      else if (controlMode == 'P')
      {
        /* VOFA仍以圈为单位显示目标位置和实际位置 */
        Target = targetValue;
        Actual = relativeTurns;

        /* 保存上一次误差，以编码器计数为单位计算本次误差 */
        positionError1 = positionError0;
        positionError0 = Target * ENCODER_COUNTS_PER_TURN - (float)encoderTotal;
        /* 仅在误差小于100个计数且Ki不为0时积分 */
        if (positionKi != 0.0f && fabsf(positionError0) < 100.0f)
        {
          positionErrorInt += positionError0;
        }
        else
        {
          positionErrorInt = 0.0f;
        }

        /* 带积分分离的位置式PID */
        Out = positionKp * positionError0 + positionKi * positionErrorInt +
              positionKd * (positionError0 - positionError1);

        /* 将位置环输出限制为-100～100 */
        if (Out > 100.0f)
        {
          Out = 100.0f;
        }
        else if (Out < -100.0f)
        {
          Out = -100.0f;
        }

        /* Out的正负控制方向，绝对值控制PWM占空比 */
        Motor_SetOutput(Out);
      }

      controlCycleCount++;
      if (controlMode == 'P')
      {
        /* 位置模式：最后两路显示positionKp和positionKi */
        vofaLength = sprintf(vofaBuffer, "%.2f,%.2f,%.2f,%.4f,%.4f\n", Target, Actual, Out,
                             positionKp, positionKi);
      }
      else
      {
        /* 速度模式：最后两路显示Kp和Ki */
        vofaLength = sprintf(vofaBuffer, "%.2f,%.2f,%.2f,%.2f,%.3f\n", Target, Actual, Out, Kp, Ki);
      }
      HAL_UART_Transmit(&huart1, (uint8_t *)vofaBuffer, (uint16_t)vofaLength, 10);
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
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
void Motor_SetOutput(float output)
{
  uint32_t compareValue;

  /* 将PID输出限制在-100～100 */
  if (output > 100.0f)
  {
    output = 100.0f;
  }
  else if (output < -100.0f)
  {
    output = -100.0f;
  }

  /*
   * 每次设置方向前先关闭PWM。
   * 这样可以避免电机正在输出时直接切换PH方向。
   */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

  if (output > 0.0f)
  {
    /* 暂时规定PH低电平为正转 */
    HAL_GPIO_WritePin(MOTOR_PH_GPIO_Port, MOTOR_PH_Pin, GPIO_PIN_RESET);
  }
  else if (output < 0.0f)
  {
    /* PH高电平为反转 */
    HAL_GPIO_WritePin(MOTOR_PH_GPIO_Port, MOTOR_PH_Pin, GPIO_PIN_SET);

    /* PWM只需要输出大小，所以把负数变成正数 */
    output = -output;
  }
  else
  {
    /* output等于0时，CCR保持为0，电机停止 */
    return;
  }

  /* 将0～100映射为0～3599 */
  compareValue = (uint32_t)(output * PWM_MAX / 100.0f);

  /* 将换算后的比较值写入TIM1通道1 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compareValue);
}
// 接受数据中断函数重定义
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 判断是否为USART1的中断（实则只设置了usart1）
  if (huart->Instance == USART1)
  {
    // 判断接收到的数据是否为回车或换行符
    if (receiveData != '\r' && receiveData != '\n')
    {
      // 判断是否超出范围，如果没有则写入
      if (commandIndex < 31)
      {
        commandBuffer[commandIndex] = receiveData;
        commandIndex++;
      }
    }
    // 如果接收到回车符，则将命令字符串结束符置为'\0'，并将命令索引重置为0，表示接收完成
    else if (receiveData == '\n')
    {
      commandBuffer[commandIndex] = '\0';
      commandIndex = 0;
      uart_event = 1;
    }
    // 重新启动中断接受
    HAL_UART_Receive_IT(&huart1, &receiveData, 1);
  }
}
// 定时器计数中断的重定义（40ms一次）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    control_event = 1;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t encoderLastTick = 0;
  static uint32_t keyLastTick = 0;
  uint32_t nowTick = HAL_GetTick();

  // 旋转编码器A相产生下降沿
  if (GPIO_Pin == ENC_A_Pin)
  {
    // 简单消抖
    if (nowTick - encoderLastTick >= 5)
    {
      encoderLastTick = nowTick;

      // 读取B相判断方向
      if (HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin) == GPIO_PIN_SET)
      {
        knobCount++;
      }
      else
      {
        knobCount--;
      }
    }
  }
  // 按下旋钮
  else if (GPIO_Pin == ENC_KEY_Pin)
  {
    // 按键消抖
    if (nowTick - keyLastTick >= 200)
    {
      keyLastTick = nowTick;
      tuneMode ^= 1;
    }
  }
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
