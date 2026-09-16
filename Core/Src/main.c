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
uint8_t receiveData;
volatile uint8_t uart_event = 0;
char commandBuffer[32];
uint8_t commandIndex = 0;
char controlMode = 0;
int32_t targetValue = 0;
volatile uint8_t control_event = 0;
uint32_t controlCycleCount = 0;
uint16_t encoderCurrent = 0;
uint16_t encoderPrevious = 0;
int16_t encoderDelta = 0;
int32_t encoderTotal = 0;
float actualSpeedRPM = 0.0f;
float relativeTurns = 0.0f;
char vofaBuffer[64];
int vofaLength = 0;
float Target, Actual, Out;			
float Kp = 15.5f, Ki=0.3f, Kd=0.0f;					
float Error0, Error1, ErrorInt=0.0f;

float positionKp = 3000.0f, positionKi = 0.0f, positionKd = 710.0f;
float positionError0, positionError1, positionErrorInt = 0.0f;

volatile int32_t knobCount = 0;
volatile uint8_t tuneMode = 0;  // 0表示Kp，1表示Kd

int32_t lastKnobCount = 0;





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
  HAL_UART_Receive_IT(&huart1,&receiveData,1);
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

    if (tuneMode == 0)
    {
        // 每格调整Kp 0.5
        Kp += (float)knobDelta * 0.5f;

        if (Kp < 0.0f)
            Kp = 0.0f;
        else if (Kp > 50.0f)
            Kp = 50.0f;
    }
    else
    {
        // 每格调整Ki 0.1
        Ki += (float)knobDelta * 0.1f;

        if (Ki < 0.0f)
            Ki = 0.0f;
        else if (Ki > 20.0f)
            Ki = 20.0f;
    }
}
    //判断中断接受是否发生
    if(uart_event == 1)
    {
      //判断是速度还是位置指令
      if(commandBuffer[0] == 'S' && commandBuffer[1] == ',')
      {
        controlMode = 'S';
        //将字符串转换为整数，从第三个字符开始
        targetValue = atoi(&commandBuffer[2]);
        ErrorInt = 0.0f;
        Error0 = 0.0f;
        Error1 = 0.0f;
        
      }
      else if(commandBuffer[0] == 'P' && commandBuffer[1] == ',')
      {
        controlMode = 'P';
        targetValue = atoi(&commandBuffer[2]);
        positionError0 = (float)targetValue - relativeTurns;
        positionError1 = positionError0;
        positionErrorInt = 0.0f;
      }
      uart_event = 0;
      }

      //判断计时器中断是否发生
      if(control_event == 1)
      {
        control_event = 0;
        //读取编码器数值
        encoderCurrent = __HAL_TIM_GET_COUNTER(&htim2);
        //计算编码器差值（用来计算速度）
        encoderDelta = (int16_t)(encoderCurrent - encoderPrevious);
        encoderPrevious = encoderCurrent;
        //计算编码器总数值（用来计算位置）
        encoderTotal += encoderDelta; 
        //计算实际速度和相对转数
        //13个脉冲/圈 × 28减速比 × 4倍频，80ms = 1/750分钟
        actualSpeedRPM = (float)encoderDelta * 750.0f / 1456.0f;
        relativeTurns = (float)encoderTotal / 1456.0f;
        //以下为速度PID算法
        if (controlMode == 'S')
        {
          Target = (float)targetValue;
          Actual = actualSpeedRPM;
          Error1 = Error0;
          Error0 = Target - Actual;
          ErrorInt += Error0;
          Out = Kp * Error0 + Ki * ErrorInt;
        
        if (Out > 3599.0f)
        {
         Out = 3599.0f;
        }
        else if (Out < 0.0f)
        {
         Out = 0.0f;
        }
         __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)Out);
      }
        //以下为位置PID算法
        else if (controlMode == 'P')
        {
          Target = (float)targetValue;
          Actual = relativeTurns;
          positionError1 = positionError0;
          positionError0 = Target - Actual;
		  positionErrorInt += positionError0;
          Out = positionKp * positionError0 + positionKi * positionErrorInt+ positionKd * (positionError0 - positionError1);
          if (Out > 3599.0f)
        {
          Out = 3599.0f;
        }
           else if (Out < 0.0f)
          {
             Out = 0.0f;
          }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)Out);
        
        }

        controlCycleCount++;
        vofaLength = sprintf(vofaBuffer,
                     "%.2f,%.2f,%.2f,%.2f,%.2f\n",
                     Target, Actual, Out, Kp, Ki);
        HAL_UART_Transmit(&huart1,(uint8_t *)vofaBuffer,(uint16_t)vofaLength,10);
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
//接受数据中断函数重定义
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //判断是否为USART1的中断（实则只设置了usart1）
  if(huart->Instance == USART1)
  {
    //判断接收到的数据是否为回车或换行符
    if (receiveData != '\r' && receiveData != '\n')
    {
      //判断是否超出范围，如果没有则写入
      if (commandIndex < 31)
      {
        commandBuffer[commandIndex] = receiveData;
        commandIndex++;
      }

    }
    //如果接收到回车符，则将命令字符串结束符置为'\0'，并将命令索引重置为0，表示接收完成
    else if (receiveData == '\n')
    {
      commandBuffer[commandIndex] = '\0'; 
      commandIndex = 0; 
    uart_event = 1;
    }
    //重新启动中断接受
    HAL_UART_Receive_IT(&huart1,&receiveData,1);

  }

}
//定时器计数中断的重定义（10ms一次）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3)
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
