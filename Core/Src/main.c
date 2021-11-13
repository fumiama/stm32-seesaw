/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GY_ACC 0x15
#define GY_RO  0x25
#define GY_MG  0x35
#define GY_EUR 0x45
#define GY_4P  0x65
#define GY_ACF 0x75
#define GY_RNG 0x85

#define GY_T_ACC "\xa5\x15\xba"
#define GY_T_RO  "\xa5\x25\xca"
#define GY_T_MG  "\xa5\x35\xda"
#define GY_T_EUR "\xa5\x45\xea"
#define GY_T_4P  "\xa5\x65\x0a"

#define GY_G_RO  "\xa5\xd5\x7a"
#define GY_G_MG  "\xa5\xe5\x8a"
#define GY_G_EUR "\xa5\x95\x3a"

#define GY_FIX_ACC "\xa5\x57\xfc"
#define GY_FIX_MG  "\xa5\x58\xfd"

#define GY_SPED_50 "\xa5\xa4\x49"

#define CMD_FRNT  '^'
#define CMD_BACK  'v'
#define CMD_LEFT  '<'
#define CMD_RGHT  '>'
#define CMD_STAT  '#'
#define CMD_STOP  '-'
#define CMD_LOCK  '+'
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t     USART1_SendBuff[USART1_SEND_LEN_MAX];
uint8_t     USART1_RecvBuff[USART1_RECV_LEN_MAX];
uint8_t     UserRecvBuff1[USART1_RECV_LEN_MAX];

uint8_t     USART2_SendBuff[USART2_SEND_LEN_MAX];
uint8_t     USART2_RecvBuff[USART2_RECV_LEN_MAX];
//uint8_t     UserRecvBuff2[USART2_RECV_LEN_MAX];

/**********GY953����ر���*********/
/*
int16_t   AccX       = 0;		//���ٶ�ֵ
int16_t   AccY       = 0;
int16_t   AccZ       = 0;

int16_t   GyroX      = 0;		//������ֵ
// y С�� 0 ˵��������ת���ĽǼ��ٶ�
int16_t   GyroY      = 0;
int16_t   GyroZ      = 0;

int16_t   MgX	    	 = 0;   //�شż�ֵ
int16_t   MgY        = 0;
int16_t   MgZ        = 0;

int16_t   Roll       = 0;		//ŷ����ֵ*100
*/
// pitch С�� 0 ˵��������б
int16_t   Pitch      = 0;
int16_t   Yaw        = 0;

BTSTAT bs;
int16_t speed1, speed2;
uint8_t isunstable = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Init_Eular(int16_t p, int16_t y);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //  ����TIM2���ж�ʹ��
  //  ����TIM1��ͨ��1��ͨ��4ΪPWM���
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  
  // UART1 ����1 ���������жϣ��浽USART1_RecvBuff�У���󳤶�Ϊ USART1_RECV_LEN_MAX
  // ���������� USART1_RECV_LEN_MAX ������֮�󣬲����ж�
  // �жϽ��ᴥ�� HAL_UART_RxCpltCallback ���ڽ����жϴ�����    
  HAL_UART_Receive_IT(&huart1, (uint8_t *)USART1_RecvBuff, USART1_RECV_LEN_MAX);
  // UART2 ����2 ͬ��
  HAL_UART_Receive_IT(&huart2, (uint8_t *)USART2_RecvBuff, USART2_RECV_LEN_MAX);

  MotoCtrl_SetValue(0, MOTOR_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    __WFI();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 399;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 500;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_IDC_GPIO_Port, LED_IDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_IDC_Pin */
  GPIO_InitStruct.Pin = LED_IDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_IDC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UKEY_Pin */
  GPIO_InitStruct.Pin = UKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(UKEY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
/* ���������������������  USER CODE BEGIN  �� USER CODE END ֮������û����򣡣�������������*/
/* ���������������������Ҳ�Ҫ�Ķ�   USER CODE BEGIN 4   ֮��ı�ǣ�������������������������*/
//
// MotoCtrl_SetValue Բ�ܶ����ת�ٿ������ú���
//
// value ת������ֵ��-200 ~ 200����Լ�� 0 ����ת��Ϊ 0
//       ��ת����ת
//       ������� -200 ~ 200 ��Χ������������
//
// motor ����ţ�MOTOR_1 �� MOTOR_2����main.h���ж���
//       MOTOR_1 �õ���htim1��TIM_CHANNEL_1���� PA8 �˿�
//       MOTOR_2 �õ���htim1��TIM_CHANNEL_4���� PA11 �˿�
static TIM_OC_InitTypeDef sConfig = {TIM_OCMODE_PWM1, 0, TIM_OCPOLARITY_HIGH, TIM_OCNPOLARITY_HIGH, TIM_OCFAST_DISABLE, TIM_OCIDLESTATE_RESET, TIM_OCNIDLESTATE_RESET};
void MotoCtrl_SetValue(int16_t value, int16_t motor) {
    // ת�����÷�Χ�ж�
    if ((value>200)||(value<-200)) return;
    TIM_OC_InitTypeDef sConfigOC = sConfig;

    if ( motor==MOTOR_2 || !~motor ){
      // ʵ������PWM��ֵ�� 100 - 500
      sConfigOC.Pulse = value+300;
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    }

    if ( motor==MOTOR_1 || !~motor ){
      // ʵ������PWM��ֵ�� 100 - 500
      sConfigOC.Pulse = -value+300;
    	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
      if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    }
}

void GY_UART_Init(void) {
  HAL_UART_Transmit_IT(&huart1, GY_T_EUR, sizeof(GY_T_EUR)-1);
  HAL_Delay(1);
  HAL_UART_Transmit_IT(&huart1, GY_T_RO, sizeof(GY_T_RO)-1);
  HAL_Delay(100);
  HAL_UART_Transmit_IT(&huart1, GY_FIX_ACC, sizeof(GY_FIX_ACC)-1);
}

void GY_UARTPackage_Unpack(void) {
  GY953Frame* gf = (GY953Frame*)UserRecvBuff1;
  unsigned char s = 0;
  for(s = 0; s < USART1_RECV_LEN_MAX-5; s++) {
    if(gf->Head != 0x5a5a) gf = (GY953Frame*)(UserRecvBuff1+s);
    else break;
  }
  if(s == USART1_RECV_LEN_MAX-5) return;
  unsigned char* rawd = gf->RawData;
  int16_t X = __builtin_bswap16(*(uint16_t*)(rawd));
  int16_t Y = __builtin_bswap16(*(uint16_t*)(rawd+2));
  int16_t Z = __builtin_bswap16(*(uint16_t*)(rawd+4));
  switch(gf->Type) {
    case GY_ACC:
      //AccX = X; AccY = Y; AccZ = Z;
      //HAL_UART_Transmit_IT(&huart2, "recv acc.\n", 10);
    break;
    case GY_RO:
      //GyroX = X; GyroY = Y; GyroZ = Z;
      if(Y>128 || Y<-128 || (Y>-4&&Y<4)) {
        isunstable = 1;
        MotoCtrl_SetValue(0, MOTOR_ALL);
      } else isunstable = 0;
      //HAL_UART_Transmit_IT(&huart2, "recv ro.\n", 9);
    break;
    case GY_MG:
      //MgX = X; MgY = Y; MgZ = Z;
      //HAL_UART_Transmit_IT(&huart2, "recv mg.\n", 9);
    break;
    case GY_EUR:
      //Roll = X;
      Pitch = Y; Yaw = Z;
      //HAL_UART_Transmit_IT(&huart2, "recv eur.\n", 10);
      Init_Eular(Y, Z);
    break;
    default: break;
  }
}

static int16_t yinit, pinit;
void Init_Eular(int16_t p, int16_t y) {
  static uint16_t isinit = 1;
  if(isinit && (Pitch||Yaw)) {
    char sndbuf[256];
    sndbuf[0] = 0;
    isinit = 0;
    yinit = Yaw;
    pinit = Pitch;
    sprintf(sndbuf, "[Init] roll: %d, pitch: %d, yaw: %d\n", Roll, Pitch, Yaw);
    int sndlen = strlen(sndbuf) + 1;
    if(sndlen > 1) HAL_UART_Transmit_IT(&huart2, sndbuf, sndlen);
  }
}

void Calc_Speed(void) {
  char sndbuf[256];
  int16_t dp = Pitch-pinit;
  int16_t dy = Yaw-yinit;
  int16_t d1 = 0, d2 = 0;
  if(dy<-32 || dy>32) {
    if(dy<-200) dy = -200;
    else if(dy>200) dy = 200;
    d1 += dy/2;
    d2 += dy/4;
  }
  if(dp<-64 || dp>64) {
    if(dp<-200) dp = -200;
    else if(dp>200) dp = 200;
    d1 += dp/2;
    d2 += dp/2;
  }
  if(d1<-200) d1 = -200;
  else if(d1>200) d1 = 200;
  if(d2<-200) d2 = -200;
  else if(d2>200) d2 = 200;
  speed1 = d1;
  speed2 = d2;
  sprintf(sndbuf, "[Calc] speed1: %d, speed2: %d\n", d1, d2);
  int sndlen = strlen(sndbuf) + 1;
  if(sndlen > 1) HAL_UART_Transmit_IT(&huart2, sndbuf, sndlen);
}

void Bluetooth_Recv(uint8_t cmd) {
  if(bs.isstarted) {
    switch(cmd) {
      case CMD_FRNT: MotoCtrl_SetValue(bs.speed, MOTOR_ALL); break;
      case CMD_BACK: MotoCtrl_SetValue(-bs.speed, MOTOR_ALL); break;
      case CMD_LEFT: MotoCtrl_SetValue(bs.speed>>1, MOTOR_1); MotoCtrl_SetValue(bs.speed, MOTOR_2); break;
      case CMD_RGHT: MotoCtrl_SetValue(bs.speed, MOTOR_1); MotoCtrl_SetValue(bs.speed>>1, MOTOR_2); break;
      case CMD_STOP: if(bs.cmd != CMD_STOP) MotoCtrl_SetValue(0, MOTOR_ALL); else bs.isstarted = 0; break;
      case CMD_LOCK: bs.islocked = ~bs.islocked; break;
      default: bs.speed = ((int)cmd)<<1; break;
    }
  } else if(cmd == CMD_STAT) {
    bs.isstarted = 1;
    bs.speed = 50;
    MotoCtrl_SetValue(50, MOTOR_ALL);
  }
  bs.cmd = cmd;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
