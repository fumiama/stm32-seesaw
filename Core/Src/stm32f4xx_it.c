/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern uint8_t     USART1_SendBuff[USART1_SEND_LEN_MAX];
extern uint8_t     USART1_RecvBuff[USART1_RECV_LEN_MAX];
extern uint8_t     UserRecvBuff1[USART1_RECV_LEN_MAX];

extern uint8_t     USART2_SendBuff[USART2_SEND_LEN_MAX];
extern uint8_t     USART2_RecvBuff[USART2_RECV_LEN_MAX];
//extern uint8_t     UserRecvBuff2[USART2_RECV_LEN_MAX];

extern BTSTAT bs;
extern int16_t speed1, speed2;
extern uint8_t isunstable;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
static uint8_t isinit = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  static uint16_t tick = 0, tickunstable = 0, rate = 256;
  if(htim->Instance == TIM2) {
    if (isinit && !bs.isstarted) {
      if(isunstable) {
        if(!tickunstable++) rate >>= 1;
        if(!(tickunstable&1023)) isunstable = tickunstable = 0;
      } else if(!(tick++&15)) {
        int16_t tmod = tick&511;
        tickunstable = 0;
        if(tmod==1) {
          Calc_Speed();
          if(speed1) MotoCtrl_SetValue(speed1, MOTOR_1);
          if(speed2) MotoCtrl_SetValue(speed2, MOTOR_2);
        } else if(tmod==rate+1) {
          MotoCtrl_SetValue(0, MOTOR_ALL);
          GY_UART_Switch();
        }
      }
    }
  }
}

//
//  HAL_UART_RxCpltCallback 串口接收中断处理函数
//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // 进入这个程序，说明 产生了串口接收数据中断
  // 判断是哪个串口接收中断
  if (huart->Instance == USART1) {
    // 在这里添加 串口1 的接收中断处理程序  
    // 把 USART1_RecvBuff 里面的数据拷到 UserRecvBuff1 里
    // 这样，USART1_RecvBuff 可以继续用于中断接收
    // 用户程序可以从 UserRecvBuff1 里读取数据来进行下一步处理
    // UART1_RecvFlag 是用来表示是否收到了数据
    memcpy(UserRecvBuff1, USART1_RecvBuff, USART1_RECV_LEN_MAX);
    GY_UARTPackage_Unpack();
    // HAL_UART_Receive_IT 接收中断只有效1次，需要再次开启，才能再次接收串口数据
    while(HAL_UART_Receive_IT(&huart1, USART1_RecvBuff, USART1_RECV_LEN_MAX) != HAL_OK);
  }
  
  if (huart->Instance == USART2) {
    // 在这里添加 串口2 的接收中断处理程序，原理与上面的  串口1 的接收中断处理程序 一样
    // 这里就不再重复解释
    Bluetooth_Recv(USART2_RecvBuff[0]);
    // HAL_UART_Receive_IT 接收中断只有效1次，需要再次开启，才能再次接收串口数据
    while(HAL_UART_Receive_IT(&huart2, USART2_RecvBuff, USART2_RECV_LEN_MAX) != HAL_OK);
  }
}

/** GPIO中断回调
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // 上升按钮
    if(GPIO_Pin == UKEY_Pin) {
        static int8_t btnOn = 0;
        GPIO_PinState pinState = HAL_GPIO_ReadPin(UKEY_GPIO_Port, UKEY_Pin);
        if(btnOn) {
          if(pinState) {   // 弹起
            // 防抖判断
            for(uint8_t i = 0; i < 5; ++i) {
              HAL_Delay(10);
              if(!HAL_GPIO_ReadPin(UKEY_GPIO_Port, UKEY_Pin)) return;
            }
            btnOn = 0;
            if(isinit) MotoCtrl_SetValue(0, MOTOR_ALL);
            else MotoCtrl_SetValue(20, MOTOR_ALL);
            GY_UART_Init();
            isinit = !isinit;
            HAL_GPIO_TogglePin(LED_IDC_GPIO_Port, LED_IDC_Pin);
          }
        } else if(!pinState) {   // 按下
          // 防抖判断
          for(uint8_t i = 0; i < 8; ++i) {
            HAL_Delay(50);
            if(HAL_GPIO_ReadPin(UKEY_GPIO_Port, UKEY_Pin)) return;
          }
          btnOn = 1;
        }
        // 清除标志
        // __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    }
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
