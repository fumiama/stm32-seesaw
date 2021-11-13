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
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/**********蓝牙控制的相关变量*********/
struct BTSTAT {
  int16_t islocked:  1,
          isstarted: 1,
          cmd     : 7,
          speed   : 7; // 速度/2
};
/**********蓝牙控制的相关变量*********/
typedef struct BTSTAT BTSTAT;
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
void MotoCtrl_SetValue(int16_t value, int16_t motor);
void GY_UARTPackage_Unpack(void);
void GY_UART_Init(void);
void Bluetooth_Recv(uint8_t cmd);
void Calc_Speed(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_IDC_Pin GPIO_PIN_13
#define LED_IDC_GPIO_Port GPIOC
#define UKEY_Pin GPIO_PIN_0
#define UKEY_GPIO_Port GPIOA
#define UKEY_EXTI_IRQn EXTI0_IRQn
#define MotorCtrl1_Pin GPIO_PIN_8
#define MotorCtrl1_GPIO_Port GPIOA
#define MotorCtrl2_Pin GPIO_PIN_11
#define MotorCtrl2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define USART1_SEND_LEN_MAX        16     // 串口 1 发送缓冲区的最大长度
#define USART1_RECV_LEN_MAX        22

#define USART2_SEND_LEN_MAX        16     // 串口 2 发送缓冲区的最大长度
#define USART2_RECV_LEN_MAX        1

#define MOTOR_ALL   -1
#define MOTOR_1	    0
#define MOTOR_2	    1

#define  STATE_READY          0						//定义了当前接收数据帧的状态
#define  STATE_HEAD1          1
#define  STATE_HEAD2          2
#define  STATE_TYPE           3
#define  STATE_NUM            4
#define  STATE_DATA           5

typedef struct {              //GY953传感器的  数据帧结构体
  uint16_t      Head;         //帧头0x5a5a
	unsigned char Type;         //数据帧类型，加速度，陀螺仪，地磁，欧拉角，4元素。。。
	unsigned char Length;       //要接收数据的长度
	unsigned char RawData[];    //原始数据
} GY953Frame;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
