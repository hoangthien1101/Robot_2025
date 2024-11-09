/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ala_Config.h
  * @brief   This file contains all the function prototypes for
  *          the ala_sendData.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 08/07/2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#ifndef __ALA_CONGIG_H__
#define __ALA_CONGIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
/* -------------------------KHAI BAO DINH NGHIA PWM---------------------------*/
extern uint8_t motorData[80];																			// mang du lieu truyen di

#define	byteStart1														motorData[0]=0xff		// byte dau cho chuoi truyen di	
#define	ID1 		 															motorData[1]=1			//  byte dia chi
#define mor_0hRo_next 	 					 							motorData[2]=0			//  byte dao chieu dong co
#define mor_0hRo_back 													motorData[2]=1			
#define	mor_0hRo			 		 											motorData[3]			// 	byte toc do

#define	byteStart2														motorData[4]=0xff	
#define	ID2 		 															motorData[5]=2
#define mor_0h_next 											 		motorData[6]=0
#define mor_0h_back 													motorData[6]=1
#define	mor_0h																motorData[7]

#define	byteStart3														motorData[8]=0xff	
#define	ID3 		 															motorData[9]=3
#define mor_4hRo_next 													motorData[10]=0
#define mor_4hRo_back 													motorData[10]=1
#define	mor_4hRo 																motorData[11]

#define	byteStart4														motorData[12]=0xff	
#define	ID4 		 															motorData[13]=4
#define mor_4h_next 													motorData[14]=1
#define mor_4h_back 													motorData[14]=0
#define	mor_4h 																motorData[15]

#define	byteStart5														motorData[16]=0xff	
#define	ID5 		 															motorData[17]=5
#define mor_8hRo_next 													motorData[18]=0
#define mor_8hRo_back 													motorData[18]=1
#define	mor_8hRo 																motorData[19]

#define	byteStart6														motorData[20]=0xff	
#define	ID6 		 															motorData[21]=6
#define mor_8h_next 													motorData[22]=0
#define mor_8h_back 													motorData[22]=1
#define	mor_8h 																motorData[23]

#define RED_BLUE	 HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) // HIGH IS BLUE - LOW IS RED
#define CB_0h	 	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)
#define CB_8h	 	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)
#define CB_4h	 	HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)
/*---------------------Ket thuc khai bao chuoi du lieu dieu khien Driver--------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;


typedef struct struct_ControlMotor {
  uint8_t a[8];    
  // uint8_t b;
  // uint8_t c;
  // uint8_t d;
  // uint8_t e;
  // uint8_t f;
  // uint8_t g;
  // uint8_t h;
} struct_message;

//int32_t readEncoderTim_1(void);
//int32_t readEncoderTim_2(void);
//int32_t readEncoderTim_3(void);
//int32_t readEncoderTim_4(void);
//int32_t readEncoderTim_5(void);
//int32_t readEncoderTim_8(void);

//void putchar4(unsigned char ch);
//void run_read_gyro_uart4(void);
//void sendData_Gamepad_Grygo(void);
void sendDataToDriver(void);
//void Error_Handler(void);
//void UART_DMA_Transmit(uint8_t *pData, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* __ALA_CONGIG_H__ */

