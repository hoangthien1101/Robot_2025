/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ala_sendData.h
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
#ifndef __ALA_SENDDATA_H__
#define __ALA_SENDDATA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
/* -------------------------KHAI BAO DINH NGHIA PWM---------------------------*/
extern uint8_t motorData[80];																			// mang du lieu truyen di

#define	byteStart1														motorData[0]=0xff		// byte dau cho chuoi truyen di	
#define	ID1 		 															motorData[1]=1			//  byte dia chi
#define mor_10h_next 	 					 							motorData[2]=0			//  byte dao chieu dong co
#define mor_10h_back 													motorData[2]=1			
#define	mor_10h			 		 											motorData[3]			// 	byte toc do

#define	byteStart2														motorData[4]=0xff	
#define	ID2 		 															motorData[5]=2
#define mor_2h_next 											 		motorData[6]=0
#define mor_2h_back 													motorData[6]=1
#define	mor_2h																motorData[7]

#define	byteStart3														motorData[8]=0xff	
#define	ID3 		 															motorData[9]=3
#define mor_4h_next 													motorData[10]=0
#define mor_4h_back 													motorData[10]=1
#define	mor_4h 																motorData[11]=0

#define	byteStart4														motorData[12]=0xff	
#define	ID4 		 															motorData[13]=4
#define mor_8h_next 													motorData[14]=0
#define mor_8h_back 													motorData[14]=1
#define	mor_8h 																motorData[15]

/*---------------------Ket thuc khai bao chuoi du lieu dieu khien Driver--------------------------*/
extern UART_HandleTypeDef huart1;
extern uint16_t crc_1, crc_2;

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

void sendDataToDriver(void);
void Error_Handler(void);
void UART_DMA_Transmit(uint8_t *pData, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* __ALA_SENDDATA_H__ */

