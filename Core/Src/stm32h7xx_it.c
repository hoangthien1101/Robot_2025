/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_it.h"
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
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN EV */
extern volatile int32_t num_over_t1, num_over_t2, num_over_t3, num_over_t4, num_over_t5, num_over_t8;
extern uint8_t RX_UART1[2], RX_UART2[15], RX_UART3[1], RX_UART4[10], RX_UART5[11];
extern int16_t IMU; 

void sendData_Gamepad_Grygo(void);

extern char MANG_GAME[8];
extern char get_Gamepad_Bluetouch(unsigned char *GP_Arr);
extern UART_HandleTypeDef huart5;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */
		static char c;
		if(RX_UART4[9]!=13)
			{
				for(c=0;c<10;c++){	RX_UART4[c]=0;	}// xoa data loi	
				HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
				HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
			}	
			
		else	IMU = -((RX_UART4[0]<<8)|RX_UART4[1]);
			
  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */
	if(!get_Gamepad_Bluetouch(MANG_GAME)){
			HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);
			HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);	
		}
  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */
	float read_IMU_mask_SS = 0;
	int16_t IMU_mask = 0;
  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */
	if(RX_UART5[0]==0x55 && RX_UART5[1]==0x53){
    read_IMU_mask_SS = (float)((RX_UART5[7] << 8) | RX_UART5[6]) / 32768.0 * 1800.0;
    if (read_IMU_mask_SS > 1800) read_IMU_mask_SS -= 3600;
    IMU_mask = read_IMU_mask_SS * 10;
    update_accumulated_yaw(IMU_mask);
	}
	else {
		HAL_UART_DeInit(&huart5);
		HAL_UART_Init(&huart5);
		HAL_UART_Receive_DMA(&huart5, RX_UART5, 11);
	}
  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET) 
    {
        if (__HAL_TIM_GET_ITSTATUS(&htim1, TIM_IT_UPDATE) != RESET) 
        {
            __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

            // Ki?m tra hu?ng d?m d? quy?t d?nh tang hay gi?m encoder_count
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) 
            {
                num_over_t1--;
            } 
            else 
            {
                num_over_t1++;
            }
        }
    }
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) 
    {
        if (__HAL_TIM_GET_ITSTATUS(&htim2, TIM_IT_UPDATE) != RESET) 
        {
            __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

            // Ki?m tra hu?ng d?m d? quy?t d?nh tang hay gi?m encoder_count
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) 
            {
                num_over_t2--;
            } 
            else 
            {
                num_over_t2++;
            }
        }
    }
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
		if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET) 
    {
        if (__HAL_TIM_GET_ITSTATUS(&htim3, TIM_IT_UPDATE) != RESET) 
        {
            __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

            // Ki?m tra hu?ng d?m d? quy?t d?nh tang hay gi?m encoder_count
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) 
            {
                num_over_t3--;
            } 
            else 
            {
                num_over_t3++;
            }
        }
    }
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET) 
    {
        if (__HAL_TIM_GET_ITSTATUS(&htim4, TIM_IT_UPDATE) != RESET) 
        {
            __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

            // Ki?m tra hu?ng d?m d? quy?t d?nh tang hay gi?m encoder_count
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) 
            {
                num_over_t4--;
            } 
            else 
            {
                num_over_t4++;
            }
        }
    }
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE) != RESET) 
    {
        if (__HAL_TIM_GET_ITSTATUS(&htim8, TIM_IT_UPDATE) != RESET) 
        {
            __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);

            // Ki?m tra hu?ng d?m d? quy?t d?nh tang hay gi?m encoder_count
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8)) 
            {
                num_over_t8--;
            } 
            else 
            {
                num_over_t8++;
            }
        }
    }
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET) 
    {
        if (__HAL_TIM_GET_ITSTATUS(&htim5, TIM_IT_UPDATE) != RESET) 
        {
            __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);

            // Ki?m tra hu?ng d?m d? quy?t d?nh tang hay gi?m encoder_count
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5)) 
            {
                num_over_t5--;
            } 
            else 
            {
                num_over_t5++;
            }
        }
    }
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	sendData_Gamepad_Grygo();
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
