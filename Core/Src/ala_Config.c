#include "ala_Config.h"
#include "main.h"
/*-------- Khai bao dia chi driver tai mang nay -----------------------*/
uint8_t motorData[80]={255,1,0,0,				// 1- ID = 1, DIRECT = 0, SPEED = 0
												255,2,0,0,			// 1- ID = 2, DIRECT = 0, SPEED = 0
												255,3,0,0,			// 1- ID = 3, DIRECT = 0, SPEED = 0
												255,4,0,0,			// 1- ID = 4, DIRECT = 0, SPEED = 0
												255,5,0,0,			// 1- ID = 5, DIRECT = 0, SPEED = 0
												255,6,0,0,			// 1- ID = 6, DIRECT = 0, SPEED = 0
												255,7,0,0,			// 1- ID = 7, DIRECT = 0, SPEED = 0
												255,8,0,0,			// 1- ID = 8, DIRECT = 0, SPEED = 0
												255,9,0,0,			// 1- ID = 9, DIRECT = 0, SPEED = 0
												255,10,0,0,			// 1- ID = 10, DIRECT = 0, SPEED = 0
												255,11,0,0,			// 1- ID = 11, DIRECT = 0, SPEED = 0
												255,12,0,0,			// 1- ID = 12, DIRECT = 0, SPEED = 0
												255,13,0,0,			// 1- ID = 13, DIRECT = 0, SPEED = 0	
												255,14,0,0,			// 1- ID = 14, DIRECT = 0, SPEED = 0
												255,15,0,0,			// 1- ID = 15, DIRECT = 0, SPEED = 0
												255,16,0,0,			// 1- ID = 16, DIRECT = 0, SPEED = 0
												255,17,0,0,			// 1- ID = 17, DIRECT = 0, SPEED = 0
												255,18,0,0,			// 1- ID = 18, DIRECT = 0, SPEED = 0
												255,19,0,0,			// 1- ID = 19, DIRECT = 0, SPEED = 0
												255,20,0,0,			// 1- ID = 20, DIRECT = 0, SPEED = 0	
};


int isSizeRxed = 0;
uint16_t size = 0;
uint8_t RxData[2048];

// Khai bao tham bien encoder
volatile int32_t num_over_t1 = 0, num_over_t2 = 0, num_over_t3 = 0, num_over_t4 = 0, num_over_t5 = 0, num_over_t8 = 0, num_over_t10 = 0;
//extern volatile int32_t num_over_t1, num_over_t2, num_over_t3, num_over_t4, num_over_t5, num_over_t8;
////////////////////////////////////////


int32_t readEncoderTim_1(void) //TIMER 1
{
	return ((num_over_t1<<16)|TIM1->CNT);  // giam so xung encoder xuong 1 nua
}

int32_t readEncoderTim_2(void) //TIMER 2
{
	return ((num_over_t2<<16)|TIM2->CNT);  // giam so xung encoder xuong 1 nua
}

int32_t readEncoderTim_3(void) //TIMER 3
{
	return ((num_over_t3<<16)|TIM3->CNT);  // giam so xung encoder xuong 1 nua
}

int32_t readEncoderTim_4(void) //TIMER 4
{
	return ((num_over_t4<<16)|TIM4->CNT);  // giam so xung encoder xuong 1 nua
}

int32_t readEncoderTim_5(void) //TIMER 5
{
	return ((num_over_t5<<16)|TIM5->CNT);  // giam so xung encoder xuong 1 nua
}

int32_t readEncoderTim_8(void) //TIMER 8
{
	return ((num_over_t8<<16)|TIM8->CNT);  // giam so xung encoder xuong 1 nua
}

void putchar4(unsigned char ch)
{
	HAL_UART_Transmit(&huart4, (uint8_t*)&ch,1,0x01FF); 
}

void run_read_gyro_uart4(void)
{ 
	int8_t i;
  for(i=0;i<7;i++)
  {
		putchar4('a');		
		HAL_Delay(2);
  }
	HAL_Delay(100);
	putchar4('z');
	HAL_Delay(100);
}

void sendData_Gamepad_Grygo(void){
	putchar4('z');
//	if(!bit_reset_IMU)	putchar4('z'); // gui yeu cau lay gia tri la ban	
//		else {
//				putchar4('a');
//				osDelay(50);
//				bit_reset_IMU = 0;
//			}
}

void sendDataToDriver(void){
	HAL_UART_Transmit(&huart1, motorData, sizeof(motorData), 1000);

//	HAL_UART_Transmit_DMA(&huart1, motorData, sizeof(motorData));
}

// Function to send data using UART and DMA
//void UART_DMA_Transmit(uint8_t *pData, uint16_t Size) {
//    if (HAL_UART_Transmit_DMA(&huart1, pData, Size) != HAL_OK) {
//        // Transmission Error
//        Error_Handler();
//    }
//}

// Error handler function
//void Error_Handler(void) {
//    // User may add their own error handling code here
//    while (1) {
//    }
//}
