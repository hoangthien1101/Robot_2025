#include "ala_sendData.h"

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

void sendDataToDriver(void){
	//HAL_UART_Transmit(&huart1, motorData, sizeof(motorData), 1000);

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
