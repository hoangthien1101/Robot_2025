// 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GAMEPAD_H
#define __GAMEPAD_H

#ifdef __cplusplus
 extern "C" {
#endif 

/*
*************************************************************************************************************************************
*															INCLUDED FILES															*
*************************************************************************************************************************************
*/
//#include "stm32h7xx_hal.h"


//extern uint8_t MANG_GAME[10];
extern int run;
#define GP_MASK_SELECT 		0x01
#define GP_MASK_START			0x08
#define GP_MASK_UP				0x10
#define GP_MASK_RIGHT			0x20
#define GP_MASK_DOWN			0x40
#define GP_MASK_LEFT			0x80
#define GP_MASK_L2				0x01
#define GP_MASK_R2				0x02
#define GP_MASK_L1				0x04
#define GP_MASK_R1				0x08
#define GP_MASK_TRIANGLE    0x10
#define GP_MASK_O           0x20
#define GP_MASK_X           0x40
#define GP_MASK_SQUARE      0x80
#define GP_MASK_RJOY        0x02
#define GP_MASK_LJOY        0x04

#define GP_MODE_DIGITAL               0x41
#define GP_MODE_NEGCON                0x23
#define GP_MODE_ANALOGUE_RED_LED      0x73
#define GP_MODE_ANALOGUE_GREEN_LED    0x53
#define GP_MODE_NONE                  0xff


// cac nut nhan tren tay game

#define SELECT        (GP_BTN[0] & GP_MASK_SELECT)
#define START        (GP_BTN[0] & GP_MASK_START)
#define RJOY        (GP_BTN[0] & GP_MASK_RJOY)
#define LJOY        (GP_BTN[0] & GP_MASK_LJOY)

#define UP            (GP_BTN[0] & GP_MASK_UP)
#define RIGHT        (GP_BTN[0] & GP_MASK_RIGHT)
#define DOWN        (GP_BTN[0] & GP_MASK_DOWN)
#define LEFT        (GP_BTN[0] & GP_MASK_LEFT)

#define L2            (GP_BTN[1] & GP_MASK_L2)
#define R2            (GP_BTN[1] & GP_MASK_R2)
#define L1            (GP_BTN[1] & GP_MASK_L1)
#define R1            (GP_BTN[1] & GP_MASK_R1)

#define TRIANGLE    (GP_BTN[1] & GP_MASK_TRIANGLE)
#define O            (GP_BTN[1] & GP_MASK_O)
#define X            (GP_BTN[1] & GP_MASK_X)
#define SQUARE        (GP_BTN[1] & GP_MASK_SQUARE)

#define RJOY_LR        (GP_BTN[2])
#define RJOY_UD        (GP_BTN[3])
#define LJOY_LR        (GP_BTN[4])
#define LJOY_UD        (GP_BTN[5])

#define SWITCH_R (GP_BTN[6]&0x02)

//    *************    khai bao bien toan cuc     ***********
char GP_BTN [10];
char MANG_GAME[8];
extern char bienBaoLech_Byte;
extern char bienTruyenNhan;
//int8_t bien_tang;
//void putchar3(unsigned char ch);
char get_Gamepad_Bluetouch(char *GP_Arr)
{
	static char a;
		if(GP_Arr[7] != 13){
			if(a)	GP_BTN[0] = 0; //SLCT JOYR JOYL STRT UP  RGHT DOWN LEFT
				else GP_BTN[0] = 0;
			GP_BTN[1] = 0; //L2   R2   L1   R1   /\   O    X    |_|
			GP_BTN[2] = 0; //2 Joy
			GP_BTN[3] = 0; //2 Joy
			GP_BTN[4] = 0; //2 Joy
			GP_BTN[5] = 0; //2 Joy
			GP_BTN[6] = 0; //2 Joy
			a=~a;
			return 0;
		}else{
			GP_BTN[0] = GP_Arr[0]; //SLCT JOYR JOYL STRT UP   RGHT DOWN LEFT
			GP_BTN[1] = GP_Arr[1]; //L2   R2   L1   R1   /\   O    X    |_|
			GP_BTN[2] = GP_Arr[2]; //2 Joy
			GP_BTN[3] = GP_Arr[3]; 
			GP_BTN[4] = GP_Arr[4]; 
			GP_BTN[5] = GP_Arr[5]; 
			GP_BTN[6] = GP_Arr[6]; 
			//bienTruyenNhan = 1;
		}
		//putchar3(bien_tang);
		return 1;
}

unsigned char gp_get_mode_uart (void)
{
  return GP_BTN[6];
} 
char wantExit (void)
{
	if(!DOWN) {
		return 1;
	} 
//	if(!X)  return 1;
		else return 0;
}

#ifdef __cplusplus
}
#endif 	


#endif /* __GAMEPAD_H */

/******************* (C) COPYRIGHT 2018 LH_ATM *****END OF FILE****/

