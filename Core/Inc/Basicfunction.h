#include "gamepad_bluetooth.h"
#include "stm32h7xx_hal.h"
#include "ala_Config.h"
#include "cmsis_os.h"
#include "4SwerveWheel.h"

extern int TARGET_SPEED;
extern int increment; // Bi?n gia tang d? di?u ch?nh t?c d? tang d?n
extern int decrement; // Bi?n gi?m d? di?u ch?nh t?c d? gi?m d?n
extern uint8_t RX_UART1[2], RX_UART2[15], RX_UART3[1], RX_UART4[10], RX_UART5[11];
//Khai bao cac bien do line
int gocTam = 0;
int errorLinePhai = 0, errorLineTrai = 0;
float speedRobotLine = 0, erRobotLine = 0, lastErRobotLine = 0, KdRobotLine = 0, KiRobotLine = 0, KpRobotLine = 0;
//************End****************//
extern char xylanhkep1, xylanhkep2, xylanhtruot;
extern char bienTruyenNhan;
extern unsigned char robot_Data [8];
extern uint16_t adc_data[11];
uint32_t _ADC[11],_ADC_SUM[11];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern volatile int32_t num_over_t1, num_over_t2, num_over_t3, num_over_t4, num_over_t5, num_over_t8, num_over_t10;
uint32_t lazer_Truoc = 0, lazer_Trai = 0, lazer_Phai = 0;
//===================Khai bao cac bien cho bo banh xe======================
int bit_stop = 0, thoatVongLap = 0;
int testXoa = 0, voHieuHoaDiThang = 1, demCheckHamDieuTocDC = 0;
int old_currentSpeed = 0, bienTangGiaToc = 0;
char bitResetBanhXe = 1, bitTest = 0;
int gocQuayBanh = 0;
int speedGocQuay0h = 0, erGocQuay0h = 0, lastErGocQuay0h = 0, KdGocQuay0h = 0, KiGocQuay0h = 0, KpGocQuay0h = 0;
int speedGocQuay4h = 0, erGocQuay4h = 0, lastErGocQuay4h = 0, KdGocQuay4h = 0, KiGocQuay4h = 0, KpGocQuay4h = 0;
int speedGocQuay8h = 0, erGocQuay8h = 0, lastErGocQuay8h = 0, KdGocQuay8h = 0, KiGocQuay8h = 0, KpGocQuay8h = 0;
int speedGocQuay10h = 0, erGocQuay10h = 0, lastErGocQuay10h = 0, KdGocQuay10h = 0, KiGocQuay10h = 0, KpGocQuay10h = 0;
int TH = 0, check1Lan = 0, gocBanDau = 0;
int erGocquay = 0, heSoQuayGoc = 0;

int bienBreak = 0;
extern float speed2h, speed4h, speed8h, speed10h, angle2h, angle4h, angle8h, angle10h;
//===================End Khai bao cac bien cho bo banh xe======================
//===================Khai bao cac bien cho la ban======================
int16_t accumulated_yaw = 0.0f;
int16_t previous_yaw = 0.0f;
int8_t first_run_yaw = 1;
extern int16_t IMU, IMU_china;
//===================End Khai bao cac bien cho la ban======================
int speedQuayTayBong = 0, erQuayTayBong = 0, lastErQuayTayBong = 0, KdQuayTayBong = 0, KiQuayTayBong = 0, KpQuayTayBong = 0;
//Khai bao cac bit chuong trinh
uint8_t bit_veHome = 1;
uint8_t bitNgatBanh0h = 0, bitNgatBanh4h = 0, bitNgatBanh8h = 0;
//Ket thuc khai bao cac bit chuong trinh
int angle = 0;       // Góc t? 0 d?n 360 d? (ki?u int)
int magnitude = 0;   // Ð? dài du?ng chéo (t?c d?) t? tâm (ki?u int)
volatile int16_t encoder_count_T1 = 0, encoder_count_T2 = 0, encoder_count_T3 = 0, encoder_count_T4 = 0, encoder_count_T5 = 0, encoder_count_T8 = 0; // Gia tri CNT hien tai (co the am)
volatile int16_t prev_encoder_count_T1 = 0, prev_encoder_count_T2 = 0, prev_encoder_count_T3 = 0, prev_encoder_count_T4 = 0, prev_encoder_count_T5 = 0, prev_encoder_count_T8 = 0; // Gia tri CNT truoc do
volatile int32_t encoder_value_T1 = 0, encoder_value_T2 = 0, encoder_value_T3 = 0, encoder_value_T4 = 0, encoder_value_T5 = 0, encoder_value_T8 = 0; // Bien 32-bit luu gia tri thuc

void putchar1(unsigned char ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch,1,0x01FF);
	//UART1->DR = (ch & (uint16_t)0x01FF); 
}
void resetIMU_TQ()
{
	uint8_t unblockIMU[5] = {0xFF, 0XAA, 0X69, 0X88, 0XB5};
	uint8_t resetDataIMU[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
	uint8_t saveDataIMU[5] = {0xFF, 0XAA, 0X00, 0X00, 0X00};
	HAL_UART_Transmit(&huart5, unblockIMU,5,0x01FF);
	HAL_Delay(2);
	HAL_UART_Transmit(&huart5, resetDataIMU,5,0x01FF);
	HAL_Delay(2);
	HAL_UART_Transmit(&huart5, saveDataIMU,5,0x01FF);
	first_run_yaw = 1;
}
//Ham update doc la ban Trung Quoc
void update_accumulated_yaw(int16_t current_yaw) {
    int16_t delta_yaw;

    // Neu la lan chay dau tien, thiet lap gia tri ban dau cho previous_yaw
    if (first_run_yaw) {
        previous_yaw = current_yaw;
        first_run_yaw = 0;
    }

    // Tính toán s? thay d?i c?a góc yaw
    delta_yaw = current_yaw - previous_yaw;

    // Dieu chinh delta_yaw khi vuot qua pham vi -1800 den 1800
    if (delta_yaw > 1800) {
        delta_yaw -= 3600;
    } else if (delta_yaw < -1800) {
        delta_yaw += 3600;
    }

    // Cong don gia tri yaw
    accumulated_yaw += delta_yaw;

    // Cap nhat gia tri truoc do
    previous_yaw = current_yaw;
		
		IMU_china = accumulated_yaw;
}
void putchar3(unsigned char ch)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch,1,0x01FF);
	//UART3->DR = (ch & (uint16_t)0x01FF); 
}
void putchar5(unsigned char ch)
{
	HAL_UART_Transmit(&huart5, (uint8_t*)&ch,1,0x01FF);
	//UART5->DR = (ch & (uint16_t)0x01FF); 
}
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ADCValue_Control()
{
	static int count,i;
	if(count++ <5)
	{
		for(i = 0; i < 8; i++)
		{
			_ADC_SUM[i] += adc_data[i];
		}
	}
	else 
	{
		for(i = 0; i < 8; i++)
		{
			_ADC[i] = _ADC_SUM[i]/5;
			_ADC_SUM[i] = 0;

		}
		count = 0;
	}
}
//Ham trung gian de quy doi encoder theo ti le banh rang, ghi vao thanh ghi khong ghi duoc so am
//int32_t quyDoiEncoder2h(int32_t encoder){
//	int32_t encoderQuyDoi = encoder*0.033536;
//	return encoderQuyDoi;
//}
//int32_t quyDoiEncoder4h(int32_t encoder){
//	int32_t encoderQuyDoi = encoder*0.033536;
//	return encoderQuyDoi;
//}
//int32_t quyDoiEncoder8h(int32_t encoder){
//	int32_t encoderQuyDoi = encoder*0.033536;
//	return encoderQuyDoi;
//}
int32_t quyDoiEncoder(int32_t encoder){
	int32_t encoderQuyDoi = encoder/10.766;
	return encoderQuyDoi;
}
void update_encoder_value() {
    // Doc gia tri hien tai cua encoder
  encoder_count_T1 = TIM1->CNT;
	encoder_count_T2 = TIM2->CNT;
	encoder_count_T3 = TIM3->CNT;
	encoder_count_T4 = TIM4->CNT;
	encoder_count_T5 = TIM5->CNT;
	encoder_count_T8 = TIM8->CNT;

    // Tinh toan su thay doi
  int16_t delta_T1 = encoder_count_T1 - prev_encoder_count_T1;
	int16_t delta_T2 = encoder_count_T2 - prev_encoder_count_T2;
	int16_t delta_T3 = encoder_count_T3 - prev_encoder_count_T3;
	int16_t delta_T4 = encoder_count_T4 - prev_encoder_count_T4;
	int16_t delta_T5 = encoder_count_T5 - prev_encoder_count_T5;
	int16_t delta_T8 = encoder_count_T8 - prev_encoder_count_T8;

    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T1 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T1 -= 65536;
    } else if (delta_T1 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T1 += 65536;
    }
		
    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T2 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T2 -= 65536;
    } else if (delta_T2 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T2 += 65536;
    }
		
    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T3 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T3 -= 65536;
    } else if (delta_T3 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T3 += 65536;
    }
		
    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T4 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T4 -= 65536;
    } else if (delta_T4 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T4 += 65536;
    }
		
    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T5 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T5 -= 65536;
    } else if (delta_T5 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T5 += 65536;
    }

    // Kiem tra tran hoac nguoc tran va dieu chinh delta
    if (delta_T8 > 32767) {
        // Truong hop dem nguoc qua 0 (underflow)
        delta_T8 -= 65536;
    } else if (delta_T8 < -32767) {
        // Truong hop dem tien qua 65535 (overflow)
        delta_T8 += 65536;
    }


    // Cong don gia tri vao encoder_value
    encoder_value_T1 += delta_T1;
		encoder_value_T2 += delta_T2;
		encoder_value_T3 += delta_T3;
		encoder_value_T4 += delta_T4;
		encoder_value_T5 += delta_T5;
		encoder_value_T8 += delta_T8;

    // Cap nhat gia tri encoder truoc do
    prev_encoder_count_T1 = encoder_count_T1;
		prev_encoder_count_T2 = encoder_count_T2;
		prev_encoder_count_T3 = encoder_count_T3;
		prev_encoder_count_T4 = encoder_count_T4;
		prev_encoder_count_T5 = encoder_count_T5;
		prev_encoder_count_T8 = encoder_count_T8;
		
}

//int16_t en_10hRo() //TIMER 1
//{
//	int16_t en10hRo;
//	en10hRo = (num_over_t1<<16)|(TIM1->CNT);
//	return en10hRo;
//}
//int16_t en_2hRo() //TIMER 2
//{
//	int16_t en2hRo;
//	en2hRo = (num_over_t2<<16)|(TIM2->CNT);
//	return en2hRo;
//}

//int16_t en_4hRo() //TIMER 3
//{
//	int16_t en4hRo;
//	en4hRo = (num_over_t3<<16)|(TIM3->CNT);
//	return en4hRo;
//}

//int16_t en_8hRo() //TIMER 4
//{
//	int16_t en8hRo;
//	en8hRo = (num_over_t4<<16)|(TIM4->CNT);
//	return en8hRo;
//}
//int32_t en_Chay2h() {
//	int16_t enChay2h;
//	enChay2h = (num_over_t5<<16)|(TIM5->CNT);
//	return enChay2h;
//}
//int32_t en_Chay8h() {
//	int32_t enChay8h;
//	enChay8h = (num_over_t8<<16)|(TIM8->CNT);
//	return enChay8h;
//}

int32_t en_10hRo() //TIMER 1
{
	int32_t encoderT1;
	encoderT1 = encoder_value_T1;
	return -quyDoiEncoder(encoderT1);
}
int32_t en_0hRo() //TIMER 5
{
	int32_t encoderT5;
	encoderT5 = encoder_value_T5;
	return quyDoiEncoder(encoderT5);
}

int32_t en_4hRo() //TIMER 3
{
	int32_t encoderT3;
	encoderT3 = encoder_value_T3;
	return -quyDoiEncoder(encoderT3);
}

int32_t en_8hRo() //TIMER 4
{
	int32_t encoderT4;
	encoderT4 = encoder_value_T4;
	return -quyDoiEncoder(encoderT4);
}
int32_t en_Chay2h() { //TIMER 2
	int32_t encoderT2;
	encoderT2 = encoder_value_T2;
	return -quyDoiEncoder(encoderT2);
}
int32_t en_Chay8h() { //TIMER 8
	int32_t encoderT8;
	encoderT8 = encoder_value_T8;
	return -quyDoiEncoder(encoderT8);
}
void ResetEn_Chay(void) 
{
		num_over_t10=0;
}
void ResetEn_10hRo(void) // TIMER1
{
	TIM1->CNT = 0; 
	//TIM1->CNT = 0; //2709
	//num_over_t1 = 0;
	encoder_value_T1 = 0;
	prev_encoder_count_T1 = 0;
}

void ResetEn_0hRo(void) // TIMER5
{
	TIM5->CNT = 0; //-2709
	//num_over_t2 = 0;
	encoder_value_T5 = 0;
	prev_encoder_count_T5 = 0;
}

void ResetEn_4hRo(void) // TIMER3
{
	TIM3->CNT = 0; //2709
	//num_over_t3 = 0;
	encoder_value_T3 = 0;
	prev_encoder_count_T3 = 0;
}

void ResetEn_8hRo(void) // TIMER4
{
	TIM4->CNT = 0; //2709
	//num_over_t4 = 0;
	encoder_value_T4 = 0;
	prev_encoder_count_T4 = 0;
}

void ResetEn_2hChay(void) // TIMER2
{
	TIM2->CNT = 0; //2709
	//num_over_t5 = 0;
	encoder_value_T2 = 0;
	prev_encoder_count_T2 = 0;
}

void ResetEn_8hChay(void) // TIMER8
{
	TIM8->CNT = 0; //2709
	//num_over_t8 = 0;
	encoder_value_T8 = 0;
	prev_encoder_count_T8 = 0;
}

void ResetEncoder_FULL(void)
{
	ResetEn_4hRo();
	ResetEn_8hRo();
	ResetEn_10hRo();
	ResetEn_0hRo();
	ResetEn_2hChay();
	ResetEn_8hChay();
}
void ResetEncoder_chay(void)
{
	ResetEn_2hChay();
	ResetEn_8hChay();
}

void ResetEncoder_Home(void)
{
	TIM3->CNT = 0; //2709
	num_over_t3 = 0;
	
	TIM4->CNT = 0; //2709
	num_over_t4 = 0;
	
	TIM5->CNT = 0; //2709
	num_over_t5 = 0;
	
	// 4hRO
	encoder_value_T3 = 0; 
	prev_encoder_count_T3 = 0;
	// 8hRO
	encoder_value_T4 = 0;
	prev_encoder_count_T4 = 0;
	// 0hRO
	encoder_value_T5 = 0;
	prev_encoder_count_T5 = 0;
}

void ResetIMU(void)
{
	run_read_gyro_uart4();
}
void motor0hRo(int pwm){	
	if(pwm > 230) pwm = 230;
	if(pwm > 0) { mor_0hRo_back;}
	else 		 mor_0hRo_next;  
	mor_0hRo = abs(pwm);
}
void motor4hRo(int pwm){
	if(pwm > 230) pwm = 230;
	if(pwm > 0) { mor_4hRo_back;}
	else 		 mor_4hRo_next;  
	mor_4hRo = abs(pwm);
}
void motor8hRo(int pwm){
	if(pwm > 230) pwm = 230;
	if(pwm > 0) { mor_8hRo_back;}
	else 		 mor_8hRo_next;  
	mor_8hRo = abs(pwm);
}

void quayGoc0h(int goc){
	erGocQuay0h = (goc - en_0hRo());
	KpGocQuay0h = erGocQuay0h * 1.2;
	KiGocQuay0h = (KiGocQuay0h + erGocQuay0h) * 0.1; //0.01
	KdGocQuay0h = (erGocQuay0h - lastErGocQuay0h) * 0.5; //0.1
	speedGocQuay0h = abs(KpGocQuay0h + KiGocQuay0h + KdGocQuay0h);
	
//	speedGocQuay = abs(1.3 * erGocQuay);
	
	if (speedGocQuay0h > 230) {
//		if(abs(erGocQuay4h) < 200) speedGocQuay4h = 50;
//		else speedGocQuay4h = 254;
		speedGocQuay0h = 230;
	}
	else if (speedGocQuay0h < 4) {
		if(abs(erGocQuay0h) == 0) speedGocQuay0h = 2;
		else speedGocQuay0h = 4;
	}
	
	if(erGocQuay0h > 0) motor0hRo(-speedGocQuay0h);
	else motor0hRo(speedGocQuay0h);
	
	lastErGocQuay0h = erGocQuay0h;
}
void quayGoc4h(int goc){
	erGocQuay4h = (goc - en_4hRo());
	KpGocQuay4h = erGocQuay4h * 1.2;
	KiGocQuay4h = (KiGocQuay4h + erGocQuay4h) * 0.1; //0.01
	KdGocQuay4h = (erGocQuay4h - lastErGocQuay4h) * 0.5; //0.1
	speedGocQuay4h = abs(KpGocQuay4h + KiGocQuay4h + KdGocQuay4h);
	
//	speedGocQuay = abs(1.3 * erGocQuay);
	
	if (speedGocQuay4h > 230) {
//		if(abs(erGocQuay4h) < 200) speedGocQuay4h = 50;
//		else speedGocQuay4h = 254;
		speedGocQuay4h = 230;
	}
	else if (speedGocQuay4h < 4) {
		if(abs(erGocQuay4h) == 0) speedGocQuay4h = 2;
		else speedGocQuay4h = 4;
	}
	
	if(erGocQuay4h > 0) motor4hRo(-speedGocQuay4h);
	else motor4hRo(speedGocQuay4h);
	
	lastErGocQuay4h = erGocQuay4h;
}
void quayGoc8h(int goc){
	erGocQuay8h = (goc - en_8hRo());
	KpGocQuay8h = erGocQuay8h * 1.2;
	KiGocQuay8h = (KiGocQuay8h + erGocQuay8h) * 0.1; //0.01
	KdGocQuay8h = (erGocQuay8h - lastErGocQuay8h) * 0.5; //0.1
	speedGocQuay8h = abs(KpGocQuay8h + KiGocQuay8h + KdGocQuay8h);
	
//	speedGocQuay = abs(1.3 * erGocQuay);
	
	if (speedGocQuay8h > 230) {
//		if(abs(erGocQuay4h) < 200) speedGocQuay4h = 50;
//		else speedGocQuay4h = 254;
		speedGocQuay8h = 230;
	}
	else if (speedGocQuay8h < 4) {
		if(abs(erGocQuay8h) == 0) speedGocQuay8h = 2;
		else speedGocQuay8h = 4;
	}
	
	if(erGocQuay8h > 0) motor8hRo(-speedGocQuay8h);
	else motor8hRo(speedGocQuay8h);
	
	lastErGocQuay8h = erGocQuay8h;
}

float modulo(float dividend, float divisor) { //ham chia co du va gia tri luon > 0
    return dividend - (divisor * (int)(dividend / divisor));
}
float signum(float x) { //Ham tra ve gia tri am, duong hoac 0
    if (x > 0.0) return 1.0;
    else if (x < 0.0) return -1.0;
    else return 0.0;
}
int closestAngle(int gocHienTai, int gocTuongLai){
	// Xem khoang cach giua 2 goc
	int dir = modulo(gocTuongLai, 3600) - modulo(gocHienTai, 3600);

	// Chuyen goc tu -360 -> 360 thanh -180 -> 180
	if (abs(dir) > 1800){
		dir = -(signum(dir) * 3600) + dir;
	}
	return dir;
}
//================================Chay dao chieu dong co
/*
void setDirection0h(int setpoint)
{
		float currentAngle = en_0hRo();
		// find closest angle to setpoint
		float setpointAngle = closestAngle(currentAngle, setpoint);
		// find closest angle to setpoint + 1800
		float setpointAngleFlipped = closestAngle(currentAngle, setpoint + 1800);
		// if the closest angle to setpoint is shorter
		if (abs(setpointAngle) <= abs(setpointAngleFlipped))
		{
			// unflip the motor direction use the setpoint
			mor_0h_next;
			quayGoc0h(currentAngle + setpointAngle);
		}
		// if the closest angle to setpoint + 180 is shorter
		else
		{
			// flip the motor direction and use the setpoint + 180
			mor_0h_back;
			quayGoc0h(currentAngle + setpointAngleFlipped);
		}
}
void setDirection4h(int setpoint)
{
		float currentAngle = en_4hRo();
		// find closest angle to setpoint
		float setpointAngle = closestAngle(currentAngle, setpoint);
		// find closest angle to setpoint + 1800
		float setpointAngleFlipped = closestAngle(currentAngle, setpoint + 1800);
		// if the closest angle to setpoint is shorter
		if (abs(setpointAngle) <= abs(setpointAngleFlipped))
		{
			// unflip the motor direction use the setpoint
			mor_4h_next;
			quayGoc4h(currentAngle + setpointAngle);
		}
		// if the closest angle to setpoint + 180 is shorter
		else
		{
			// flip the motor direction and use the setpoint + 180
			mor_4h_back;
			quayGoc4h(currentAngle + setpointAngleFlipped);
		}
}
void setDirection8h(int setpoint)
{
		float currentAngle = en_8hRo();
		// find closest angle to setpoint
		float setpointAngle = closestAngle(currentAngle, setpoint);
		// find closest angle to setpoint + 1800
		float setpointAngleFlipped = closestAngle(currentAngle, setpoint + 1800);
		// if the closest angle to setpoint is shorter
		if (abs(setpointAngle) <= abs(setpointAngleFlipped))
		{
			// unflip the motor direction use the setpoint
			mor_8h_next;
			quayGoc8h(currentAngle + setpointAngle);
		}
		// if the closest angle to setpoint + 180 is shorter
		else
		{
			// flip the motor direction and use the setpoint + 180
			mor_8h_back;
			quayGoc8h(currentAngle + setpointAngleFlipped);
		}
}
*/
//================================Chay ma khong dao chieu dong co
void setDirection0h(int setpoint)
{
	mor_0h_next;
	float currentAngle = en_0hRo();
	// find closest angle to setpoint
	float setpointAngle = closestAngle(currentAngle, setpoint);
	quayGoc0h(currentAngle + setpointAngle);
}
void setDirection4h(int setpoint)
{
	mor_4h_next;
	float currentAngle = en_4hRo();
	// find closest angle to setpoint
	float setpointAngle = closestAngle(currentAngle, setpoint);
	quayGoc4h(currentAngle + setpointAngle);
}
void setDirection8h(int setpoint)
{
	mor_8h_next;
	float currentAngle = en_8hRo();
	// find closest angle to setpoint
	float setpointAngle = closestAngle(currentAngle, setpoint);
	quayGoc8h(currentAngle + setpointAngle);
}

void quayGoc3Banh(int goc0h, int goc4h, int goc8h){
	setDirection0h(goc0h);
	setDirection4h(goc4h);
	setDirection8h(goc8h);
}
void chay3Banh(int speed0h, int speed4h, int speed8h){
//	if(speed2h > 0) mor_2h_next;
//	else mor_2h_back;
//	
//	if(speed4h > 0) mor_4h_next;
//	else mor_4h_back;
//	
//	if(speed8h > 0) mor_8h_next;
//	else mor_8h_back;
//	
//	if(speed10h > 0) mor_10h_next;
//	else mor_10h_back;
	
	mor_0h = speed0h;
	mor_4h = speed4h;
	mor_8h = speed8h;
}
//void chay4BanhXoay(int tocDo){
//	motor2hRo(tocDo);
//	motor4hRo(tocDo);
//	motor8hRo(tocDo);
//	motor10hRo(tocDo);
//}

void veHome(){
	int pwmGoHome = 50;
	int pwmBackHome = 10;
	
	motor0hRo(-pwmGoHome);
	motor4hRo(-pwmGoHome);
	motor8hRo(-pwmGoHome);
	
	int bien1 = 0, bien2 = 0, bien3 = 0;
	while(1){
		if(!CB_0h) motor0hRo(2), bien1 = 1;
		if(!CB_4h) motor4hRo(2), bien2 = 1;
		if(!CB_8h) motor8hRo(2), bien3 = 1;
		if(bien1 && bien2 && bien3) break;
		osDelay(1);
	}
	motor0hRo(-50);
	motor4hRo(-50);
	motor8hRo(-50);
	
	osDelay(50);
	
	ResetEncoder_FULL();

	bien1 = 0, bien2 = 0, bien3 = 0;
	while(1){
		int xungEn = 100;
		if(en_0hRo() >= xungEn) motor0hRo(2), bien1 = 1;
		if(en_4hRo() >= xungEn) motor4hRo(2), bien2 = 1;
		if(en_8hRo() >= xungEn) motor8hRo(2), bien3 = 1;
		if(bien1 && bien2 && bien3) break;
		osDelay(1);
	}
	motor0hRo(pwmBackHome);
	motor4hRo(pwmBackHome);
	motor8hRo(pwmBackHome);
	
	bien1 = 0, bien2 = 0, bien3 = 0;
	while(1){
		if(!CB_0h) motor0hRo(2), bien1 = 1;
		if(!CB_4h) motor4hRo(2), bien2 = 1;
		if(!CB_8h) motor8hRo(2), bien3 = 1;
		if(bien1 && bien2 && bien3) break;
		osDelay(1);
	}
	osDelay(50);
	ResetEncoder_FULL();
	
//	motor0hRo(-pwmBackHome);
//	motor4hRo(-pwmBackHome);
//	motor8hRo(-pwmBackHome);
//	
//	osDelay(5);
	
//	bien1 = 0, bien2 = 0, bien3 = 0;
//	while(1){
//		int buEn0h = 0, buEn4h = 0, buEn8h = 0;
//		
//		if(en_0hRo() <= -buEn0h) motor0hRo(2), bien1 = 1;
//		if(en_4hRo() <= -buEn4h) motor4hRo(2), bien2 = 1;
//		if(en_8hRo() <= -buEn8h) motor8hRo(2), bien3 = 1;

//		if(bien1 && bien2 && bien3) break;
//		osDelay(1);
//	}		
//	osDelay(50);
//	ResetEncoder_Home();
	
	bitNgatBanh0h = 0;
	bitNgatBanh4h = 0;
	bitNgatBanh8h = 0;
	
	bit_veHome = 0;
}
//void chuongTrinhChay(){
//	while(1){
//		if(en_Chay2h() < 1600) robotRun(40, 100);
//		else robotRun(40, 20);
//		
//		if(en_Chay2h() > 1830) break;
//		osDelay(1);
//	}
//	robotStop(0);
//	osDelay(1000);
//	ResetEncoder_chay();
//	while(1){
//		if(en_Chay2h() < 1300) robotRun(0, 100);
//		else robotRun(0, 20);
//		
//		if(en_Chay2h() > 1500) break;
//		osDelay(1);
//	}
//	robotStop(0);
//	osDelay(1000);
//	
//	while(1){
//		runAngle(870, 50, 20);
//		if(robotAngle() > 880) break;
//		osDelay(1);
//	}
//	
//	ResetEncoder_chay();
//	while(1){
//		if(en_Chay2h() < 1300) robotRun(0, 100);
//		else robotRun(0, 20);
//		
//		if(en_Chay2h() > 1500) break;
//		osDelay(1);
//	}
//	robotStop(0);
//	osDelay(1000);

//	ResetEncoder_chay();
//	while(1){
//		if(en_Chay2h() < 1000) robotRun(90, 100);
//		else robotRun(90, 20);
//		
//		if(en_Chay2h() > 1200) break;
//		osDelay(1);
//	}
//	robotStop(0);	
//	
//	ResetEncoder_chay();
//	while(1){
//		runSnake(90, 50, 50);
//		
//		if(robotAngle() > 1800) break;
//		osDelay(1);
//	}

//	ResetEncoder_chay();
//	while(1){
//		if(en_Chay2h() < 2000) robotRun(90, 100);
//		else robotRun(90, 20);
//		
//		if(en_Chay2h() > 2200) break;
//		osDelay(1);
//	}
//	robotStop(0);
//	
//	osDelay(1000);
//}
//void chayThuNghiem(){
//	if(!O){
//		chuongTrinhChay();
//	}
//	else gamePad();
//}

// Hàm tính toán góc và d? dài du?ng chéo
void calculateJoystickOutput(int x, int y) {
    // Tính toán góc t? tr?c Y và theo chi?u kim d?ng h?
    int raw_angle = (int)(atan2(x, y) * (1800.0 / 3.14)); // Nhân 10 d? tang d? chính xác
    if (raw_angle < 0) {
        raw_angle += 3600; // Chuy?n d?i t? kho?ng -1800-1800 thành 0-3600
    }
    angle = raw_angle; // Ðua v? kho?ng 0-360 d?

    // Tính toán d? dài du?ng chéo (magnitude)
    magnitude = (int)sqrt(x * x + y * y); // Magnitude s? là giá tr? nguyên
}
void gamePad(){
	int mapToc = map(magnitude,0,140,0,255);
	if(magnitude > 10){
		if((int8_t)RJOY_LR >= 50) runSnake(angle, mapToc, mapToc/2);
		else if((int8_t)RJOY_LR <= -50) runSnake(angle, mapToc, -(mapToc/2));
		else robotRun(angle, mapToc);
	}
	else {
		if((int8_t)RJOY_LR >= 50) runSnake(0, 0, 50);
		else if((int8_t)RJOY_LR <= -50) runSnake(0, 0, -50);
		else robotStop(0);
	}
	
//	int omega = 0, vantoc = 0, tocdo = 0;
//	if(L1) tocdo = 250;
//	else tocdo = 50;
//	
//	//if(!R1) angle2h = 0, angle4h = 0, angle8h = 0, angle10h = 0;
//	
//	int stop = 0;
//	if(UP && !DOWN && !RIGHT && !LEFT ) {
//		if((int8_t)RJOY_LR >= 100) runSnake(0, tocdo, 30);
//		else if((int8_t)RJOY_LR <= -100) runSnake(0, tocdo, -30);
//		else robotRun(0, tocdo);
//		//robotRun(0, tocdo);
//	}
//	else if(!UP && DOWN && !RIGHT && !LEFT) {
//		if((int8_t)RJOY_LR >= 100) runSnake(1800, tocdo, 30);
//		else if((int8_t)RJOY_LR <= -100) runSnake(1800, tocdo, -30);
//		else robotRun(1800, tocdo);
//		//robotRun(1800, tocdo);
//	}
//	else if(!UP && !DOWN && RIGHT && !LEFT) {
//		if((int8_t)RJOY_LR >= 100) runSnake(900, tocdo, 30);
//		else if((int8_t)RJOY_LR <= -100) runSnake(900, tocdo, -30);
//		else robotRun(900, tocdo);
//		//robotRun(900, tocdo);
//	}
//	else if(!UP && !DOWN && !RIGHT && LEFT) {
//		if((int8_t)RJOY_LR >= 100) runSnake(-900, tocdo, 30);
//		else if((int8_t)RJOY_LR <= -100) runSnake(-900, tocdo, -30);
//		else robotRun(-900, tocdo);
//		//robotRun(-900, tocdo);
//	}
//	
//	else if((int8_t)RJOY_LR >= 100) runSnake(0, 0, tocdo);
//	else if((int8_t)RJOY_LR <= -100) runSnake(0, 0, -tocdo);
//	else robotStop(0), increment = 0, decrement = 0;

}