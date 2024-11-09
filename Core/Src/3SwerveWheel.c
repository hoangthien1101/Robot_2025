//#include "ala_Config.h"

#include "main.h"
#include "ala_sendData.h"
#include "3SwerveWheel.h"
#define pi 3.1415
int bienTest = 0;
//=================================================
//------ Bien toan cuc de dieu khien ------
//=================================================
int part = 0;
int _0hRO = 0, _4hRO = 0, _8hRO = 0, _speed0h = 0, _speed4h = 0, _speed8h = 0;
int _0h = 0, _4h = 0, _8h = 0, _giaToc0 = 0, _giaToc4 = 0, _giaToc8 = 0;
//==============KhaiBaoTam=====================
int _robotChange = 1, IMURobotRun = 0;
int chenhLechTocDo = 0;
int omega = 0;
int TARGET_SPEED = 0;
int currentSpeed = 0;
#define MAX_SPEED 200          // T?c d? t?i da
#define ACCELERATION_LIMIT 500 // Gi?i h?n gia t?c (tùy ch?nh)
#define DECELERATION_LIMIT 20000 // Gi?i h?n gi?m t?c (tùy ch?nh)
int increment = 0; // Bi?n gia tang d? di?u ch?nh t?c d? tang d?n
int decrement = 0; // Bi?n gi?m d? di?u ch?nh t?c d? gi?m d?n
//===============================================
int initial_angle = 0;
int current_angle = 0;
int deviation_angle = 0;
//==============KhaiCacBienTinhToan==============
#define R 0.71
#define W 1
#define L 1
float V0hx, V0hy, V4hx, V4hy, V8hx, V8hy;
float speed0h, speed4h, speed8h, angle0h, angle4h, angle8h;
//===============================================

// Ð?nh nghia bi?n PIDController
PIDController pidRunStraight;
PIDController pidBamThanhTrai;
PIDController pidBamThanhTraiLui;
PIDController pidGiamTocLzTruoc;

extern int16_t IMU;

int16_t robotAngle(void)
{
	//Goc IMU duoc tinh nhu sau: cung chieu kim dong ho IMU > 0, nguoc chieu IMU < 0
	//return -IMU;
	return -IMU_china;
}
int16_t robotAngle_TQ(void)
{
	//Goc IMU duoc tinh nhu sau: cung chieu kim dong ho IMU > 0, nguoc chieu IMU < 0
	return -IMU_china;
}

// Tinh toan dt
//float calculate_dt(PIDController *pid) {
//    uint32_t current_time = HAL_GetTick(); // Lay thoi gian hien tai
//    float dt = (current_time - pid->last_time) / 1000.0f; // Tinh toan delta t (s)
//    pid->last_time = current_time; // Cap nhat thoi gian cuoi cung
//    return dt;
//}
// Tinh toan PID
float PID_Compute(float Kp, float Ki, float Kd, float minValue, float maxValue, 
                  PIDController *pid, float setpoint, float measured_value) {
    float error = setpoint - measured_value;

    // Tính toán ph?n tích phân
    pid->integral += error;

    // Tính toán ph?n d?o hàm (n?u c?n)
    float derivative = error - pid->previous_error;
    pid->previous_error = error;

    // Tính toán d?u ra PID
    float output = (Kp * error) + (Ki * pid->integral) + (Kd * derivative);

    // Gi?i h?n output trong kho?ng minValue và maxValue
    if (output > maxValue) {
        output = maxValue;
    } else if (output < minValue) {
        output = minValue;
    }

    return output;
}

int exponential_acceleration() {
    return (int) increment;
}

// Hàm gi?m t?c theo hàm mu
int exponential_deceleration() {
    return (int)(MAX_SPEED * (1-(exp(increment/DECELERATION_LIMIT)-1)/(exp(1)-1)));
}
//====Ham tra ve banh co toc do lon nhat
int findMaxValue() {
    int max_value = _0h;
    int result = 0;

    if (_4h > max_value) {
        max_value = _4h;
        result = 4;
    }

    if (_8h > max_value) {
        max_value = _8h;
        result = 8;
    }

    return result;
}
int gioiHanTocDo(int speed){
	if(speed > 250) speed = 250;
	else if(speed <= 3) {
		if(TARGET_SPEED == 2) speed = 2;
		else speed = 3;
	}
	return speed;
}
// C?p nh?t t?c d? d?a trên gia tang
void update_speed() {
//	mor_0h_next;
//	mor_4h_next;
//	mor_8h_next;
	static char p = 0;
	//Toc do hien tai < toc do muc tieu => tang toc
	if(currentSpeed < TARGET_SPEED) {
		//Lam cham qua trinh tang toc so voi ++
		if(p++ %2 == 0) {
			//Toc do be hon 50 thi cho tang toc tu tu +=1, sau khi dat duoc 50 thi tang toc nhanh +=3
			if(currentSpeed < 50) currentSpeed++;
			else currentSpeed+=3;
			//Khi toc do hien tai >= toc do muc tieu thi toc do hien tai = toc do muc tieu
			if(currentSpeed >= TARGET_SPEED) currentSpeed = TARGET_SPEED;
		}
	}
	//Toc do hien tai > toc do muc tieu => giam toc
	else if(currentSpeed > TARGET_SPEED) {
		//Lam cham qua trinh giam toc so voi ++
		if(p++ %1 == 0){
			//Toc do lon hon 150 thi cho giam toc nhanh -=2, sau khi be hon 150 thi giam toc tu tu --
			if(currentSpeed > 150)	currentSpeed-=2;
			else currentSpeed--;
			//Khi toc do hien tai <= toc do muc tieu thi toc do hien tai = toc do muc tieu
			if(currentSpeed <= TARGET_SPEED) currentSpeed = TARGET_SPEED;
		}
	}
	//Ham findMaxValue se tra ve gia tri toc do banh nao la lon nhat
	//Toc do banh lon nhat se = currentSpeed, con toc do cac banh con lai 
	//se phu thuoc vao chenh lech giua banh lon nhat va cac banh con lai
	if(findMaxValue()==0){
		_speed0h = gioiHanTocDo(currentSpeed);
		_speed4h = gioiHanTocDo(currentSpeed - abs(_0h - _4h));
		_speed8h = gioiHanTocDo(currentSpeed - abs(_0h - _8h));
	}
	else if(findMaxValue()==4){
		_speed4h = gioiHanTocDo(currentSpeed);
		_speed0h = gioiHanTocDo(currentSpeed - abs(_4h - _0h));
		_speed8h = gioiHanTocDo(currentSpeed - abs(_4h - _8h));
	}
	else if(findMaxValue()==8){
		_speed8h = gioiHanTocDo(currentSpeed);
		_speed0h = gioiHanTocDo(currentSpeed - abs(_8h - _0h));
		_speed4h = gioiHanTocDo(currentSpeed - abs(_8h - _4h));
	}
}
//=======================End cac ham gia toc============================
void tinhToanGoc(float gocXoay, float V, float omega){
	V0hx = V * cos(gocXoay * pi / 1800);
	V0hy = (V * sin(gocXoay * pi / 1800)) + omega;
	V4hx = V * cos((gocXoay - 1200) * pi / 1800);
	V4hy = (V * sin((gocXoay - 1200) * pi / 1800)) + omega;
	V8hx = V * cos((gocXoay - 2400) * pi / 1800);
	V8hy = (V * sin((gocXoay - 2400) * pi / 1800)) + omega;
	
	//=============Tinh toan goc 0h=============
	speed0h = sqrt(pow(V0hx,2) + pow(V0hy,2));
	angle0h = atan2(V0hy,V0hx) * 1800 / pi;
	//=============Tinh toan goc 4h=============
	speed4h = sqrt(pow(V4hx,2) + pow(V4hy,2));
	angle4h = atan2(V4hy,V4hx) * 1800 / pi;
	//=============Tinh toan goc 8h=============
	speed8h = sqrt(pow(V8hx,2) + pow(V8hy,2));
	angle8h = atan2(V8hy,V8hx) * 1800 / pi;
	
	_0hRO = angle0h;
	_4hRO = angle4h;
	_8hRO = angle8h;
	
	_0h = speed0h;
	_4h = speed4h;
	_8h = speed8h;
}
void robotStop(int doCung){
	//TARGET_SPEED = 2;
	_0h = 2;
	_4h = 2;
	_8h = 2;
}
int findSection(float angle) {
    // Chuan hóa góc v tu 0 do 3600 do
    float normalizedAngle = fmod(fmod(angle, 3600) + 3600, 3600);

    // Xác dinh cac phan cua hình tròn
    if ((normalizedAngle > 3150 && normalizedAngle < 3600) || (normalizedAngle >= 0 && normalizedAngle <= 450)) {
        return 1; // Phan 1
    } else if (normalizedAngle <= 1350) {
        return 2; // Phan 2
    } else if (normalizedAngle <= 2250) {
        return 3; // Phan 3
    } else {
        return 4; // Phan 4
    }
}
void robotRun(int dir,int tocdo){ // Co IMU
	TARGET_SPEED = tocdo;
	//Xem truoc do robot co thay doi goc hay khong
	if(_robotChange){
		IMURobotRun = robotAngle();
		_robotChange = 0;
	}
	//Gan bien part bang truong hop nao tu bien dir
	part = findSection(dir);
	
	chenhLechTocDo = -PID_Compute(0.05, 0, 0.005, -250, 250, &pidRunStraight, IMURobotRun, robotAngle());
	
	if(part == 1){
		_0h = tocdo;
		_4h = tocdo + chenhLechTocDo;
		_8h = tocdo - chenhLechTocDo;
	}
	else if(part == 2){
		_0h = tocdo - chenhLechTocDo;
		_4h = tocdo + chenhLechTocDo;
		_8h = tocdo;
	}
	else if(part == 3){
		_0h = tocdo;
		_4h = tocdo - chenhLechTocDo;
		_8h = tocdo + chenhLechTocDo;
	}
	else if(part == 4){
		_0h = tocdo + chenhLechTocDo;
		_4h = tocdo;
		_8h = tocdo - chenhLechTocDo;
	}
	
	// Gioi han lai toc do
	if(_0h >= 254)  _0h = 254;
	else if(_0h <= 3) _0h = 3;
	
	if(_4h >= 254)  _4h = 254;
	else if(_4h <= 3) _4h = 3;
	
	if(_8h >= 254)  _8h = 254;
	else if(_8h <= 3) _8h = 3;
	
	///////////////////////////
	_0hRO = dir;
	_4hRO = dir - 1200;
	_8hRO = dir + 1200;
//	_0h = tocdo;
//	_4h = tocdo;
//	_8h = tocdo;
}
//void runStraight(int goc, int tocDoChay){
//	if(_robotChange){
//		IMURobotRun = robotAngle();
//		_robotChange = 0;
//	}
//	omega = PID_Compute(&pidRunStraight, IMURobotRun, robotAngle());
//	tinhToanGoc(goc, tocDoChay, omega);
//	//chay4Banh(speed2h, speed4h, speed8h, speed10h);
//}
void runSnake(int goc, int tocDoChay, int tocDoXoay){
	_robotChange = 1;
	tinhToanGoc(goc, tocDoChay, tocDoXoay);
	//chay4Banh(speed2h, speed4h, speed8h, speed10h);
}
void runGrab(){

}

void stop_rotation() {
	// Reset giá tr? d? l?ch v? 0
	deviation_angle = 0;
}

void runAngle(int goc, int tocDoChay, int tocDoXoay){
//	if(_robotChange){
//		IMURobotRun = robotAngle();
//		_robotChange = 0;
//	}
	// Tính toán d? l?ch
	deviation_angle = (goc - robotAngle()) / 10;
	
	// Ði?u ch?nh d? l?ch d? n?m trong kho?ng -180 d?n 180 d?
	if (deviation_angle > 180) {
			deviation_angle -= 360;
	} else if (deviation_angle < -180) {
			deviation_angle += 360;
	}
	
	runSnake(deviation_angle, tocDoChay, tocDoXoay);
}
//void bamThanhLzTrai(int lzTrai, int tocDo){
//	int goc;
//	if(tocDo < 0){
//		goc = PID_Compute(&pidBamThanhTrai, lazer_Trai, lzTrai);
//		robotRun(goc + 180, abs(tocDo));
//	}
//	else {
//		goc = PID_Compute(&pidBamThanhTrai, lzTrai, lazer_Trai);
//		robotRun(goc, tocDo);
//	}
//}
//void chayGocLzTraiTruoc(int lzTrai, int lzTruoc, int tocDo){
//	bienBreak = 0;
//	int tocDoPIDLzTruoc;
//	while(1){
//		if(lazer_Truoc >  lzTruoc){
//			tocDoPIDLzTruoc = PID_Compute(&pidGiamTocLzTruoc, lazer_Truoc, lzTruoc);
//			if(tocDoPIDLzTruoc > tocDo) tocDoPIDLzTruoc = tocDo;
//			bamThanhLzTrai(lzTrai, tocDoPIDLzTruoc);
//		}
//		else {
//			tocDoPIDLzTruoc = PID_Compute(&pidGiamTocLzTruoc, lzTruoc, lazer_Truoc);
//			if(tocDoPIDLzTruoc > tocDo) tocDoPIDLzTruoc = tocDo;
//			bamThanhLzTrai(lzTrai, -abs(tocDoPIDLzTruoc));
//		}
//		if(abs(lazer_Truoc -  lzTruoc) <= 0) { bienBreak = 1;
//		}
//		if(bienBreak == 1) break;
//		osDelay(1);
//	}
//	robotStop(0);
//}