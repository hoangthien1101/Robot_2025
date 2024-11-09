#include "4SwerveWheel.h"
#define pi 3.1415
int bienTest = 0;
//=================================================
//------ Bien toan cuc de dieu khien ------
//=================================================
int part = 0;
int _2hRO = 0, _4hRO = 0, _8hRO = 0, _10hRO = 0, _speed2h = 0, _speed4h = 0, _speed8h = 0, _speed10h = 0;
int _2h = 0, _4h = 0, _8h = 0, _10h = 0, _giaToc2 = 0, _giaToc4 = 0, _giaToc8 = 0, _giaToc10 = 0;
//==============KhaiBaoTam=====================
int _robotChange = 1, IMURobotRun = 0;
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
float Vx, Vy, A, B, C, D;
float V2hx, V2hy, V4hx, V4hy, V8hx, V8hy, V10hx, V10hy;
float speed2h, speed4h, speed8h, speed10h, angle2h, angle4h, angle8h, angle10h;
//===============================================

// Khai báo c?u trúc PIDController
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float previous_error;
    float maxValue;
    float minValue;
    uint32_t last_time; // Thoi gian cuoi cung tinh toan PID
} PIDController;

// Ð?nh nghia bi?n PIDController
PIDController pidRunStraight;
PIDController pidBamThanhTrai;
PIDController pidBamThanhTraiLui;
PIDController pidGiamTocLzTruoc;

extern int16_t IMU;

int16_t robotAngle(void)
{
	//Goc IMU duoc tinh nhu sau: cung chieu kim dong ho IMU > 0, nguoc chieu IMU < 0
	return -IMU;
}
// Khoi tao PID
void PID_Init(PIDController *pid, float Kp, float Ki, float Kd, float minValue, float maxValue){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->maxValue = maxValue;
    pid->minValue = minValue;
    pid->last_time = HAL_GetTick(); // Khoi tao thoi gian cuoi cung
}

// Tinh toan dt
float calculate_dt(PIDController *pid) {
    uint32_t current_time = HAL_GetTick(); // Lay thoi gian hien tai
    float dt = (current_time - pid->last_time) / 1000.0f; // Tinh toan delta t (s)
    pid->last_time = current_time; // Cap nhat thoi gian cuoi cung
    return dt;
}
// Tinh toan PID
float PID_Compute(PIDController *pid, float setpoint, float measured_value) {
    float dt = calculate_dt(pid); // Tinh toan delta t
    if (dt == 0) dt = 1e-6; // Dam bao dt khong bang 0 de tranh loi chia cho 0

    float error = setpoint - measured_value;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;

    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // Gioi han output trong khoang minValue va maxValue
    if (output > pid->maxValue) {
        output = pid->maxValue;
    } else if (output < pid->minValue) {
        output = pid->minValue;
    }

    return output;
}
// Hàm tang t?c theo hàm mu
//int exponential_acceleration() {
//    return (int)((MAX_SPEED * (exp(increment / (float)ACCELERATION_LIMIT) - 1) / (exp(1) - 1)) + currentSpeed);
//}
int exponential_acceleration() {
    return (int) increment;
}

// Hàm gi?m t?c theo hàm mu
int exponential_deceleration() {
    return (int)(MAX_SPEED * (1-(exp(increment/DECELERATION_LIMIT)-1)/(exp(1)-1)));
}
//====Ham tra ve banh co toc do lon nhat
int findMaxValue() {
    int max_value = _2h;
    int result = 2;

    if (_4h > max_value) {
        max_value = _4h;
        result = 4;
    }

    if (_8h > max_value) {
        max_value = _8h;
        result = 8;
    }

    if (_10h > max_value) {
        max_value = _10h;
        result = 10;
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
	mor_2h_next;
	mor_4h_next;
	mor_8h_next;
	mor_10h_next;
	static char p = 0;
	if(currentSpeed < TARGET_SPEED) {
		if(p++ %2 == 0) {
			if(currentSpeed < 50) currentSpeed++;
			else currentSpeed+=3;
			if(currentSpeed >= TARGET_SPEED) currentSpeed = TARGET_SPEED;
		}
	}
	else if(currentSpeed > TARGET_SPEED) {
		if(p++ %1 == 0){
			if(currentSpeed > 150)	currentSpeed-=2;
			else currentSpeed--;
			if(currentSpeed <= TARGET_SPEED) currentSpeed = TARGET_SPEED;
		}
	}
	
	if(findMaxValue()==2){
		_speed2h = gioiHanTocDo(currentSpeed);
		_speed4h = gioiHanTocDo(currentSpeed - abs(_2h - _4h));
		_speed8h = gioiHanTocDo(currentSpeed - abs(_2h - _8h));
		_speed10h = gioiHanTocDo(currentSpeed - abs(_2h - _10h));
	}
	else if(findMaxValue()==4){
		_speed4h = gioiHanTocDo(currentSpeed);
		_speed2h = gioiHanTocDo(currentSpeed - abs(_4h - _2h));
		_speed8h = gioiHanTocDo(currentSpeed - abs(_4h - _8h));
		_speed10h = gioiHanTocDo(currentSpeed - abs(_4h - _10h));
	}
	else if(findMaxValue()==8){
		_speed8h = gioiHanTocDo(currentSpeed);
		_speed2h = gioiHanTocDo(currentSpeed - abs(_8h - _2h));
		_speed4h = gioiHanTocDo(currentSpeed - abs(_8h - _4h));
		_speed10h = gioiHanTocDo(currentSpeed - abs(_8h - _10h));
	}
	else {
		_speed10h = gioiHanTocDo(currentSpeed);
		_speed2h = gioiHanTocDo(currentSpeed - abs(_10h - _2h));
		_speed4h = gioiHanTocDo(currentSpeed - abs(_10h - _4h));
		_speed8h = gioiHanTocDo(currentSpeed - abs(_10h - _8h));
	}
}
//=======================End cac ham gia toc============================
void tinhToanGoc(float gocXoay, float V, float omega){
	Vx = V * cos(gocXoay * pi / 180);
	Vy = V * sin(gocXoay * pi / 180);
	A = Vx - (omega*L)/2;
	B = Vx + (omega*L)/2;
	C = Vy - (omega*W)/2;
	D = Vy + (omega*L)/2;
	
	//=============Tinh toan goc 2h=============
	V2hx = A; //B - A
	V2hy = D; //C - D
	speed2h = sqrt(pow(A,2) + pow(D,2));
	angle2h = atan2(D,A) * 180 / pi;
	//angle2h = atan2(C,B);
	//===========End tinh toan goc 2h===========
	
	//=============Tinh toan goc 4h=============
	V4hx = A;
	V4hy = C;
	speed4h = sqrt(pow(A,2) + pow(C,2));
	angle4h = atan2(C,A) * 180 / pi;
	//angle4h = atan2(C,A);
	//===========End tinh toan goc 4h===========
	
	//=============Tinh toan goc 8h=============
	V8hx = B; //A - B
	V8hy = C; //D - C
	speed8h = sqrt(pow(B,2) + pow(C,2));
	angle8h = atan2(C,B) * 180 / pi;
	//angle8h = atan2(D,A);
	//===========End tinh toan goc 8h===========
	
	//=============Tinh toan goc 10h=============
	V10hx = B;
	V10hy = D;
	speed10h = sqrt(pow(B,2) + pow(D,2));
	angle10h = atan2(D,B) * 180 / pi;
	//angle10h = atan2(D,B);
	//===========End tinh toan goc 10h===========
	
	_2hRO = angle2h;
	_4hRO = angle4h;
	_8hRO = angle8h;
	_10hRO = angle10h;
	
	_2h = speed2h;
	_4h = speed4h;
	_8h = speed8h;
	_10h = speed10h;
}
void robotStop(int doCung){
	TARGET_SPEED = 2;
}
int findSection(float angle) {
    // Chu?n hóa góc v? kho?ng 0 d?n 360 d?
    float normalizedAngle = fmod(fmod(angle, 360) + 360, 360);

    // Xác d?nh ph?n c?a hình tròn
    if ((normalizedAngle > 337.5 && normalizedAngle <= 360) || (normalizedAngle >= 0 && normalizedAngle <= 22.5)) {
        return 1; // Ph?n 1
    } else if (normalizedAngle <= 67.5) {
        return 2; // Ph?n 2
    } else if (normalizedAngle <= 112.5) {
        return 3; // Ph?n 3
    } else if (normalizedAngle <= 157.5) {
        return 4; // Ph?n 4
    } else if (normalizedAngle <= 202.5) {
        return 5; // Ph?n 5
    } else if (normalizedAngle <= 247.5) {
        return 6; // Ph?n 6
    } else if (normalizedAngle <= 292.5) {
        return 7; // Ph?n 7
    } else {
        return 8; // Ph?n 8
    }
}
void robotRun(int dir,int tocdo){ // Co IMU
	
	TARGET_SPEED = tocdo;
	
	if(_robotChange){
		IMURobotRun = robotAngle();
		_robotChange = 0;
	}
	part = findSection(dir);
	
	int chenhLechTocDo = -PID_Compute(&pidRunStraight, IMURobotRun, robotAngle());
	
	if(part == 1){
		_2h = tocdo + chenhLechTocDo;
		_4h = tocdo + chenhLechTocDo;
		_8h = tocdo - chenhLechTocDo;
		_10h = tocdo - chenhLechTocDo;
	}
	else if(part == 2){
		_2h = tocdo;
		_4h = tocdo + chenhLechTocDo;
		_8h = tocdo;
		_10h = tocdo - chenhLechTocDo;
	}
	else if(part == 3){
		_2h = tocdo - chenhLechTocDo;
		_4h = tocdo + chenhLechTocDo;
		_8h = tocdo + chenhLechTocDo;
		_10h = tocdo - chenhLechTocDo;
	}
	else if(part == 4){
		_2h = tocdo - chenhLechTocDo;
		_4h = tocdo;
		_8h = tocdo + chenhLechTocDo;
		_10h = tocdo;
	}
	else if(part == 5){
		_2h = tocdo - chenhLechTocDo;
		_4h = tocdo - chenhLechTocDo;
		_8h = tocdo + chenhLechTocDo;
		_10h = tocdo + chenhLechTocDo;
	}
	else if(part == 6){
		_2h = tocdo;
		_4h = tocdo - chenhLechTocDo;
		_8h = tocdo;
		_10h = tocdo + chenhLechTocDo;
	}
	else if(part == 7){
		_2h = tocdo + chenhLechTocDo;
		_4h = tocdo - chenhLechTocDo;
		_8h = tocdo - chenhLechTocDo;
		_10h = tocdo + chenhLechTocDo;
	}
	else{
		_2h = tocdo + chenhLechTocDo;
		_4h = tocdo;
		_8h = tocdo - chenhLechTocDo;
		_10h = tocdo;
	}
	
	// Gioi han lai toc do
	if(_2h >= 254)  _2h = 254;
	else if(_2h <= 3) _2h = 3;
	
	if(_4h >= 254)  _4h = 254;
	else if(_4h <= 3) _4h = 3;
	
	if(_8h >= 254)  _8h = 254;
	else if(_8h <= 3) _8h = 3;
	
	if(_10h >= 254)  _10h = 254;
	else if(_10h <= 3) _10h = 3;
	///////////////////////////
	_2hRO = dir;
	_4hRO = dir;
	_8hRO = dir;
	_10hRO = dir;
}
void runStraight(int goc, int tocDoChay){
	if(_robotChange){
		IMURobotRun = robotAngle();
		_robotChange = 0;
	}
	omega = PID_Compute(&pidRunStraight, IMURobotRun, robotAngle());
	tinhToanGoc(goc, tocDoChay, omega);
	chay4Banh(speed2h, speed4h, speed8h, speed10h);
}
void runSnake(int goc, int tocDoChay, int tocDoXoay){
	_robotChange = 1;
	tinhToanGoc(goc, tocDoChay, tocDoXoay);
	chay4Banh(speed2h, speed4h, speed8h, speed10h);
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
void bamThanhLzTrai(int lzTrai, int tocDo){
	int goc;
	if(tocDo < 0){
		goc = PID_Compute(&pidBamThanhTrai, lazer_Trai, lzTrai);
		robotRun(goc + 180, abs(tocDo));
	}
	else {
		goc = PID_Compute(&pidBamThanhTrai, lzTrai, lazer_Trai);
		robotRun(goc, tocDo);
	}
}
void chayGocLzTraiTruoc(int lzTrai, int lzTruoc, int tocDo){
	bienBreak = 0;
	int tocDoPIDLzTruoc;
	while(1){
		if(lazer_Truoc >  lzTruoc){
			tocDoPIDLzTruoc = PID_Compute(&pidGiamTocLzTruoc, lazer_Truoc, lzTruoc);
			if(tocDoPIDLzTruoc > tocDo) tocDoPIDLzTruoc = tocDo;
			bamThanhLzTrai(lzTrai, tocDoPIDLzTruoc);
		}
		else {
			tocDoPIDLzTruoc = PID_Compute(&pidGiamTocLzTruoc, lzTruoc, lazer_Truoc);
			if(tocDoPIDLzTruoc > tocDo) tocDoPIDLzTruoc = tocDo;
			bamThanhLzTrai(lzTrai, -abs(tocDoPIDLzTruoc));
		}
		if(abs(lazer_Truoc -  lzTruoc) <= 0) { bienBreak = 1;
		}
		if(bienBreak == 1) break;
		osDelay(1);
	}
	robotStop(0);
}