//Author by Nguyen Nho Phong Vu
//Date version: 17/8/2024 version 1 - 3 Wheel config
//Date version: 17/8/2024 version 2 - 3 Wheel math
#include <stdlib.h>
#include <math.h>

extern int bienTest;
//=================================================
//------ Bien toan cuc de dieu khien ------
//=================================================
extern int _0hRO, _4hRO, _8hRO, _speed0h, _speed4h, _speed8h;
extern int _0h, _4h, _8h, _giaToc0, _giaToc4, _giaToc8;
//==============KhaiBaoTam=====================
extern int _robotChange, IMURobotRun;
extern int omega;
extern int TARGET_SPEED;
extern int curentSpeed;
extern int increment; // Bi?n gia tang d? di?u ch?nh t?c d? tang d?n
extern int decrement; // Bi?n gi?m d? di?u ch?nh t?c d? gi?m d?n
//===============================================
extern int initial_angle;
extern int current_angle;
extern int deviation_angle;
//==============KhaiCacBienTinhToan==============
extern float Vx, Vy, A, B, C, D;
extern float V0hx, V0hy, V4hx, V4hy, V8hx, V8hy;
extern float speed0h, speed4h, speed8h, angle0h, angle4h, angle8h;
//===============================================
extern int16_t IMU, IMU_china;
// pid.h
#ifndef PID_H
#define PID_H

typedef struct {
    float integral;
    float previous_error;
} PIDController;

// Khai báo hàm PID_Compute v?i các tham s? PID tr?c ti?p
float PID_Compute(float Kp, float Ki, float Kd, float minValue, float maxValue, 
                  PIDController *pid, float setpoint, float measured_value);

#endif // PID_H


int16_t robotAngle(void);
int16_t robotAngle_TQ(void);
int parabola_acceleration(int current_speed, int target_speed, int t);
int parabola_deceleration(int current_speed, int target_speed, int t);
void tinhToanGoc(float gocXoay, float V, float omega);
void robotStop(int doCung);
void runStraight(int goc, int tocDoChay);
void runSnake(int goc, int tocDoChay, int tocDoXoay);
void runGrab();
void stop_rotation();
void runAngle(int goc, int tocDoChay, int tocDoXoay);