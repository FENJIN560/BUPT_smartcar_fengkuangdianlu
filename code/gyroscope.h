/*
 * gyroscope.h
 *
 *  Created on: 2025Äê11ÔÂ15ÈÕ
 *      Author: zheng
 */

#ifndef CODE_GYROSCOPE_H_
#define CODE_GYROSCOPE_H_

#include "zf_common_headfile.h"

typedef struct
{
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_param_t;

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMU_param_t;

typedef struct extKalman_t
{
    float X_last;
    float X_mid;
    float X_now;
    float P_mid;
    float P_now;
    float P_last;
    float kg;
    float A;
    float B;
    float Q;
    float R;
    float H;
} extKalman_t;

typedef struct IMU_Original
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
} IMU_Original;

extern IMU_Original IMU_original;

typedef struct Angle
{
    float pitch_temp;
    float roll_temp;
    float pitch;
    float roll;
    float yaw;
} Angle;

typedef struct First_Complement
{
    Angle angle;
} First_Complement;

extern First_Complement first_complement;
extern IMU_param_t IMU_Data;
extern extKalman_t Kalman1;
extern extKalman_t Kalman2;
extern extKalman_t Zero;

extern float gyro_Offset_flag;
extern float Daty_Z;
extern float Daty_X;
extern float Daty_Y;
extern float yaw_limit_360;
extern float yaw_total;
extern int GL_IMU_Flag;
extern float Pitch_Max;
extern float Pitch_Min;
extern float Roll_Max;
extern float Roll_Min;
extern float Max_Delta_Pitch;
extern float Max_Delta_Roll;

void First_complement(void);
void low_pass_filter_init(void);
float low_pass_filter(float value);
float My_abs(float x);
float IMU_gyro_Offset_Init(void);
void IMU_GetValues(void);
void IMU_YAW_integral(void);
void IMU_Handle_180(void);
void IMU_Handle_360(void);
void IMU_Handle_0(void);
float fast_sqrt(float num);
void KalmanCreate(extKalman_t *p, float T_Q, float T_R);
float KalmanFilter(extKalman_t *p, float dat);
void gyro_init(void);
void gyro_proc(void);

#endif /* CODE_GYROSCOPE_H_ */
