/*
 * gyroscope.c
 *
 *  Created on: 2025年11月15日
 *      Author: zheng
 */

#include "gyroscope.h"

First_Complement first_complement;
gyro_param_t Gyro_Offset;   // 陀螺仪零漂参数结构体
IMU_param_t IMU_Data;       // 去除零漂后的IMU数据结构体
extKalman_t Kalman1;
extKalman_t Kalman2;
extKalman_t Zero;

float Pitch_Max = 0;
float Pitch_Min = 0;
float Roll_Max = 0;
float Roll_Min = 0;
float Max_Delta_Pitch = 0;
float Max_Delta_Roll = 0;

void First_complement()
{
    float k = 0.85;
    static float Pitch_Temp = 0;
    static float Roll_Temp = 0;
    static float Delta_Roll = 0;
    static char Pitch_Flag = 0;
    static char Roll_Flag = 0;

    Pitch_Temp = (atan(IMU_Data.acc_y / IMU_Data.acc_z)) * 180 / 3.1415926535f;
    Roll_Temp = (atan(IMU_Data.acc_x / IMU_Data.acc_z)) * 180 / 3.1415926535f;

    Pitch_Temp = Pitch_Temp + IMU_Data.gyro_x * 0.005f;
    Roll_Temp = Roll_Temp + IMU_Data.gyro_y * 0.005f;

    first_complement.angle.pitch = k * Pitch_Temp + (1 - k) * (first_complement.angle.pitch + IMU_Data.gyro_x * 0.005f);
    first_complement.angle.roll = k * Roll_Temp + (1 - k) * (first_complement.angle.roll + IMU_Data.gyro_y * 0.005f);

    if (Delta_Roll < 0.1 && Delta_Roll > -0.1)
    {
        first_complement.angle.roll = first_complement.angle.roll - Delta_Roll;
    }

    first_complement.angle.roll = KalmanFilter(&Kalman1, first_complement.angle.roll);
    first_complement.angle.pitch = KalmanFilter(&Kalman2, first_complement.angle.pitch);

    if (Pitch_Flag == 0)
    {
        Pitch_Min = first_complement.angle.roll;
        Pitch_Max = first_complement.angle.roll;

        Pitch_Flag = 1;
    }

    if (Roll_Flag == 0)
    {
        Roll_Min = first_complement.angle.pitch;
        Roll_Max = first_complement.angle.pitch;

        Roll_Flag = 1;
    }

    if (My_abs(Pitch_Min) > My_abs(first_complement.angle.roll))
    {
        Pitch_Min = My_abs(first_complement.angle.roll);
    }
    else if (My_abs(Pitch_Max) < My_abs(first_complement.angle.roll))
    {
        Pitch_Max = My_abs(first_complement.angle.roll);
    }

    if (My_abs(Roll_Min) > My_abs(first_complement.angle.pitch))
    {
        Roll_Min = My_abs(first_complement.angle.pitch);
    }
    else if (My_abs(Roll_Max) < My_abs(first_complement.angle.pitch))
    {
        Roll_Max = My_abs(first_complement.angle.pitch);
    }
    Max_Delta_Pitch = Pitch_Max - Pitch_Min;
    Max_Delta_Roll = Roll_Max - Roll_Min;
}

float gyro_Offset_flag = 0;

float IMU_gyro_Offset_Init()
{
    Gyro_Offset.Xdata = 0;
    Gyro_Offset.Ydata = 0;
    Gyro_Offset.Zdata = 0;

    for (uint16_t i = 0; i < 100; i++)
    {
        Gyro_Offset.Xdata += imu660ra_gyro_x;
        Gyro_Offset.Ydata += imu660ra_gyro_y;
        Gyro_Offset.Zdata += imu660ra_gyro_z;
        system_delay_ms(5);
    }

    Gyro_Offset.Xdata /= 1000;
    Gyro_Offset.Ydata /= 1000;
    Gyro_Offset.Zdata /= 1000;

    return gyro_Offset_flag = 1;
}

void IMU_GetValues()
{
    imu660ra_get_gyro();
    imu660ra_get_acc();

    IMU_Data.gyro_x = ((float)imu660ra_gyro_x - Gyro_Offset.Xdata) * PI / 180 / 16.4f;
    IMU_Data.gyro_y = ((float)imu660ra_gyro_y - Gyro_Offset.Ydata) * PI / 180 / 16.4f;
    IMU_Data.gyro_z = ((float)imu660ra_gyro_z - Gyro_Offset.Zdata) * PI / 180 / 16.4f;

    IMU_Data.acc_x = (((float)imu660ra_acc_x) * 0.3f) + IMU_Data.acc_x * 0.7f;
    IMU_Data.acc_y = (((float)imu660ra_acc_y) * 0.3f) + IMU_Data.acc_y * 0.7f;
    IMU_Data.acc_z = (((float)imu660ra_acc_z) * 0.3f) + IMU_Data.acc_z * 0.7f;
}

float Daty_Z = 0;
float Daty_X = 0;
float Daty_Y = 0;

void IMU_YAW_integral()
{
    if (IMU_Data.gyro_z < 0.015 && IMU_Data.gyro_z > -0.015)
    {
        Daty_Z -= 0;
    }
    else
    {
        IMU_Handle_180();
        IMU_Handle_360();
        IMU_Handle_0();
    }

    if (IMU_Data.gyro_x < 0.015 && IMU_Data.gyro_x > -0.015)
    {
        Daty_X -= 0;
    }
    if (IMU_Data.gyro_y < 0.015 && IMU_Data.gyro_y > -0.015)
    {
        Daty_Y -= 0;
    }
}

void IMU_Handle_180()
{
    Daty_Z -= RAD_TO_ANGLE(IMU_Data.gyro_z * 0.005);

    if ((Daty_Z > 0 && Daty_Z <= 180) || (Daty_Z < 0 && Daty_Z >= (-180)))
    {
        Daty_Z = +Daty_Z;
    }
    else if (Daty_Z > 180 && Daty_Z <= 360)
    {
        Daty_Z -= 360;
    }
    else if (Daty_Z < (-180) && Daty_Z >= (-360))
    {
        Daty_Z += 360;
    }
}

char Round = 0;
float yaw_limit_360 = 0;

void IMU_Handle_360()
{
    if (Round == 0)
    {
        yaw_limit_360 -= RAD_TO_ANGLE(IMU_Data.gyro_z * 0.005);
        if (yaw_limit_360 > 360)
        {
            yaw_limit_360 = 360;
            Round = 1;
        }
        else if (yaw_limit_360 < -360)
        {
            yaw_limit_360 = -360;
            Round = 1;
        }
    }
    if (Round == 1)
    {
        if (yaw_limit_360 <= 360 && yaw_limit_360 >= -360)
        {
            yaw_limit_360 += RAD_TO_ANGLE(IMU_Data.gyro_z * 0.005);
            if (yaw_limit_360 > 360)
            {
                yaw_limit_360 = 360;
                Round = 0;
            }
        }
        else if (yaw_limit_360 >= -360)
        {
            yaw_limit_360 -= RAD_TO_ANGLE(IMU_Data.gyro_z * 0.005);
            if (yaw_limit_360 < -360)
            {

                yaw_limit_360 = -360;
                Round = 0;
            }
        }
    }
}

float yaw_total = 0;

void IMU_Handle_0()
{
    yaw_total -= RAD_TO_ANGLE(IMU_Data.gyro_z * 0.005);
}

float fast_sqrt(float num)
{
    float halfx = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);

    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float My_abs(float x)
{
    return x < 0 ? -x : x;
}

void KalmanCreate(extKalman_t *p, float T_Q, float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

float KalmanFilter(extKalman_t *p, float dat)
{
    p->X_mid = p->A * p->X_last;
    p->P_mid = p->A * p->P_last + p->Q;
    p->kg = p->P_mid / (p->P_mid + p->R);
    p->X_now = p->X_mid + p->kg * (dat - p->X_mid);
    p->P_now = (1 - p->kg) * p->P_mid;
    p->P_last = p->P_now;
    p->X_last = p->X_now;
    return p->X_now;
}

void gyro_init(void)
{
    imu660ra_init();
    IMU_gyro_Offset_Init();
}

void gyro_proc(void)
{
    IMU_GetValues();
    IMU_YAW_integral();
    First_complement();
}
