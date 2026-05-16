/*
 * my_pid.h
 *
 *  Created on: 2026年2月3日
 *      Author: rue
 */

#ifndef CODE_MY_PID_H_
#define CODE_MY_PID_H_

#include "zf_common_headfile.h"

typedef struct{
  float Kp;
  float Ki;
  float Kd;
  float Kp2;
  float Kd2;
  float p_out;
  float i_out;
  float d_out;
  float d2_out;
  float Err;
  float Last_Err;       // �ϴ����
  float Previous_Err;   // ���ϴ����
  float Output;
  float Target;
  float Measure;
}PID;

float Incremental_PID (PID *pid ,int16 target_speed, int16 measured_value );
float Position_PID(PID *pid ,float gyro,float error );

extern PID pid_L;
extern PID pid_R;
extern PID pid_turn;
extern PID pid_Angular;

#endif /* CODE_MY_PID_H_ */
