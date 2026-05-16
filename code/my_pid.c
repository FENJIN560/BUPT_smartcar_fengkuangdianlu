/*
 * my_pid.c
 *
 *  Created on: 2026年2月3日
 *      Author: rue
 */

#include "my_pid.h"

PID pid_L;
PID pid_R;
PID pid_turn;
PID pid_Angular;

void Pid_Init(void)
{
    //左电机速度环参数
    pid_L.Kp = 8.0;
    pid_L.Ki = 0.5;
    pid_L.Kd = 0.0;
    pid_L.Target = 30;

    //右电机速度环参数
    pid_R.Kp = 8.0;
    pid_R.Ki = 0.5;
    pid_R.Kd = 0.0;
    pid_R.Target = 30;

    //转向环参数
    pid_turn.Kp = 30;
    pid_turn.Kp2 = 0;
    pid_turn.Kd = 0;
    pid_turn.Kd2 = 0.05;


    //角速度环参数
    pid_Angular.Kp = 0.1;
    pid_Angular.Kd = 0.0;
}


//增量式pid
float Incremental_PID (PID *pid ,int16 target_speed, int16 measured_value )
{
  pid->Err = target_speed - measured_value ;

  pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);
  pid->i_out = pid->Ki * pid->Err;

  pid->Output += pid->p_out + pid->i_out;
  if(pid->Output > 4444)
  {
    pid->Output=4444;
  }
  if(pid->Output<-4444)
  {
    pid->Output=-4444;
  }

  pid->Last_Err =  pid->Err;

  return pid->Output;
}

//位置式pid
float Position_PID(PID *pid ,float gyro,float error )
{
  pid->Err = error;

  pid->p_out = pid->Kp * pid->Err + pid->Kp2 * pid->Err * fabs(pid->Err);
  pid->d_out = pid->Kd * (pid->Err - pid->Last_Err) + (pid->Kd2 * gyro)/100;

  pid->Output= pid->p_out + pid->d_out;

  pid->Last_Err =  pid->Err;

  return pid->Output;
}





