/*
 * my_encoder.h
 *
 *  Created on: 2026Äê2ÔÂ3ÈÕ
 *      Author: rue
 */

#ifndef CODE_MY_ENCODER_H_
#define CODE_MY_ENCODER_H_

#include "zf_common_headfile.h"

#define ENCODER_LEFT        TIM4_ENCODER
#define ENCODER_RIGHT       TIM6_ENCODER

#define ENCODER_LEFT_A      TIM4_ENCODER_CH1_P02_8
#define ENCODER_LEFT_B      TIM4_ENCODER_CH2_P00_9

#define ENCODER_RIGHT_A     TIM6_ENCODER_CH1_P20_3
#define ENCODER_RIGHT_B     TIM6_ENCODER_CH2_P20_0

void encoder_init();
void Get_Encoder();

extern int16 left_encoder;
extern int16 right_encoder;
extern int16 total_dist;

#endif /* CODE_MY_ENCODER_H_ */
