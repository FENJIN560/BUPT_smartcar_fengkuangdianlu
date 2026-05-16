/*
 * my_encoder.c
 *
 *  Created on: 2026Äê2ÔÂ3ÈÕ
 *      Author: rue
 */

#include "my_encoder.h"

void encoder_init()
{
    encoder_dir_init(ENCODER_LEFT, ENCODER_LEFT_A, ENCODER_LEFT_B);
    encoder_dir_init(ENCODER_RIGHT, ENCODER_RIGHT_A, ENCODER_RIGHT_B);
    pit_ms_init(CCU60_CH1,5);
}

int16 total_dist = 0;
int16 left_encoder=0;
int16 right_encoder=0;
int16 last_left_encoder=0;
int16 last_right_encoder=0;

void Get_Encoder()
{
    left_encoder = encoder_get_count(ENCODER_LEFT);
    left_encoder = -left_encoder*0.8+last_left_encoder*0.2;
    last_left_encoder = left_encoder;
    encoder_clear_count(ENCODER_LEFT);

    right_encoder = encoder_get_count(ENCODER_RIGHT);
    right_encoder = right_encoder*0.8+last_right_encoder*0.2;
    last_right_encoder = right_encoder;
    encoder_clear_count(ENCODER_RIGHT);

    total_dist += (left_encoder + right_encoder)/2;

}
