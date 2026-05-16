/*
 * joystick.h
 *
 *  Created on: 2026年2月9日
 *      Author: rue
 */

#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#include "zf_common_headfile.h"

/* ================= 协议参数 ================= */

#define FRAME_HEAD1   0xAA
#define FRAME_HEAD2   0x55
#define DATA_LEN      5     // KEY + LX + LY + RX + RY

/* ================= 数据结构 ================= */

typedef struct
{
    uint8_t KEY;
    uint8_t LX;
    uint8_t LY;
    uint8_t RX;
    uint8_t RY;
}Joystick_t;

/* ================= 全局变量 ================= */

extern Joystick_t joystick;
extern uint8 uart2_rx;

/* ================= 接口函数 ================= */

void Joystick_Process_Byte(uint8_t b);

#endif


