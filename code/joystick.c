#include "joystick.h"

Joystick_t joystick;  // ���ս��ȫ�ֱ���

#define FRAME_HEAD1 0xAA
#define FRAME_HEAD2 0x55
#define DATA_LEN    5   // KEY + LX + LY + RX + RY

static uint8_t rx_buf[DATA_LEN + 1]; // +1 У��
static uint8_t rx_index = 0;
static uint8_t state = 0;

uint8 uart2_rx=0;

void Joystick_Process_Byte(uint8_t b)
{

//    char str[8];
//    sprintf(str, "%02X", b);
//    ips200_show_string(0, 180, str);
    static uint8_t step = 0;
    static uint8_t len  = 0;
    static uint8_t buf[5];
    static uint8_t idx  = 0;
    static uint8_t sum  = 0;

    switch (step)
    {
        case 0: // AA
            if (b == 0xAA) step = 1;
            break;

        case 1: // 55
            if (b == 0x55) step = 2;
            else step = 0;
            break;

        case 2: // LEN
            len = b;
            if (len == 5)
            {
                idx = 0;
                sum = len;   // ? �� LEN �ӽ�У��
                step = 3;
            }
            else
                step = 0;
            break;

        case 3: // DATA
            buf[idx++] = b;
            sum += b;

            if (idx >= len)
                step = 4;
            break;

        case 4: // CHECKSUM
            if (sum == b)
            {
                joystick.KEY = buf[0];
                joystick.LX  = buf[1];
                joystick.LY  = buf[2];
                joystick.RX  = buf[3];
                joystick.RY  = buf[4];
            }
            step = 0;
            break;
    }
    gas = joystick.LY;
    l_r_range = joystick.RX;
}

