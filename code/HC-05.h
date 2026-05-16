//
// Created by rue on 2025/11/21.
//

#ifndef STM32FUYA_CAR_HC_05_H
#define STM32FUYA_CAR_HC_05_H

#include "zf_common_headfile.h"

#define UART_BUF_SIZE 1024
#define PACK_HEAD   0xA5 //閸栧懎銇�
#define PACK_TAIL   0x5A //閸栧懎鐔�

//閺佺増宓佺猾璇茬�锋稉顏呮殶閿涘瞼鏁ら幋鐤殰瀹稿崬鐣炬稊锟�
#define RX_BOOL_NUM   3
#define RX_CHAR_NUM   1
#define RX_INT_NUM    2
#define RX_FLOAT_NUM  6

//閸氬嫭鏆熼幑顔捐閸ㄥ鐡ч懞鍌涙殶
#define BOOL_BYTES   ((RX_BOOL_NUM + 7) >> 3)
#define CHAR_BYTES   (RX_CHAR_NUM)
#define INT_BYTES    (RX_INT_NUM * 4)
#define FLOAT_BYTES  (RX_FLOAT_NUM * 4)

#define PACK_DATA_LEN  (BOOL_BYTES + CHAR_BYTES + INT_BYTES + FLOAT_BYTES)
#define PACK_LEN       (1 + PACK_DATA_LEN + 1 + 1) //閺佺増宓侀崠鍛摟閼哄倿鏆辨惔锟�

//鐎规矮绠熼弫鐗堝祦閸栧懐绮ㄩ弸鍕秼
typedef struct
{
    uint8_t bools[RX_BOOL_NUM];
    char chars[RX_CHAR_NUM];
    int32_t integers[RX_INT_NUM];
    float floats[RX_FLOAT_NUM];
} RxPack;

void HC_05_init();
uint8_t ParsePacket(RxPack *p);
void get_data_BT();

extern uint8_t rx_temp_byte;
extern uint8_t uart_rxbuf[UART_BUF_SIZE];
extern volatile uint16_t rx_write;
extern RxPack rxPack;

extern uint32 fifo_data_count;
extern fifo_struct uart_data_fifo;
extern uint8_t fifo_get_data[UART_BUF_SIZE];

#endif //STM32FUYA_CAR_HC_05_H
