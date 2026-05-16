//
// Created by rue on 2025/11/21.
//
#include "HC-05.h"  //蓝牙模块部分,用于蓝牙手机调参

uint8_t uart_rxbuf[UART_BUF_SIZE];
uint8_t fifo_get_data[UART_BUF_SIZE];

volatile uint16_t rx_write = 0;
volatile uint16_t rx_read  = 0;

uint8_t rx_temp_byte;

RxPack rxPack;

uint32 fifo_data_count = 0;
fifo_struct uart_data_fifo;

void HC_05_init()
{
    uart_init(UART_0,115200,UART0_TX_P14_0,UART0_RX_P14_1);
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_rxbuf, 64);
    uart_init(UART_2,9600,UART2_TX_P10_5,UART2_RX_P10_6);//9600
    uart_rx_interrupt(UART_2, 1);
    uart_query_byte(UART_2,&rx_temp_byte);
}


static int buffer_available(void)
{
    return (rx_write - rx_read) & (UART_BUF_SIZE - 1);
}

static uint8_t buffer_read(void)
{
    uint8_t b = uart_rxbuf[rx_read];
    rx_read = (rx_read + 1) & (UART_BUF_SIZE - 1); //rx_read++
    return b;
}


static uint8_t buffer_peek_offset(int offset)
{
    return uart_rxbuf[(rx_read + offset) & (UART_BUF_SIZE - 1)];
}

uint8_t ParsePacket(RxPack *p)
{
    while (buffer_available() >= PACK_LEN)
    {

        if (buffer_peek_offset(0) != PACK_HEAD)
        {
            buffer_read();
            continue;
        }


        if (buffer_peek_offset(PACK_LEN - 1) != PACK_TAIL)
        {
            buffer_read();
            continue;
        }

        //-----------------------------------------
        //
        //-----------------------------------------
        buffer_read();


        int bool_bytes = (RX_BOOL_NUM + 7) >> 3;
        uint8_t bool_buf[bool_bytes];


        for (int i = 0; i < bool_bytes; i++)
        {
            bool_buf[i] = buffer_read();
        }


        for (int i = 0; i < RX_BOOL_NUM; i++)
        {
            uint8_t byte_index = i >> 3;
            uint8_t bit_index  = i & 0x07;
            p->bools[i] = (bool_buf[byte_index] >> bit_index) & 0x01;
        }



        for (int i = 0; i < RX_CHAR_NUM; i++)
            p->chars[i] = buffer_read();


        for (int i = 0; i < RX_INT_NUM; i++)
        {
            uint8_t b0 = buffer_read();
            uint8_t b1 = buffer_read();
            uint8_t b2 = buffer_read();
            uint8_t b3 = buffer_read();

            p->integers[i] = (int32_t)(b0 | (b1 << 8) | (b2 << 16) | (b3 << 24));
        }


        for (int i = 0; i < RX_FLOAT_NUM; i++)
        {
            union { uint8_t b[4]; float f; } u;
            u.b[0] = buffer_read();
            u.b[1] = buffer_read();
            u.b[2] = buffer_read();
            u.b[3] = buffer_read();
            p->floats[i] = u.f;
        }

        buffer_read();

        return 1;
    }

    return 0;
}

void get_data_BT()
{
    fifo_data_count = fifo_used(&uart_data_fifo);
    if(fifo_data_count != 0)
    {
       fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 灏� fifo 涓暟鎹鍑哄苟娓呯┖ fifo 鎸傝浇鐨勭紦鍐�
    }

    if(ParsePacket(&rxPack))
    {
      run_flag = rxPack.bools[0];
      fuya_flag = rxPack.bools[1];
      pid_L.Kp = rxPack.floats[0];
      pid_L.Ki = rxPack.floats[1];
      pid_turn.Kp = rxPack.floats[2];
      pid_turn.Kp2 = rxPack.floats[3];
      pid_turn.Kd = rxPack.floats[4];
      pid_turn.Kd2 = rxPack.floats[5];
    }
}


