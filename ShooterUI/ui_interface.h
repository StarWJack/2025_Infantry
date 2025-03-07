#ifndef SERIAL_TEST_UI_INTERFACE_H
#define SERIAL_TEST_UI_INTERFACE_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "ui_types.h"
#include "usart.h"

extern int ui_self_id;

void print_message(const uint8_t* message, int length);

#define SEND_MESSAGE(message, len)   HAL_UART_Transmit_DMA(&huart6, message, len)
//由于裁判系统接收限制，你可能需要使用队列或延时来控制发送频率，此时需要自己建立一个新的发送函数替换掉HAL_UART_Transmit_DMA

void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);

#endif //SERIAL_TEST_UI_INTERFACE_H
