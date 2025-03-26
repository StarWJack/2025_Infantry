#ifndef __CONTROL_H__
#define __CONTROL_H__


#include "main.h"

extern uint8_t shoot_test;
extern uint8_t y_1;
extern uint8_t y_2;
extern uint8_t y_3;
extern uint8_t y_4;


extern void ControlTask(void *argument);
void ControlInit(void);



#endif
