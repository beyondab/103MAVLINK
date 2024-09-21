#ifndef __BOARD_LED_H__
#define __BOARD_LED_H__

#include "main.h"

void Board_LED_ON(void);
void Board_LED_OFF(void);
void Board_LED_Toggle(void);

void Board_LED_task(void * argument);

#endif
