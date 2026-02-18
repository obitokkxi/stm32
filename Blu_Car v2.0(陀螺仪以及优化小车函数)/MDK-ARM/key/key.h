#ifndef __KEY_H
#define __KEY_H

#include "main.h" // 包含你的主头文件，它通常会引入stm32xxx_hal.h

typedef enum {
	 free_state=0,
	 singlepress,
	 doublepress,
	 longpress,
	 releasewait
}keystate;

uint8_t KeyState(void);
void Keyscan(void);

#endif 
