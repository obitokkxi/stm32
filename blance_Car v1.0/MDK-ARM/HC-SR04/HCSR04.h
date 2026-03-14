#ifndef __HCSR04_H
#define __HCSR04_H

#include "main.h"

// 定义引脚 (如果在 CubeMX 里没设 Label，就手动修改这里的 Port 和 Pin)
#define TRIG_PORT   SR04_Trig_GPIO_Port
#define TRIG_PIN    SR04_Trig_Pin
#define ECHO_PORT   SR04_Echo_GPIO_Port
#define ECHO_PIN    SR04_Echo_Pin

extern float g_Distance;

// 函数声明
void SR04_Init(void);
float SR04_Read(void);

#endif
