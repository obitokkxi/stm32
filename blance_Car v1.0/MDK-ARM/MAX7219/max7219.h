// max7219.h

#ifndef __MAX7219_H
#define __MAX7219_H

#include "main.h" // 包含你的主头文件，它通常会引入stm32xxx_hal.h

// ============ 1. 用户配置区 ============
// 请根据你的硬件接线修改这里的宏定义

//extern SPI_HandleTypeDef hspi2; // 声明你在main.c中定义的SPI句柄

//#define MAX7219_SPI_HANDLE  hspi2        // 使用的SPI句柄
#define MAX7219_CS_PORT     GPIOA        // CS片选引脚的GPIO端口
#define MAX7219_CS_PIN      GPIO_PIN_8   // CS片选引脚的GPIO引脚号
// =======================================


// ============ 2. 函数声明区 ============

/**
 * @brief  初始化MAX7219芯片
 * @param  None
 * @retval None
 */
void MAX7219_Init(void);

/**
 * @brief  设置MAX7219的显示亮度
 * @param  brightness: 亮度值，范围从 0x00 (最暗) 到 0x0F (最亮)
 * @retval None
 */
void MAX7219_SetBrightness(uint8_t brightness);

/**
 * @brief  清空点阵屏幕（全部熄灭）
 * @param  None
 * @retval None
 */
void MAX7219_Clear(void);

/**
 * @brief  在点阵上显示一个8x8的图案
 * @param  pattern: 指向一个包含8个字节图案数据的数组指针
 * @retval None
 */
void MAX7219_DisplayPattern(const uint8_t* pattern);


#endif /* __MAX7219_H */
