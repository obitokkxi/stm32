// max7219.c

#include "max7219.h"

// MAX7219 内部寄存器地址定义
#define REG_NO_OP           0x00
#define REG_DIGIT_0         0x01
#define REG_DIGIT_1         0x02
#define REG_DIGIT_2         0x03
#define REG_DIGIT_3         0x04
#define REG_DIGIT_4         0x05
#define REG_DIGIT_5         0x06
#define REG_DIGIT_6         0x07
#define REG_DIGIT_7         0x08
#define REG_DECODE_MODE     0x09
#define REG_INTENSITY       0x0A
#define REG_SCAN_LIMIT      0x0B
#define REG_SHUTDOWN        0x0C
#define REG_DISPLAY_TEST    0x0F

/**
 * @brief  向MAX7219发送一个16位的命令 (内部静态函数)
 * @param  reg: 寄存器地址
 * @param  data: 要写入的数据
 * @retval None
 */
static void MAX7219_SendCommand(uint8_t reg, uint8_t data)
{
    // 1. 拉低CS，选中芯片
    HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_RESET);

    // 2. 打包数据并通过SPI发送
    uint8_t buffer[2] = {reg, data};
    //HAL_SPI_Transmit(&MAX7219_SPI_HANDLE, buffer, 2, HAL_MAX_DELAY);

    // 3. 拉高CS，取消选中，命令生效
    HAL_GPIO_WritePin(MAX7219_CS_PORT, MAX7219_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief  初始化MAX7219芯片
 */
void MAX7219_Init(void)
{
    // 设置译码模式：不使用BCD译码，直接控制点阵
    MAX7219_SendCommand(REG_DECODE_MODE, 0x00);
    // 设置扫描限制：扫描所有8行
    MAX7219_SendCommand(REG_SCAN_LIMIT, 0x07);
    // 设置初始亮度：中等亮度
    MAX7219_SetBrightness(0x07);
    // 关闭显示测试模式
    MAX7219_SendCommand(REG_DISPLAY_TEST, 0x00);
    // 开启芯片，退出关断模式
    MAX7219_SendCommand(REG_SHUTDOWN, 0x01);
    // 初始化后清屏
    MAX7219_Clear();
}

/**
 * @brief  设置MAX7219的显示亮度
 */
void MAX7219_SetBrightness(uint8_t brightness)
{
    // 限制亮度值在0x00到0x0F之间
    if (brightness > 0x0F) {
        brightness = 0x0F;
    }
    MAX7219_SendCommand(REG_INTENSITY, brightness);
}

/**
 * @brief  清空点阵屏幕
 */
void MAX7219_Clear(void)
{
    for (uint8_t i = 0; i < 8; i++) {
        // 行寄存器地址从1到8
        MAX7219_SendCommand(REG_DIGIT_0 + i, 0x00);
    }
}

/**
 * @brief  在点阵上显示一个8x8的图案
 */
void MAX7219_DisplayPattern(const uint8_t* pattern)
{
    for (uint8_t i = 0; i < 8; i++) {
        MAX7219_SendCommand(REG_DIGIT_0 + i, pattern[i]);
    }
}