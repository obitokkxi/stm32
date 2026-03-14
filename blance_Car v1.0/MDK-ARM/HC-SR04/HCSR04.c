#include "HCSR04.h"
// ?? 彻底删除了 #include "tim.h"，你的定时器自由了！

float g_Distance = 999.0f; // 初始化为一个很大的安全值

// --- 1. 毫秒级延时 (纯 CPU 循环，不占用任何定时器！) ---
void delay_us(uint32_t us)
{
    // SystemCoreClock 通常是 72000000 (72MHz)
    // 这是一个经过粗略调校的纯软件延时，对超声波完全够用
    uint32_t delay = (SystemCoreClock / 8000000) * us;
    while (delay--) {
        __NOP(); // 空指令，什么都不做，纯耗 CPU 时间
    }
}

// --- 2. 初始化 ---
void SR04_Init(void)
{
    // 不再需要开启 htim3
    
    // 确保 Trig 拉低
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

// --- 3. 读取距离 (使用纯软件计数) ---
float SR04_Read(void)
{
    float distance = 0;
    uint32_t time_us = 0;
    uint32_t timeout = 0;

    // A. 发送触发信号 (15us 高电平)
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(15); 
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // B. 等待 Echo 变高 (超时处理)
    timeout = 0;
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
    {
        delay_us(10);  // 每次死等 10us
        timeout += 10;
        // 如果等了 10ms (10000us) 还没高电平，说明传感器没插好
        if(timeout > 10000) return -1.0; 
    }

    // C. 【关键】开始测量高电平时间
    time_us = 0;
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
    {
        delay_us(10);  // 每次循环消耗 10us
        time_us += 10; // 记录总时间
        
        // 超时保护：如果超过 50ms (约8.5米)，强制退出，防止程序卡死
        if(time_us > 50000) return 0.0; 
    }
    
    // D. 计算距离
    // 公式：距离 = 时间 * 340m/s / 2 
    // 简化：cm = us / 58
    distance = (float)time_us / 58.0f;
    if(distance < 2.0f){
        distance = 2.0f;
    }    
    return distance; // 返回时变量名保持 g_Distance 一致的更新逻辑
}