#include "HCSR04.h"
#include "tim.h" // 必须包含这个，因为我们要用 htim3


float g_Distance = 999.0f; // 初始化为一个很大的安全值

// --- 1. 毫秒级延时 (使用 TIM3 做秒表) ---
void delay_us(uint16_t us)
{
    // 归零计数器
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    
    // 启动计数 (如果是第一次其实只需要 Start 一次，但为了保险放在这里)
    // 实际上建议在 Init 里 Start，这里只负责读
    
    // 等待计数器达到 us
    while (__HAL_TIM_GET_COUNTER(&htim3) < us);
}

// --- 2. 初始化 ---
void SR04_Init(void)
{
    // 开启定时器 (让它开始默默数数)
    HAL_TIM_Base_Start(&htim3);
    
    // 确保 Trig 拉低
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

// --- 3. 读取距离 (使用 TIM3 计时) ---
float SR04_Read(void)
{
    float distance = 0;
    uint32_t time_us = 0;

    // A. 发送触发信号 (15us 高电平)
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(15); 
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    // B. 等待 Echo 变高 (超时处理)
    // 这里也可以用 TIM3 做超时保护
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
    {
        // 如果等了 10ms (10000us) 还没高电平，说明传感器没插好
        if(__HAL_TIM_GET_COUNTER(&htim3) > 10000) return -1.0; 
    }

    // C. 【关键】开始测量高电平时间
    // 1. 记下开始时刻 (或者直接清零)
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    
    // 2. 等待 Echo 变低
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
    {
        // 超时保护：如果超过 50ms (约17米)，强制退出
        if(__HAL_TIM_GET_COUNTER(&htim3) > 50000) return 0.0; 
    }
    
    // 3. 读取结束时刻 (因为我们清零过，现在的值就是持续时间)
    time_us = __HAL_TIM_GET_COUNTER(&htim3);

    // D. 计算距离
    // 公式：距离 = 时间 * 340m/s / 2 
    // 简化：cm = us / 58
    distance = (float)time_us / 58.0f;
	 if(distance<2.0f){
		distance=2.0f;
	 }	
    return distance;
}