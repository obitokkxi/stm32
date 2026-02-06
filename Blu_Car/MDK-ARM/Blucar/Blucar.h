#ifndef __BLUCAR_H
#define __BLUCAR_H

#include "main.h" // 包含你的主头文件，它通常会引入stm32xxx_hal.h
#include "gpio.h"
#include "tim.h"   // 引用定时器
#include "usart.h" // 引用串口
#include "adc.h"   // 引用ADC
#include <stdio.h>
#include <string.h>
#include "HCSR04.h"
#include "PID.h"
//电机方向引脚
#define IN_GPIO_Port GPIOB
#define AIN1_PORT AIN1_GPIO_Port
#define AIN2_PORT AIN2_GPIO_Port
#define BIN1_PORT BIN1_GPIO_Port
#define BIN2_PORT BIN2_GPIO_Port
#define AIN1_PIN AIN1_Pin
#define AIN2_PIN AIN2_Pin
#define BIN1_PIN BIN1_Pin
#define BIN2_PIN BIN2_Pin

//定时器以及通道
#define TIM htim2
#define MOTOR_CHANNEL1 TIM_CHANNEL_1
#define MOTOR_CHANNEL2 TIM_CHANNEL_2

//ADC
#define ADC hadc1

//串口
#define huart huart1

// 惯性系数 (越大越快，越小越平滑)
// 建议范围: 1 (像开船) ~ 10 (很灵敏)
//#define INERTIA_STEP 5
#define DEFAULT_INERTIA_STEP 5 // 惯性系数
#define DEFAULT_MODE 1     // 电机初始模式
#define MIN_MOTOR_PWM  40  // 电机启动最小占空比

// ================= 物理参数定义 =================
#define WHEEL_DIAMETER   6.50f   // 轮子直径 (单位: cm)
#define ENCODER_GRIDS    20      // 码盘孔数
#define SAMPLE_TIME      0.05f   // 采样时间 (单位: 秒, 即50ms)
#define PI               3.14159f

// 定义电机死区 (低于这个值电机不动，强行补偿)
#define DEAD_ZONE 30
// 定义你的电机死区 (根据你的描述，这里至少要填 35 或 40)
// 意思就是：PWM 小于 40 电机根本不动，那我们就把 0-100 映射到 40-100
#define MOTOR_DEAD_ZONE 28

typedef struct {
    // --- 运行状态 ---
    int16_t Target_PWM_L;
    int16_t Target_PWM_R;
    int16_t   Current_PWM_L;
    int16_t   Current_PWM_R;
    uint16_t  Target_Speed;
    // --- 配置参数 ---
	 uint8_t cmd;
    uint8_t Mode; 
    float Inertia_Step;
} Car_Handle_t;

extern Car_Handle_t myCar;
extern PID_TypeDef Motor_PID_L; 
extern PID_TypeDef Motor_PID_R;
extern float speed_L ;         // 当前速度 (cm/s)
extern float speed_R ;  
extern volatile long pulse_count_L;
extern volatile long pulse_count_R ;
// ================== 3. 函数声明 ==================
void Motor_Init(void);                                  // 初始化电机
void Car_Set_Motion(Car_Handle_t *Car_Set); // 设定运动目标
void Car_Speed_Handle(Car_Handle_t *Car_Set);                            // 惯性计算循环
void Car_Report_Status(void);                           // 发送电量和避障信息
void Car_AutoRun(Car_Handle_t *Car_Set);
void Car_Track_Line(Car_Handle_t *Car_Set);
void Car_Wheelspd(uint8_t cmd, Car_Handle_t *Car_Set, int16_t L, int16_t R);
void Car_Force_Stop(Car_Handle_t *Car_Set);                          // 强制急停 (用于看门狗)
float Get_PID_Error(void);
void Car_Run(Car_Handle_t *Car_Set, char cmd, uint16_t speed);
void Set_Motor_Execute(float pwm_l, float pwm_r);
void Car_Getspd(long L_val, long R_val);
void Track_PID_Init(void);
#endif 
