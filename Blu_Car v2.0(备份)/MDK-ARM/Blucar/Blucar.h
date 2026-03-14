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
#include "mpu6050.h"
#include <math.h>
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

//循迹传感器引脚
#define L1_Port Track_L1_GPIO_Port
#define L1_Pin Track_L1_Pin
#define R1_Port Track_R1_GPIO_Port
#define R1_Pin Track_R1_Pin
#define L2_Port Track_L2_GPIO_Port
#define L2_Pin Track_L2_Pin
#define R2_Port Track_R2_GPIO_Port
#define R2_Pin Track_R2_Pin


// 惯性系数 (越大越快，越小越平滑)
// 建议范围: 1 (像开船) ~ 10 (很灵敏)
#define INERTIA_STEP 6
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

#define BASE_PWM      40     // 基础 PWM (稍微给大一点防止倒车不动)

// 2. 陀螺仪参数
#define GYRO_KP_FWD  2.5f   // 前进修正力度
#define GYRO_KP_REV  1.0f   // 后退修正力度 (倒车必须改小，否则必摆！)

#define GYRO_KP  0.2f   // P项：主要纠正力度

#define GYRO_KD  0.1f   // D项：阻尼，防止画龙 (非常重要！)

// 循迹专用参数
// ==========================================
#define BASE_SPEED      40    // 基础速度



// 误差权重
#define ERR_CENTER      0
#define ERR_LOW         0.8f  // 微调
#define ERR_MID         2.0f  // 中度偏离
#define ERR_HIGH        5.0f  // 急转弯
// ==========================================

// 1. 启用开关
#define USE_SPEED_PID  0      // 0=使用纯PWM(开环), 1=使用速度PID(闭环)
// 1. 请根据实测修改这两个值！(非常重要)
// ==========================================
#define DEAD_ZONE_L  25  // 左轮最小启动电压
#define DEAD_ZONE_R  25  // 右轮最小启动电压 (如果右轮更涩，设大一点)



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

// 定义状态
typedef enum {
    STATE_NORMAL = 0,   // 正常巡线
    STATE_LOCK_LEFT,    // 锁定左转 (直角/锐角)
    STATE_LOCK_RIGHT    // 锁定右转
} TrackState_t;

extern Car_Handle_t myCar;
extern MPU6050_t mpu; // 陀螺仪数据结构体
extern PID_TypeDef Motor_PID_L; 
extern PID_TypeDef Motor_PID_R;
extern float speed_L ;         // 当前速度 (cm/s)
extern float speed_R ;  
extern volatile long pulse_count_L;
extern volatile long pulse_count_R ;

// ================== 3. 函数声明 ==================
void Motor_Init(void);                                  // 初始化电机
void Car_Report_Status(void);                           // 发送电量和避障信息
void Car_AutoRun(Car_Handle_t *Car_Set);
void Car_Track_Line(Car_Handle_t *Car_Set,MPU6050_t *gyro);
void Car_Wheelspd(uint8_t cmd, Car_Handle_t *Car_Set, int16_t L, int16_t R);
void Car_Force_Stop(Car_Handle_t *Car_Set);                          // 强制急停 (用于看门狗)
void Car_Getspd(long L_val, long R_val);
void Track_PID_Init(uint8_t P,uint8_t D);
float Car_Speed_Ctrl(float Target, float Actual, PID_TypeDef *pid);
void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle);
float Car_Angle_Ctrl(uint8_t cmd, float Base_Speed);
void Car_Motor_Output(uint8_t cmd, uint8_t Mode, float PWM_L, float PWM_R);

#endif 
