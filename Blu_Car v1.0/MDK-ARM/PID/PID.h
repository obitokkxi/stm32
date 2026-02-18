#ifndef __PID_H
#define __PID_H

typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数

    float Target;       // 目标值 (比如 50cm/s)
    float Actual;       // 实际值 (编码器测出来的)
    float Error;        // 当前误差
    float Last_Error;   // 上次误差
    float Sum_Error;    // 误差累计 (积分项)

    float Output;       // 计算出的 PWM 输出
    float Output_Max;   // 输出限幅 (比如 PWM 不能超过 100)
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_out);
float PID_Compute(PID_TypeDef *pid, float actual_val,float target_val);

#endif
