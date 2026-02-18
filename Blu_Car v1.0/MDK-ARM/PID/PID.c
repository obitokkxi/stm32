#include "pid.h"

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_out) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->Output_Max = max_out;
	 pid->Actual=0;  
    pid->Sum_Error = 0;
    pid->Last_Error = 0;
	
}

float PID_Compute(PID_TypeDef *pid, float actual_val,float target_val) {
    pid->Actual = actual_val;
	 pid->Target=target_val;
    pid->Error = pid->Target - pid->Actual;

    // 1. 积分限幅 (防止长时间偏差导致积分爆炸)
    // 简单的抗饱和处理
    if(pid->Sum_Error > 500) pid->Sum_Error = 500;
    if(pid->Sum_Error < -500) pid->Sum_Error = -500;
    
    pid->Sum_Error += pid->Error;

    // 2. 核心公式
    pid->Output = (pid->Kp * pid->Error) + 
                  (pid->Ki * pid->Sum_Error) + 
                  (pid->Kd * (pid->Error - pid->Last_Error));

    // 3. 更新误差
    pid->Last_Error = pid->Error;

    // 4. 输出限幅
    if(pid->Output > pid->Output_Max) pid->Output = pid->Output_Max;
    if(pid->Output < -pid->Output_Max) pid->Output = -pid->Output_Max;

    return pid->Output;
}
