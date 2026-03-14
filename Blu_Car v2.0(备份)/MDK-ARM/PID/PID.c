#include "pid.h"

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_out) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->Output_Max = max_out;
	 pid->Actual=0;  
    pid->Sum_Error = 0;
    pid->Last_Error = 0;
	 pid->Integral_Max=500;
}

float PID_Compute(PID_TypeDef *pid, float actual_val, float target_val) {
    pid->Actual = actual_val;
    pid->Target = target_val;
    pid->Error = pid->Target - pid->Actual;

    // 1. 核心公式计算
    // P项
    float P_Term = pid->Kp * pid->Error;
    
    // D项
    float D_Term = pid->Kd * (pid->Error - pid->Last_Error);

    // I项 (先计算，不急着加进去，为了做抗饱和)
    // 只有在输出没有达到极限，或者 误差方向与输出方向相反时，才进行积分累加
    // 简单的做法：先累加，后限幅
    pid->Sum_Error += pid->Error;

    // === 修正：积分限幅必须在累加之后做！===
    if(pid->Sum_Error > pid->Integral_Max) pid->Sum_Error = pid->Integral_Max; // 建议在结构体里加个 Integral_Max
    else if(pid->Sum_Error < -pid->Integral_Max) pid->Sum_Error = -pid->Integral_Max;
    // 如果没定义 Integral_Max，就写死数字比如 1000 (取决于你的 Ki 大小)

    float I_Term = pid->Ki * pid->Sum_Error;

    // 2. 计算总输出
    pid->Output = P_Term + I_Term + D_Term;

    // 3. 更新误差
    pid->Last_Error = pid->Error;

    // 4. 停车特殊处理 (你的逻辑很好，保留)
    if (pid->Target == 0 && (pid->Actual < 5 && pid->Actual > -5)) {
        pid->Sum_Error = 0;
        pid->Output = 0;
        pid->Last_Error = 0; // 停车时把 Last_Error 也清了，防止下次启动 D 项跳变
        return 0; 
    }

    // 5. 输出限幅
    if(pid->Output > pid->Output_Max) pid->Output = pid->Output_Max;
    if(pid->Output < -pid->Output_Max) pid->Output = -pid->Output_Max;

    return pid->Output;
}

// 复位 PID（停车时调用）
void PID_Reset(PID_TypeDef *pid) {
    pid->Sum_Error = 0;
    pid->Last_Error = 0;
    pid->Error = 0;
}