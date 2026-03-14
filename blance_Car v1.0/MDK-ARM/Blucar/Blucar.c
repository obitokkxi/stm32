#include "Blucar.h"
#include "HCSR04.h"

// ================== 全局变量定义 ==================

static char send_buf[64];
PID_TypeDef Track_PID; // 循迹专用 PID

//小车结构体初始化
Car_Handle_t myCar = {
    .Inertia_Step = DEFAULT_INERTIA_STEP,
    .Mode         = DEFAULT_MODE,
	 
};

//extern MPU6050 mpu;

// 循迹PID 参数
float Track_Kp = 0.0f;
float Track_Kd = 0.0f;

// 速度环PID参数
float Speed_Kp = 0.0f;//0.3
float Speed_Ki = 0.0000f;//0.0004f

//直立环PID参数
float Std_Kp = 0.0f;//490
float Std_Kd = 0.0f;//0.29

//转向环PID参数
float Turn_Kp = 0.0f;
float Turn_Kd = 0.0f;

float BLANCE_ANGLE=0;

float Speed_Integral = 0.0f;  // 速度环积分池

float Speed_Out;
float Upright_Out;

char show[20];
/**
 * @brief 初始化电机相关 (启动PWM)
 */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&TIM, MOTOR_CHANNEL1); // 左轮
    HAL_TIM_PWM_Start(&TIM, MOTOR_CHANNEL2); // 右轮
	
	 //获取小程序当前模式
	 sprintf(send_buf, "[Connect]"); 
    HAL_UART_Transmit(&huart, (uint8_t*)send_buf, strlen(send_buf), 100);
	 // 上电先报一次状态
    Car_Report_Status();
}

void Track_PID_Init(uint8_t P,uint8_t D)
{
    // === 调参区 ===
    // 直线摇摆？ -> 加大 Kd，减小 Kp
    // 转不过弯？ -> 加大 Kp
    Track_PID.Kp = P;  // P: 发现偏了，猛打方向
    Track_PID.Kd = D;  // D: 车头快正了，赶紧反向阻尼，防止甩过头
    Track_PID.Last_Error = 0;
}

/*小车函数(前版)

void Control_Car(uint8_t mode,uint8_t cmd,uint8_t speed)
{
	
	uint16_t Speed=speed;
		// 2. 防止 PWM 太小电机不转 (设置死区补偿)
        // 假设你的 ARR 是 100，如果 PWM < 30 可能电机就不动了
   if(Speed > 0 && Speed < 30) Speed = 30;
	

	if(mode==1){
		
		
		 switch(cmd)
		 {
			  case 'A': // 前进
					// 1. 设置方向引脚 (L298N 的 IN1/IN2/IN3/IN4)
					HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);   // 左正
					HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN,GPIO_PIN_SET);   // 右正
					HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
					
					Target_PWM_L=Speed*0.8;
					Target_PWM_R=Speed*0.8;
					// 2. 设置速度 (PWM)
					//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Current_PWM_L*0.8); // 80% 速度
					//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Current_PWM_R*0.8);
					break;

			  case 'B': // 后退
					// 方向反过来
					HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
					
					Target_PWM_L=Speed*0.8;
					Target_PWM_R=Speed*0.8;
					//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Current_PWM_L*0.8); // 后退慢一点
					//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,Current_PWM_R*0.8);
					break;
					  
			  case 'R': // 右转(差速)
				// 1. 设置方向引脚 (L298N 的 IN1/IN2/IN3/IN4)
					HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);   // 左正
					HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN,  GPIO_PIN_SET);   // 右正
					HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
					
					Target_PWM_L=Speed;
					Target_PWM_R=Speed*0.5;
					//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Current_PWM_L); // PWM 
					//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Current_PWM_R*0.5);
							
					break;
			  
			  case 'L': // 左转
				// 1. 设置方向引脚 (L298N 的 IN1/IN2/IN3/IN4)
					HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);   // 左正
					HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN,  GPIO_PIN_SET);   // 右正
					HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
					
					Target_PWM_L=Speed*0.5;
					Target_PWM_R=Speed;
					//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Current_PWM_L*0.5); // PWM 
					//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Current_PWM_R);
							
					break;
			  
			  case 'S': // 停止
					Target_PWM_L=0;
					Target_PWM_R=0;
							
					break;
		}
	
    }
	
	 else if(mode==2){
		switch (cmd)
        {
            case 'A': // 前进 
                HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET); 
					 HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET); 
				    HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
                Target_PWM_L=Speed;
					 Target_PWM_R=Speed;
					 Current_PWM_L = Speed; // 瞬间到位
					 Current_PWM_R = Speed; // 瞬间到位
                break;
            
            case 'B': // 后退 (同上)
                HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET); 
					 HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET); 
					 HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
                Target_PWM_L=Speed;
					 Target_PWM_R=Speed;
					 Current_PWM_L = Speed; // 瞬间到位
					 Current_PWM_R = Speed; // 瞬间到位
                break;

            case 'R': // 右转 (坦克掉头：左前，右后)
                HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);   
                HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET); // 右轮反转
                HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
                
                // 全速旋转
                Target_PWM_L=Speed;
					 Target_PWM_R=Speed;
					 Current_PWM_L = Speed; // 瞬间到位
					 Current_PWM_R = Speed; // 瞬间到位
                break;
            
            case 'L': // 左转 (坦克掉头：左后，右前)
                HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET); // 左轮反转
                HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
                HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);   
                HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
                
                // 全速旋转
                Target_PWM_L=Speed;
					 Target_PWM_R=Speed;
					 Current_PWM_L = Speed; // 瞬间到位
					 Current_PWM_R = Speed; // 瞬间到位
                break;

            case 'S': // 停止
                Target_PWM_L=0;
					 Target_PWM_R=0;
					 Current_PWM_L = 0; // 瞬间到位
					 Current_PWM_R = 0; // 瞬间到位
				
                break;
        }
	 }
	 
	 else if(mode==3){
		
	 }
}


// 专门处理惯性的函数
void Car_Speed_Loop(uint8_t mode)
{
	// 定义动态步长
    uint8_t step = INERTIA_STEP;
	// 模式1用小步长(更平滑)，模式2用大步长(更暴力)
    if (mode == 1) step = 1; 
    else step = 5;
	
    // --- 左轮处理 ---
    if (Current_PWM_L < Target_PWM_L) {
        Current_PWM_L += INERTIA_STEP;
        if (Current_PWM_L > Target_PWM_L) Current_PWM_L = Target_PWM_L;
    } 
    else if (Current_PWM_L > Target_PWM_L) {
        Current_PWM_L -= INERTIA_STEP;
        if (Current_PWM_L < Target_PWM_L) Current_PWM_L = Target_PWM_L;
    }

    // --- 右轮处理 ---
    if (Current_PWM_R < Target_PWM_R) {
        Current_PWM_R += INERTIA_STEP;
        if (Current_PWM_R > Target_PWM_R) Current_PWM_R = Target_PWM_R;
    } 
    else if (Current_PWM_R > Target_PWM_R) {
        Current_PWM_R -= INERTIA_STEP;
        if (Current_PWM_R < Target_PWM_R) Current_PWM_R = Target_PWM_R;
    }

    // --- 写入硬件 ---
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Current_PWM_L);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Current_PWM_R);
    
    // 优化：如果速度真的降到0了，为了安全/省电，可以关闭方向引脚
    if(Current_PWM_L == 0 && Current_PWM_R == 0) {
       HAL_GPIO_WritePin(GPIOB, AIN1_PIN|AIN2_PIN|BIN1_PIN|BIN2_PIN, GPIO_PIN_RESET);
    }
}

void Car_btry(void){
	// 1. 读取 ADC 值 (0 ~ 4095)
			  HAL_ADC_Start(&hadc1);
			  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
			  {
					uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
					
					// 2. 转换公式 (直接测量模式)
					// 3.3V 是参考电压 (Vref)，4095 是 12位最大值
					float voltage = (float)adc_val / 4095.0f * 3.3f;

					// 3. 构造数据包: [Power,3.25]
					// %.2f 保留两位小数，方便你看精度
					sprintf(send_buf, "[Power,%.2f]", voltage);
						//sprintf(send_buf, "[Power,1.5]");
				  HAL_Delay(30);
					// 4. 发送
					HAL_UART_Transmit(&huart1, (uint8_t*)send_buf, strlen(send_buf), 100);
				  sprintf(send_buf, "[Sonic,15]");
				  HAL_UART_Transmit(&huart1, (uint8_t*)send_buf, strlen(send_buf), 100);
			  }
			  HAL_ADC_Stop(&hadc1);
}
*/

/**
 * @brief 核心控制逻辑：设置目标速度和方向
 * @param mode: 1=竞速(惯性), 2=越野(瞬时), 3=自动
 * @param cmd: 'A','B','L','R','S'
 * @param speed: 0-100
 */


/**
 * @brief 上报电量和避障信息 (含防粘包延时)
 */
void Car_Report_Status(void)
{
    HAL_ADC_Start(&ADC);
    if (HAL_ADC_PollForConversion(&ADC, 10) == HAL_OK)
    {
        uint32_t adc_val = HAL_ADC_GetValue(&ADC);
        // 假设分压系数调整后
        float voltage = (float)adc_val / 4095.0f * 3.3f; // 根据实际电路调整系数

		 // 3. 计算电池实际电压 (核心修改！)
    // 如果你的原理图是: 电池+ -> 10k电阻 -> ADC引脚 -> 10k电阻 -> GND
    // 那么电压被除以了2，这里就要乘 2.0
    // 如果是电机驱动板，通常系数是 3.0 或 11.0，请用万用表测一下实际电压反推
		  float battery_voltage = voltage * 4.05f; // <--- 这里的 3.0 请根据硬件调整
        // 1. 发送电量
        // 临时调试用，打印真实电压和 ADC 原始数据
        //sprintf(send_buf, "[Power,%.2f] [Raw_ADC:%lu\r\n]", battery_voltage, adc_val);
        sprintf(send_buf, "[Power,%.2f]", battery_voltage);
        HAL_UART_Transmit(&huart2, (uint8_t*)send_buf, strlen(send_buf), 100);

		  
    }
    HAL_ADC_Stop(&ADC);
}

// ==========================================================
// 1. 速度环独立函数 (PI 控制)
// 作用：根据速度误差计算出目标角度的偏移量
// ==========================================================
float Speed_Loop(float Target_Speed, float Encoder_L,float Encoder_R)
{
   // float Speed_Error = Target_Speed - Current_Speed; 
    static float Speed_Last_Error,Speed_Error_Sum,Speed_Filtered=0;
    float Speed_Error,Speed_Out,Speed_Current;
    
    // 1. 当前速度 = 左+右 (带方向的脉冲数/周期)
    Speed_Current = Encoder_L + Encoder_R;
    // 2. 一阶低通滤波 (α=0.3, 滤掉编码器毛刺)
    //    公式: filtered = α × new + (1-α) × old
    Speed_Filtered = 0.3f * Speed_Current + 0.7f * Speed_Filtered;
    // 3. 误差 = 滤波后速度 - 目标速度
    Speed_Error = Speed_Filtered - Target_Speed;
    //Speed_Error=(Econder_L+Econder_R)-Target_Speed;
    //Speed_Out=0.3*Speed_Error+0.7*Speed_Last_Error;
    //Speed_Last_Error=Speed_Out;
    // ★ Bug3 修复：停车时清除积分
    // 当目标速度为0且实际速度很小时，快速衰减积分
    
    // 积分累加与严格限幅
    Speed_Integral += Speed_Error; 
    if(Speed_Integral > 5000.0f)  Speed_Integral = 5000.0f;
    if(Speed_Integral < -5000.0f) Speed_Integral = -5000.0f;

    // 输出角度补偿量
    return -((Speed_Kp * Speed_Error) + (Speed_Ki * Speed_Integral));
}

// ==========================================================
// 2. 直立环独立函数 (PD 控制)
// 作用：根据当前角度和目标角度，计算出维持平衡所需的基础动力
// ==========================================================
float Stand_Loop(float Current_Angle, float Target_Angle, float Gyro_Rate)
{
    // P项算角度差，D项直接使用真实角速度
    return -(Std_Kp * (Current_Angle - Target_Angle) + Std_Kd * Gyro_Rate);
}

// ==========================================================
// 3. 转向环独立函数 (P 控制)
// 作用：抵抗自转，或执行左右转向指令
// ==========================================================
float Turn_Loop(float Target_Turn, float Gyro_Z)
{
    //float Turn_Error = Target_Turn - Gyro_Z; 
    return (Turn_Kp * Target_Turn+ Turn_Kd * Gyro_Z); 
}

void Car_selfbalance(uint16_t car_speed,char car_dir){ 
    
  
    char t[10];
    static float Speed_Filtered_L;
    static float Speed_Filtered_R;
    static float Gyro_X_Filtered = 0;   // ★ 新增
    static float Gyro_Z_Filtered = 0;   // ★ 新增
    float speed=0 ;
    float target_turn = 0;
    // 定义一个全局变量，代表最终给到 PID 的“平滑目标速度”
    static float Smooth_Target_Speed = 0;
    // 0. 更新感官数据
    // if (MPU6050_Update(&mpu) != HAL_OK) return; 

    // float Angle_X = mpu.angle_roll;  
    // float Gyro_X  = mpu.gyro_dps[0]; 
    // float Gyro_Z  = mpu.gyro_dps[2]; 
    // float Angle_X = mpu.KalmanAngleX;
    // float Gyro_X  = mpu.Gx;
    // float Gyro_Z  = mpu.Gz;
    float Angle_X = mpu.roll ;
    float Gyro_X  = mpu.GyroX/ 16.4f;
    float Gyro_Z  = mpu.GyroZ/ 16.4f;
   // float Current_Speed = (Speed_L + Speed_R) / 2.0f; 
    // ★★★ 对角速度做低通滤波，杀掉高频噪声 ★★★
    // α=0.7: 保留70%新值 + 30%旧值 (比0.3更跟手，同时滤掉毛刺)
    Gyro_X_Filtered = 0.7f * Gyro_X + 0.3f * Gyro_X_Filtered;
    Gyro_Z_Filtered = 0.7f * Gyro_Z + 0.3f * Gyro_Z_Filtered;
    

    //倒地保护锁
    if(Angle_X > 40.0f || Angle_X < -40.0f) 
    {
            Car_Motor_Output('S', 2, 0, 0);
            Speed_Integral = 0; // 倒地清空积分
            Speed_Filtered_L = 0;      // ★ 滤波也清零!
            Speed_Filtered_R = 0;
            Gyro_X_Filtered = 0;  // ★ 清零
            Gyro_Z_Filtered = 0;
            return;    // ← 直接退出, 不往下算了
    }

   if (car_dir == 'A')      speed = car_speed; 
    else if (car_dir == 'B') speed = -car_speed;
 
    else if (car_dir == 'L') target_turn = (int)car_speed;  // 左转 -30°/s
    else if (car_dir == 'R') target_turn =  -(int)car_speed;  // 右转 +30°/s
    
    else if (car_dir == 'S') { // 停止
        speed = 0; // 给负速度，车子就会后仰倒车
    }else                     target_turn =  0.0f;   // 直走


// ==========================================================
    float step = 15.0f; // 步进值，越小起步越柔和，越大越暴躁。建议从 10~20 开始调
    
    if (Smooth_Target_Speed < speed) {
        Smooth_Target_Speed += step;
        if (Smooth_Target_Speed > speed) Smooth_Target_Speed = speed;
    } 
    else if (Smooth_Target_Speed > speed) {
        Smooth_Target_Speed -= step;
        if (Smooth_Target_Speed < speed) Smooth_Target_Speed = speed;
    }

    // --------------------------------------------------------
    // ? 串级控制流水线 (极其清晰的模块化调用)
    // --------------------------------------------------------
    
    // 1. 计算速度环 (得出角度补偿)
    // 注意：调直立环时，如果把 Speed_Kp 和 Speed_Ki 设为 0，这里输出就是 0，完全不影响直立
    Speed_Out = Speed_Loop(Smooth_Target_Speed, Speed_L, Speed_R);

    // 速度环输出限制在 ±10° 以内
    if (Speed_Out >  10.0f) Speed_Out =  10.0f;
    if (Speed_Out < -10.0f) Speed_Out = -10.0f;

    // 2. 计算直立环 (得出前后基础 PWM)
    // 动态目标角度 = 物理绝对零点 - 速度补偿
    float Target_Angle = BLANCE_ANGLE - Speed_Out; 
    // if(car_dir == 'A'){
    //     car_speed=BLANCE_ANGLE-5;
    // }
    // if(car_dir == 'B'){
    //     Target_Angle=BLANCE_ANGLE+5;
    // }
    Upright_Out = Stand_Loop(Angle_X, Target_Angle, Gyro_X_Filtered);

    // 暴力限幅 (绝不让 PWM 越界)
    if (Upright_Out > PWM_LIMIT) Upright_Out = PWM_LIMIT;   // ← int16_t 比较用整数
    if (Upright_Out < -PWM_LIMIT) Upright_Out = -PWM_LIMIT;
    // 3. 计算转向环 (得出左右差速 PWM)
    // (注意：这里 car_dir 如果是 'L' 或 'R' 字符，直接做浮点运算可能会有逻辑问题，
    // 建议小程序下发的转向指令是具体的数值，比如 0代表直走，20代表右转，-20代表左转)
   
    float Turn_Out = Turn_Loop(target_turn, Gyro_Z_Filtered);

    // 4. 三环合一，力量融合
    float Final_PWM_L = Upright_Out + Turn_Out;
    float Final_PWM_R = (Upright_Out - Turn_Out);

    // --------------------------------------------------------
    // 终极安全与底层输出
    // --------------------------------------------------------
    

    // OLED 打印调试信息
    
    // SSD1306_GotoXY(0, 40);
    // sprintf(t,"[P:%.2f]", Final_PWM_L);
    // SSD1306_Puts(t, &Font_11x18, 1);
    // SSD1306_UpdateScreen();
    
    // ? 发送给电机底层执行 (Mode 2 瞬间爆发模式)
    Car_Motor_Output('A', 2, Final_PWM_L, Final_PWM_R);
}

void Car_AutoRun(Car_Handle_t *Car_Set){
	static uint8_t scan_state = 0;
    static uint8_t wait_timer = 0; 
        
    static float dist_L = 0;
    static float dist_R = 0;
    static float dist_Front = 0;
	
    if(Car_Set->Mode == 3)
    {
        // 状态定义：
        // 0-4: 扫描流程 (不变)
        // 5: 决策 (分配任务)
        // 6: 死胡同-倒车中
        // 7: 死胡同-旋转中
        // 8: 正常-转弯中
        // 9: 舵机归位
        // 10: 等待舵机归位
        
        

        switch(scan_state)
        {
            case 0: // 准备：看前方
                Set_Servo_Angle(&htim1, TIM_CHANNEL_1, 90); 
                dist_Front = SR04_Read();
				if(dist_Front <= 0) dist_Front = 999.0f; // 0视为空旷
                
                if(dist_Front > 0 && dist_Front < 25.0f) {
                    
						  Car_Wheelspd('S', Car_Set, 0, 0);
                    scan_state = 1; 
                } else {
                    
						  Car_Wheelspd('A', Car_Set, 45, 45);
                }
                break;

            case 1: // 动作：向左看
                Set_Servo_Angle(&htim1, TIM_CHANNEL_1, 180); 
                wait_timer = 0; // 借用计数器来等待舵机
                scan_state = 2; 
                break;

            case 2: // 动作：等待并测左边
                // 等待 300ms (3 * 100ms)
                if(++wait_timer >= 5) { 
                    dist_L = SR04_Read();
                    scan_state = 3; 
                }
                break;

            case 3: // 动作：向右看
                Set_Servo_Angle(&htim1, TIM_CHANNEL_1, 0); 
                wait_timer = 0;
                scan_state = 4;
                break;

            case 4: // 动作：等待并测右边
                if(++wait_timer >= 5) { 
                    dist_R = SR04_Read();
                    scan_state = 5; // 下一步：决策
                }
                break;

            // ========================================================
            // ?? 【核心修改】 将原来的阻塞逻辑拆解为非阻塞状态
            // ========================================================
            
            case 5: // 动作：决策分配
                if (dist_L < 25.0f && dist_R < 25.0f) 
                {
                    // === 触发死胡同模式 ===
                    // 1. 发送倒车指令
                   
                    Car_Wheelspd('B', Car_Set, 45, 45);
                    wait_timer = 0;
                    scan_state = 6; // 跳转到"正在倒车"状态
                }
                else 
                {
                    // === 触发正常避障 ===
                    // 1. 发送转弯指令
                    if(dist_L > dist_R) Car_Wheelspd('L', Car_Set, -50, 50);
                    else                Car_Wheelspd('R', Car_Set, 50, -50);
                   
                    
                    wait_timer = 0;
                    scan_state = 8; // 跳转到"正在转弯"状态
                }
                break;

            case 6: // 状态：死胡同-正在倒车 (目标 500ms)
                // 每次进来代表过了100ms
                if(++wait_timer >= 5) {
						 // 倒车结束，开始原地大旋转
						 if(dist_L > dist_R)
						 {
							Car_Wheelspd('L', Car_Set, -60, 60);
						 }							 
                   else Car_Wheelspd('R', Car_Set, 60, -60);
                    
                    wait_timer = 0;
                    scan_state = 7; // 跳转到"正在大旋转"
                }
                break;

            case 7: // 状态：死胡同-正在大旋转 (目标 900ms)
                if(++wait_timer >= 5) {
                    scan_state = 9; // 旋转完成，去收尾
                }
                break;

            case 8: // 状态：正常避障-正在转弯 (目标 400ms)
                if(++wait_timer >= 3) {
                    scan_state = 9; // 转弯完成，去收尾
                }
                break;

            case 9: // 动作：收尾 & 舵机回正
                // 1. 舵机回正
                Set_Servo_Angle(&htim1, TIM_CHANNEL_1, 90);
                
                // 2. 准备等待舵机转回去
                wait_timer = 0;
                scan_state = 10;
                break;
                
            case 10: // 状态：等待舵机归位 (目标 200ms)
                if(++wait_timer >= 3) {
                    // 全部结束，回到初始状态，开始新一轮检测
                    scan_state = 0; 
                }
                break;
        }
    }
   
		 
	 else
	 {
			 scan_state = 0;
			 wait_timer = 0;
			 // 只有非模式3才更新全局距离用于显示
			 g_Distance = SR04_Read();
			 
			 // 确保舵机是朝前的
			 // 为了防止一直发 PWM 抖动，可以加个标志位只发一次
			 static uint8_t servo_reset = 0;
			 if(servo_reset == 0) {
					Set_Servo_Angle(&htim1, TIM_CHANNEL_1, 90);
					servo_reset = 1;
			 }
	 
	 }
}

void Car_Track_Line(Car_Handle_t *Car_Set, MPU6050 *gyro)
{
   uint8_t L1 = HAL_GPIO_ReadPin(L1_Port, L1_Pin); 
    uint8_t R1 = HAL_GPIO_ReadPin(R1_Port, R1_Pin); 
    uint8_t L2 = HAL_GPIO_ReadPin(L2_Port, L2_Pin); 
    uint8_t R2 = HAL_GPIO_ReadPin(R2_Port, R2_Pin); 

    static float Last_Error = 0; 
    float Error = 0;
    
    // 1. 计算误差 (这里包含了微调和急转的所有情况)
    if      (L1 && R1) Error = ERR_CENTER; // 直行
    else if (L1 && !R1) Error = -ERR_LOW;   // 偏右 -> 左微调
    else if (!L1 && R1) Error = ERR_LOW;    // 偏左 -> 右微调
    
    else if (L2 && L1) Error = -ERR_MID;    // 偏右较多
    else if (R2 && R1) Error = ERR_MID;     // 偏左较多
    
    else if (L2)       Error = -ERR_HIGH;   // 极度偏右 -> 左急转
    else if (R2)       Error = ERR_HIGH;    // 极度偏左 -> 右急转
    
    else // 全白 (盲区)
    {
        if (Last_Error > 0) Error = 5.0f;       // 继续右转
        else if (Last_Error < 0) Error = -5.0f; // 继续左转
        else Error = 0;
    }

    // 2. PID 计算
    float P_Value = Track_Kp * Error;
    float D_Value = Track_Kd * (Error - Last_Error);
    Last_Error = Error;
    
    float Turn_Output = P_Value + D_Value;

    // 3. 计算左右电机速度
    float Motor_L = BASE_SPEED + Turn_Output;
    float Motor_R = BASE_SPEED - Turn_Output;

    // 4. 限幅
    if (Motor_L > 100) Motor_L = 100;
    if (Motor_R > 100) Motor_R = 100;
    if (Motor_L < -60) Motor_L = -60;
    if (Motor_R < -60) Motor_R = -60;

    // ============================================================
    // 5. 【核心修改】动态生成指令字符 ('A'/'L'/'R')
    // ============================================================
    uint8_t final_cmd;

    // 如果 PID 输出非常小 (比如小于 5)，说明基本在走直线
    // 发送 'A' 激活你的 Car_Wheelspd 里的陀螺仪锁头功能！
    if (fabs(Turn_Output) < 5.0f) 
    {
        final_cmd = 'A';
    }
    // 如果 Turn_Output 是正数 (左轮快，右轮慢)，说明是右转
    else if (Turn_Output > 0)
    {
        final_cmd = 'R';
    }
    // 如果 Turn_Output 是负数 (左轮慢，右轮快)，说明是左转
    else
    {
        final_cmd = 'L';
    }

    // 6. 发送给电机
    //Car_Wheelspd(final_cmd, Car_Set, (int16_t)Motor_L, (int16_t)Motor_R);
}

/**
 * @brief 速度环计算
 * @param Target 目标值 (如果是开环，则是PWM；如果是闭环，则是速度cm/s)
 * @param Actual 实际速度
 * @param pid PID结构体
 * @return 计算后的基础 PWM
 */
float Car_Speed_Ctrl(float Target, float Actual, PID_TypeDef *pid)
{
#if USE_SPEED_PID == 1
    // === 闭环模式 (PID) ===
    // 只有在测速准确时才用这个
    return PID_Compute(pid, Actual, Target);
#else
    // === 开环模式 (纯PWM) ===
    // 目标是多少，就输出多少，完全平滑，不抖动
    return Target;
#endif
}

/**
 * @brief 角度环计算
 * @return 修正量 Gyro_Fix (正数代表需要向右修)
 */

 float Car_Angle_Ctrl(uint8_t cmd, float Base_Speed)
{
    static float Target_Yaw_Lock = 0; 
    static uint8_t Is_Yaw_Locked = 0; 
    static uint8_t Last_Cmd = 'S';
    float Fix = 0;

    // 只有在【纯直走 A/B】且【有速度】时启用
    if ((cmd == 'A' || cmd == 'B') && Base_Speed > 0)
    {
        // 状态切换时重置锁
        if (cmd != Last_Cmd || Is_Yaw_Locked == 0) {
            Target_Yaw_Lock = mpu.angle_yaw; 
            Is_Yaw_Locked = 1;
            // 如果有PID，这里应该清除积分，但现在不需要了
        }

        float Yaw_Err = mpu.angle_yaw - Target_Yaw_Lock;
        float Yaw_Rate = mpu.angle_yaw; 

        // 计算修正量
        Fix = (Yaw_Err * GYRO_KP) + (Yaw_Rate * GYRO_KD);

        // 倒车不取反 (根据你之前的测试结果)
        // if (cmd == 'B') Fix = -Fix;

        // 限幅 (修正量不超过基础速度的 40%)
        float Max_Fix = Base_Speed * 0.4f;
        if (Fix > Max_Fix) Fix = Max_Fix;
        if (Fix < -Max_Fix) Fix = -Max_Fix;
    }
    else
    {
        Target_Yaw_Lock = mpu.angle_yaw; 
        Is_Yaw_Locked = 0;
        Fix = 0;
    }
    Last_Cmd = cmd;
    
    return Fix;
}


/**
 * @brief 硬件输出层
 * 逻辑：接收带符号的速度 (-100 ~ 100)
 * 负数 = 反转 (GPIO切换，PWM取绝对值)
 * 正数 = 正转
 */
void Car_Motor_Output(uint8_t cmd, uint8_t Mode, float PWM_L, float PWM_R)
{
   
    // ============================================================
    // 3. 确定方向与幅值 (现在根据 Current_PWM 来定方向，就对了！)
    // ============================================================
    int16_t Final_L = (int16_t)PWM_L;
    int16_t Final_R = (int16_t)PWM_R;

    // --- 左轮方向 ---
    if (Final_L >= 0) 
    {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
    } 
    else 
    {
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
        Final_L = -Final_L; // 取绝对值用于 PWM
    }

    // --- 右轮方向 ---
    if (Final_R < 0) 
    {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
        Final_R = -Final_R; // 取绝对值
    } 
    else 
    {
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
       
    }

 
    // ============================================================
    // 4. 限幅与死区补偿
    // ============================================================
    
    // if (Final_L > 0 && Final_L < DEAD_ZONE) 
    // Final_L = DEAD_ZONE;
    // if (Final_R > 0 && Final_R < DEAD_ZONE) 
    // Final_R = DEAD_ZONE;

    

   
    __HAL_TIM_SET_COMPARE(&TIM, MOTOR_CHANNEL1, (uint16_t)Final_L);
    __HAL_TIM_SET_COMPARE(&TIM, MOTOR_CHANNEL2, (uint16_t)Final_R);
}

/**
 * @brief 闭环控制核心函数
 * @param cmd 指令 'A'前进, 'B'后退, 'L'/'R'转弯, 'S'停止
 * @param L   左轮目标速度 (不再是PWM，是期望的速度值，如 40cm/s)
 * @param R   右轮目标速度
 */

 
void Car_Wheelspd(uint8_t cmd, Car_Handle_t *Car_Set, int16_t L, int16_t R)
{
    Car_Set->cmd = cmd;
    
    // 静态变量 (陀螺仪锁头用)
    static uint8_t Last_Cmd = 'S';
    static float Target_Yaw_Lock = 0; 
    static uint8_t Is_Yaw_Locked = 0; 

    // 初始化目标
    float Target_L = 0;
    float Target_R = 0;

	// ============================================================
    // 2. 陀螺仪直走修正 (仅在 'A' 且有速度时生效)
    // ============================================================
    float Gyro_Fix = 0;
	
	// 1. 强制停车逻辑 (优先级最高！)
    // 1. 强制停车逻辑 (增强版)
    // 只要命令是 'S' 或者 目标速度极低，就强制掐断！
    if (cmd == 'S' || (L == 0 && R == 0)) 
    {
        Is_Yaw_Locked = 0;
        // 关键：强制归零，防止陀螺仪的鬼影修正值 leaking 出去
        Car_Motor_Output('S', Car_Set->Mode, 0, 0); 
        
        Car_Set->Target_PWM_L = 0;
        Car_Set->Target_PWM_R = 0;
        return; // 直接退出，不给后面代码执行的机会
    }
    // ============================================================
    // 1. 逻辑分流 (核心修改!)
    // ============================================================
    
    // --- 情况 A: 循迹模式 (Mode 4) ---
    // 策略：完全透传！循迹函数算好了是多少就是多少 (比如 0, 95)
    // 绝对不要在这里乘 0.2，否则循迹就动不了了
    if (Car_Set->Mode == 4)
    {
        Target_L = (float)L;
        Target_R = (float)R;
    }
    
    // --- 情况 B: 遥控/避障模式 (Mode 1, 2, 3) ---
    // 策略：保留你原来的逻辑，根据 Base_Speed 计算差速
    else
    {
        float Base_Speed = (float)L; // 以 L 为基准速度

        if (cmd == 'A') // 前进
        { 
            Target_L = Base_Speed; 
            Target_R = Base_Speed; 
        }
        else if (cmd == 'B') // 后退
        { 
            Target_L = -Base_Speed; 
            Target_R = -Base_Speed; 
        }
        else if (cmd == 'S') // 停止
        { 
            Target_L = 0; 
            Target_R = 0; 
        }
        else if (cmd == 'L') // 左转
        { 
            // 恢复你的逻辑：
            // 模式2(坦克): 左轮反转
            // 模式1(普通): 左轮慢转 (0.2倍)
            if(Car_Set->Mode == 2) Target_L = -Base_Speed; 
            else                   Target_L = Base_Speed * 0.2f;
            
            Target_R = Base_Speed; 
        }
        else if (cmd == 'R') // 右转
        { 
            Target_L = Base_Speed; 
            
            // 恢复你的逻辑：
            if(Car_Set->Mode == 2) Target_R = -Base_Speed; 
            else                   Target_R = Base_Speed * 0.2f;
        }
    }

    

    // 只有前进时开启锁头 (不管是循迹还是遥控)
    if (cmd == 'A' && Target_L > 10)
    {
        if (cmd != Last_Cmd || Is_Yaw_Locked == 0) 
        {
            Target_Yaw_Lock = mpu.angle_yaw; 
            Is_Yaw_Locked = 1; 
            PID_Reset(&Motor_PID_L);
            PID_Reset(&Motor_PID_R);
        }

        float Yaw_Err = mpu.angle_yaw - Target_Yaw_Lock;
        float Yaw_Rate = mpu.angle_yaw; 
        Gyro_Fix = (Yaw_Err * GYRO_KP) + (Yaw_Rate * GYRO_KD);

        float Max_Fix = Target_L * 0.3f;
        if (Gyro_Fix > Max_Fix) Gyro_Fix = Max_Fix;
        if (Gyro_Fix < -Max_Fix) Gyro_Fix = -Max_Fix;
    }
    else
    {
        Is_Yaw_Locked = 0;
        Gyro_Fix = 0;
    }
    Last_Cmd = cmd;

    // ============================================================
    // 3. 最终输出计算
    // ============================================================
    float Out_L = Target_L;
    float Out_R = Target_R;

    if (cmd == 'A') {
        Out_L += Gyro_Fix;
        Out_R -= Gyro_Fix;
    }

    // 4. 发送给硬件
    Car_Motor_Output(cmd, Car_Set->Mode, Out_L, Out_R);
    
}

void Encoder_Init_Start(void)
{
    // 开启定时器2和定时器3的全部硬件编码器通道
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void Car_Getspd(float *speed_L, float *speed_R)
{
	// 1. 读取 10ms 内的“瞬时脉冲增量”
    // 强转为 int16_t 解决正反转极性问题
    int16_t temp_pulse_L = -(int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t temp_pulse_R =  (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    // 2. 读取完当前速度后，必须将计数器立刻清零！
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // 3. 计算当前的真实物理速度（注意：这里必须是 = 赋值，绝对不能是 += 累加！）
    // *speed_L = ((float)temp_pulse_L / (WHEEL_PULSES_PER_ROUND * DELTA_T)) * (PI * WHEEL_DIAMETER);
    // *speed_R = ((float)temp_pulse_R / (WHEEL_PULSES_PER_ROUND * DELTA_T)) * (PI * WHEEL_DIAMETER);
// ? 核心修改：直接把脉冲数赋值给全局变量，不要做任何公式计算！

    *speed_L = (float)temp_pulse_L;
    *speed_R = (float)temp_pulse_R;

    // 4. 将瞬时脉冲累加到总脉冲池中（这才是真正的里程表，用于以后的位置环） [cite: 2026-02-17]
    pulse_count_L += temp_pulse_L; 
    pulse_count_R += temp_pulse_R;

}

/**
 * @brief 强制停车 (看门狗触发用)
 */
void Car_Force_Stop(Car_Handle_t *Car_Set)
{
    __HAL_TIM_SET_COMPARE(&TIM, MOTOR_CHANNEL1,0);
    __HAL_TIM_SET_COMPARE(&TIM, MOTOR_CHANNEL2,0);
	Car_Wheelspd('S', Car_Set, 0, 0);  
    HAL_GPIO_WritePin(IN_GPIO_Port, AIN1_PIN|AIN2_PIN|BIN1_PIN|BIN2_PIN, GPIO_PIN_RESET);
}




