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
void Car_Set_Motion(Car_Handle_t *Car_Set)
{
   
			static uint8_t lastmode;
			uint8_t MODE = Car_Set->Mode;
			lastmode=MODE;
			if(MODE!=lastmode)
			{
				Car_Set->Target_Speed=0;
				Car_Set->Target_PWM_L=0;
				Car_Set->Target_PWM_R=0;
			}
		
		 // 1. 从结构体读取数据
			uint16_t Target_Spd = Car_Set->Target_Speed; 
			char cmd = Car_Set->cmd;
				 
			/*
			// ?? 【核心修改】 绝对防御逻辑 (解决微动问题)
		 // ==========================================================
		 // 如果是模式 1 或 2，且距离危险
			 if(MODE != 3 && g_Distance < 20.0f)
			 {
				  // 如果想“前进(A)”，直接篡改为“停车(S)”
				  // 注意：这里允许 B(后退) 和 L/``R(原地转向) 通过，方便倒车
				  if(cmd == 'A') 
				  {
						cmd = 'S'; 
						Target_Spd = 0;
						// 顺便把结构体里的值也改了，防止下次循环误读
						Car_Set->cmd = 'S';
						Car_Set->Target_Speed = 0;
				  }
			 }	
			 */
		
				 //避免电机死区,限定速度范围
				if(Target_Spd>0&&Target_Spd<MIN_MOTOR_PWM){
					Target_Spd=MIN_MOTOR_PWM;
				}
				//临时速度变量
				 uint16_t next_L = 0;
				 uint16_t next_R = 0;

				 // 2. 计算目标 PWM
				 switch (cmd)
				 {
					  case 'A': next_L = Target_Spd;       next_R = Target_Spd;       break;
					  case 'B': next_L = Target_Spd;       next_R = Target_Spd;       break;
					  case 'R': if(MODE==1)
									{
										next_L = Target_Spd;       
										next_R = Target_Spd * 0.4;
									} 
									
									if(MODE == 4)
									{
										Car_Set->Target_PWM_L = Target_Spd; 
										Car_Set->Target_PWM_R = Target_Spd*0.2;
										Car_Set->Current_PWM_L = Car_Set->Target_PWM_L;
									  Car_Set->Current_PWM_R = Car_Set->Target_PWM_R;
									} 
									
									break;
					  case 'L': if(MODE==1)
									{
										next_L = Target_Spd * 0.4; 
										next_R = Target_Spd;
									}
									
									if(MODE == 4)
									{
										
											Car_Set->Target_PWM_L =Target_Spd*0.2; 
											Car_Set->Target_PWM_R = Target_Spd;
										
										
										// 速度直接一步到位
									  Car_Set->Current_PWM_L = Car_Set->Target_PWM_L;
									  Car_Set->Current_PWM_R = Car_Set->Target_PWM_R;
									}	
									
									break;
					  case 'S': next_L = 0;                next_R = 0;                break;
				 }
				 //避免电机死区,限定速度范围
				 // 处理左轮
				 if(next_L > 0) {
					  if(cmd == 'A' || cmd == 'B' ) {
							// 直线或坦克模式，必须有力
							if(next_L < MIN_MOTOR_PWM) next_L = MIN_MOTOR_PWM;
					  }
					  
				 }

				 // 处理右轮 (同理)
				 if(next_R > 0) {
					  if(cmd == 'A' || cmd == 'B' ) {
							if(next_R < MIN_MOTOR_PWM) next_R = MIN_MOTOR_PWM;
					  }
				 }
				 
				 if (MODE == 1) {
					 
					 //赋值给目标值,速度平缓加或减
					 Car_Set->Target_PWM_L = next_L;
					 Car_Set->Target_PWM_R = next_R;
				 }
				 // 4. 处理暴力模式 (Mode 2)
				 if (MODE == 2|| MODE == 3 ) 
				 {		 
					  Car_Set->Target_PWM_L=Target_Spd;
					  Car_Set->Target_PWM_R=Target_Spd;
					  // 速度直接一步到位
					  Car_Set->Current_PWM_L = Car_Set->Target_PWM_L;
					  Car_Set->Current_PWM_R = Car_Set->Target_PWM_R;
				 }
				 /*
				 if(MODE == 4){
						
							Car_Set->Target_PWM_L=Target_Spd;
						  Car_Set->Target_PWM_R=Target_Spd;
						  // 速度直接一步到位
						  Car_Set->Current_PWM_L = Car_Set->Target_PWM_L;
						  Car_Set->Current_PWM_R = Car_Set->Target_PWM_R;
						
				 }
	*/
				 // 5. 设置 GPIO 方向
				 if(cmd != 'S') 
				 {
					 

							switch(cmd)
							{
								case 'A': // 前进
									 HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET); 
									 HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
									 HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET); 
									 HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
									 break;
								
								case 'B': // 后退
									 HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET); 
									 HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
									 HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET); 
									 HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
									 break;
								
								case 'R': // 右转
									 if(MODE==1||MODE==4){ 
										 HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET); 
										 HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
										 HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET); 
										 HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
									 }
									 else
									 {
										 HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET); 
										 HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
										 HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET); 
										 HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
									 }
									
									 break;
									 
								case 'L': // 左转
									 if(MODE==1||MODE==4){ 
										 HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET); 
										 HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
										 HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET); 
										 HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
									 }
									 else
									 {
										 HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET); 
										 HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
										 HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET); 
										 HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
									 }
									 break;

						  }
					  
				 }

}



/**
 * @brief 惯性计算循环 (建议 10ms 调用一次)
 */
void Car_Speed_Handle(Car_Handle_t *Car_Set)
{
	
	 char cmd = Car_Set->cmd;
	 uint8_t MODE=Car_Set->Mode;
	 uint8_t L_spd=Car_Set->Target_PWM_L;
	 uint8_t R_spd=Car_Set->Target_PWM_R;
	 uint8_t next_L,next_R;
	 //uint16_t Current_PWM_L = Car_Set->Current_PWM_L;
	// uint16_t Current_PWM_R = Car_Set->Current_PWM_R;
	 float Inertia_Step=Car_Set->Inertia_Step;
	
		switch (cmd)
		{
					  case 'A': next_L = L_spd;       next_R = R_spd;       break;
					  case 'B': next_L = L_spd;       next_R = R_spd;       break;
					  case 'R': if(MODE==1)
									{
										next_L = L_spd;       
										next_R = R_spd * 0.4;
									} 
									
									if(MODE == 4)
									{
										next_L = L_spd;       
										next_R = R_spd * 0.2;
										
									} 
									
									break;
					  case 'L': if(MODE==1)
									{
										next_L = L_spd * 0.4; 
										next_R = R_spd;
									}
									
									if(MODE == 4)
									{
										
											next_L =L_spd*0.2; 
											next_R= R_spd;
										
										
										
									}	
									
									break;
					  case 'S': next_L = 0;                next_R = 0;                break;
		}
	
		if(MODE == 1){
		 /*
				// === 减速分频锁 (让惯性更明显) ===
			 static uint8_t divider = 0;
			 if(++divider < 2) return; // 每 30ms 执行一次计算
			 divider = 0;
			*/
		 
			float Step_To_Use = Inertia_Step;
       //if(MODE == 4) Step_To_Use = Inertia_Step * 2; // 循迹时加速减速快一倍
			 // --- 左轮计算 ---
			 if (Car_Set->Current_PWM_L < next_L) {
				  Car_Set->Current_PWM_L += Step_To_Use;
				  if (Car_Set->Current_PWM_L > next_L) 
					  Car_Set->Current_PWM_L = next_L;
			 } else if (Car_Set->Current_PWM_L >next_L) {
				  Car_Set->Current_PWM_L -= Step_To_Use;
				  if (Car_Set->Current_PWM_L < next_L) 
					  Car_Set->Current_PWM_L = next_L;
			 }

			 // --- 右轮计算 ---
			 if (Car_Set->Current_PWM_R < next_R) {
				  Car_Set->Current_PWM_R += Step_To_Use;
				  if (Car_Set->Current_PWM_R > next_R) 
					  Car_Set->Current_PWM_R = next_R;
			 } else if (Car_Set->Current_PWM_R > next_R) {
				  Car_Set->Current_PWM_R -= Step_To_Use;
				  if (Car_Set->Current_PWM_R < next_R) 
					  Car_Set->Current_PWM_R = next_R;
			 }
			 
			 // --- 写入硬件 ---
			 __HAL_TIM_SET_COMPARE(&TIM, MOTOR_CHANNEL1, (uint16_t)Car_Set->Current_PWM_L);
			 __HAL_TIM_SET_COMPARE(&TIM, MOTOR_CHANNEL2, (uint16_t)Car_Set->Current_PWM_R);

				 
			 
	 }
	
    // 如果是模式2，直接写入硬件(虽然上面设了，这里双保险)，跳过计算
    if(MODE == 2||MODE == 3||MODE == 4) {
		 
		 Car_Set->Current_PWM_L = L_spd;
		 Car_Set->Current_PWM_R = R_spd;
        __HAL_TIM_SET_COMPARE(&TIM, MOTOR_CHANNEL1, (uint16_t)Car_Set->Current_PWM_L);
        __HAL_TIM_SET_COMPARE(&TIM, MOTOR_CHANNEL2, (uint16_t)Car_Set->Current_PWM_R);
		
        return; 
    }
	 
	 
    // 速度彻底归零后，关闭 GPIO 省电
	 if(Car_Set->Current_PWM_L == 0 && Car_Set->Current_PWM_R == 0) {
			HAL_GPIO_WritePin(IN_GPIO_Port, AIN1_PIN|AIN2_PIN|BIN1_PIN|BIN2_PIN, GPIO_PIN_RESET);
	 }
}

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
		  float battery_voltage = voltage * 3.0f; // <--- 这里的 3.0 请根据硬件调整
        // 1. 发送电量
        sprintf(send_buf, "[Power,%.2f]", battery_voltage);
        HAL_UART_Transmit(&huart, (uint8_t*)send_buf, strlen(send_buf), 100);

        // 2. 防粘包延时
        HAL_Delay(50);
      
		  
    }
    HAL_ADC_Stop(&ADC);
}

// 专门用来“一键设置”动作，不仅赋值，还顺便调用计算逻辑
void Car_Run(Car_Handle_t *Car_Set, char cmd, uint16_t speed)
{
    Car_Set->cmd = cmd;
    Car_Set->Target_Speed = speed;
    //Car_Set->Mode=mode;
    // 这里可以直接调用计算函数，省得外面再调一次
    Car_Set_Motion(Car_Set); 
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
                    Car_Run(Car_Set, 'S', 0);
                    Car_Speed_Handle(Car_Set);
                    scan_state = 1; 
                } else {
                    Car_Run(Car_Set, 'A', 45);
                    Car_Speed_Handle(Car_Set);
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
                    Car_Run(Car_Set, 'B', 45); 
                    Car_Speed_Handle(Car_Set);
                    
                    wait_timer = 0;
                    scan_state = 6; // 跳转到"正在倒车"状态
                }
                else 
                {
                    // === 触发正常避障 ===
                    // 1. 发送转弯指令
                    if(dist_L > dist_R) Car_Run(Car_Set, 'L', 50); 
                    else                Car_Run(Car_Set, 'R', 50);
                    Car_Speed_Handle(Car_Set);
                    
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
							Car_Run(Car_Set, 'L', 60); 
						 }							 
                   else Car_Run(Car_Set, 'R', 60);
                    
                    Car_Speed_Handle(Car_Set);
                    
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

void Car_Track_Line(Car_Handle_t *Car_Set, MPU6050_t *gyro)
{
   uint8_t L1 = HAL_GPIO_ReadPin(L1_Port, L1_Pin); 
    uint8_t R1 = HAL_GPIO_ReadPin(R1_Port, R1_Pin); 
    uint8_t L2 = HAL_GPIO_ReadPin(L2_Port, L2_Pin); 
    uint8_t R2 = HAL_GPIO_ReadPin(R2_Port, R2_Pin); 

    // 状态机：0=正常, 1=左急转, 2=右急转, 3=缓冲回正期
    static uint8_t State = 0; 
    static uint16_t Timer = 0;

    // ============================================================
    // 第一阶段：状态切换逻辑
    // ============================================================
    
    // --- 正常模式 -> 发现弯道 ---
    if (State == 0) 
    {
        if (L2 == 1) 
        {
            State = 1; // 进左急转
            Timer = 0;
        }
        else if (R2 == 1) 
        {
            State = 2; // 进右急转
            Timer = 0;
        }
    }
    
    // --- 急转模式 -> 发现黑线 -> 进缓冲 ---
    else if (State == 1 || State == 2)
    {
        Timer++;
        // 只有内侧 L1/R1 重新扫到黑线，说明头正了
        if (L1 == 1 || R1 == 1) 
        {
            State = 3; // 【关键】不要直接回0，先进入缓冲期
            Timer = 0; // 重置计时器给缓冲用
        }
        // 超时保护 (防止转过头)
        else if (Timer > 150)
        {
            State = 0; // 强制回正常
        }
    }
    
    // --- 缓冲模式 -> 强制直行 -> 回正常 ---
    else if (State == 3)
    {
        Timer++;
        // 缓冲时间到，才真正回到由传感器接管的正常模式
        if (Timer > RECOVER_TIME) 
        {
            State = 0;
        }
    }

    // ============================================================
    // 第二阶段：执行动作
    // ============================================================

    // --- 状态 1: 左急转 ---
    if (State == 1)
    {
        Car_Wheelspd('L', Car_Set, TURN_SLOW, TURN_FAST);
    }
    
    // --- 状态 2: 右急转 ---
    else if (State == 2)
    {
        Car_Wheelspd('R', Car_Set, TURN_FAST, TURN_SLOW);
    }
    
    // --- 状态 3: 缓冲回正 (强制直行) ---
    // 利用这段时间的直行，把转弯的旋转惯性卸掉
    else if (State == 3)
    {
        Car_Wheelspd('A', Car_Set, TRK_FWD, TRK_FWD);
    }
    
    // --- 状态 0: 正常微调 ---
    else 
    {
        // 此时传感器完全接管
        if (L1 == 1 && R1 == 1)      Car_Wheelspd('A', Car_Set, TRK_FWD, TRK_FWD);
        else if (L1 == 1 && R1 == 0) Car_Wheelspd('L', Car_Set, ADJ_SLOW, ADJ_FAST);
        else if (R1 == 1 && L1 == 0) Car_Wheelspd('R', Car_Set, ADJ_FAST, ADJ_SLOW);
        else                         Car_Wheelspd('A', Car_Set, TRK_FWD, TRK_FWD);
    }
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
            Target_Yaw_Lock = mpu.KalmanAngleZ; 
            Is_Yaw_Locked = 1;
            // 如果有PID，这里应该清除积分，但现在不需要了
        }

        float Yaw_Err = mpu.KalmanAngleZ - Target_Yaw_Lock;
        float Yaw_Rate = mpu.Gz; 

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
        Target_Yaw_Lock = mpu.KalmanAngleZ; 
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
    // ==========================================
    // 1. 死区补偿 (双向)
    // ==========================================
    // 左轮
    if (PWM_L > 2)       PWM_L += DEAD_ZONE_L;  // 正向死区
    else if (PWM_L < -2) PWM_L -= DEAD_ZONE_L;  // 反向死区 (例如 -10 变为 -45)
    else                 PWM_L = 0;             // 噪音归零

    // 右轮
    if (PWM_R > 2)       PWM_R += DEAD_ZONE_R;
    else if (PWM_R < -2) PWM_R -= DEAD_ZONE_R;
    else                 PWM_R = 0;

    // ==========================================
    // 2. 转换为整数并处理方向
    // ==========================================
    int16_t Final_L = (int16_t)PWM_L;
    int16_t Final_R = (int16_t)PWM_R;

    // --- 左轮处理 ---
    if (Final_L >= 0) 
    {
        // 正转逻辑
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_RESET);
    } 
    else 
    {
        // 反转逻辑
        HAL_GPIO_WritePin(AIN1_PORT, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_PORT, AIN2_PIN, GPIO_PIN_SET);
        Final_L = -Final_L; // 【关键】取绝对值，变成正数给PWM
    }

    // --- 右轮处理 ---
    if (Final_R >= 0) 
    {
        // 正转逻辑
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_RESET);
    } 
    else 
    {
        // 反转逻辑
        HAL_GPIO_WritePin(BIN1_PORT, BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_PORT, BIN2_PIN, GPIO_PIN_SET);
        Final_R = -Final_R; // 【关键】取绝对值，变成正数给PWM
    }

    // ==========================================
    // 3. 限幅与输出
    // ==========================================
    if(Final_L > 100) Final_L = 100;
    if(Final_R > 100) Final_R = 100;

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
            Target_Yaw_Lock = mpu.KalmanAngleZ; 
            Is_Yaw_Locked = 1; 
            PID_Reset(&Motor_PID_L);
            PID_Reset(&Motor_PID_R);
        }

        float Yaw_Err = mpu.KalmanAngleZ - Target_Yaw_Lock;
        float Yaw_Rate = mpu.Gz; 
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

void Car_Getspd(long L_val, long R_val)
{
	 // 1. 获取当前周期内的脉冲数
        
        // 一圈总脉冲 = 格子数 * 2 (因为是双边沿)
        float distance_one_pulse = (PI * WHEEL_DIAMETER) / (ENCODER_GRIDS * 2.0f);
        float L_speed = (L_val * distance_one_pulse) / SAMPLE_TIME;
		  float R_speed = (R_val * distance_one_pulse) / SAMPLE_TIME;
        pulse_count_L = 0;
		  pulse_count_R = 0;
			

			 // 2. 低通滤波 (平滑数据)
			 speed_L = (0.7f * speed_L) + (0.3f * L_speed);
			 speed_R = (0.7f * speed_R) + (0.3f * R_speed);
		
	
}

/**
 * @brief 强制停车 (看门狗触发用)
 */
void Car_Force_Stop(Car_Handle_t *Car_Set)
{
	Car_Run(&myCar,'S',0);
	//Car_Set->Target_Speed = 0;
	//Car_Set->cmd='S';
	 // 2. 强制清零"当前速度" (跳过惯性，直接刹死)
    //Car_Set->Current_PWM_L = 0;
    //Car_Set->Current_PWM_R = 0;
	// 1. 下达停车指令
    Car_Set_Motion(Car_Set);

    // 3. 立即写入硬件
    __HAL_TIM_SET_COMPARE(&TIM, MOTOR_CHANNEL1, 0);
    __HAL_TIM_SET_COMPARE(&TIM, MOTOR_CHANNEL2, 0);
    HAL_GPIO_WritePin(IN_GPIO_Port, AIN1_PIN|AIN2_PIN|BIN1_PIN|BIN2_PIN, GPIO_PIN_RESET);
}




