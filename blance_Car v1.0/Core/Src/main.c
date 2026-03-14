/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "oled.h"
#include "font.h"
#include <stdio.h>
#include <stdlib.h>
#include "dht11.h"
// #include "max7219.h"
#include "key.h"
// #include "MPU6050.h"
#include "Blucar.h"
#include "PID.h"
#include "mpu6050.h"
#include "HCSR04.h"
#include "filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_MAX_LEN 1024 // 定义足够大的接收缓冲区
#define RX_BUFFER_SIZE 512
#define RX_BLU_SIZE 256 // 定义缓冲区大小
#define free_state 0
#define singlepress 1
#define doublepress 2
#define longpress 3
#define WEATHER_KEY "SkauKTjXsbUijiTkw"
#define WEATHER_CITY "guangzhou"
// OneNET 配置信息
#define ONENET_PID "NW4ydVqKGU" // 产品 ID
#define ONENET_DEVICE "pro1"	// 设备名称
// 【注意】这里必须填计算出来的 Token (res=products/PID/devices/pro1)
#define ONENET_TOKEN "version=2018-10-31&res=products%2FNW4ydVqKGU%2Fdevices%2Fpro1&et=2541412770&method=md5&sign=2vCMFmP%2FtvBfhgw8YwXQDg%3D%3D"

// OneNET 物模型上报 Topic (固定格式)
#define TOPIC_POST "$sys/" ONENET_PID "/" ONENET_DEVICE "/thing/property/post"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_TypeDef Motor_PID_L;
PID_TypeDef Motor_PID_R;
// MPU6050 MM;
//extern MPU6050_t mpu;
extern MPU6050 mpu;
extern float Speed_Out;
extern float Upright_Out;

KalmanFilter KK1;
KalmanFilter KK2;

//循迹环PID参数
extern float Track_Kp;
extern float Track_Kd;
//速度环PID参数
extern float Speed_Kp;
extern float Speed_Ki;
//直立环PID参数
extern float Std_Kp;
extern float Std_Kd;
//机械零点
extern float BLANCE_ANGLE;

uint16_t pid;
char temp_str[10] = {0};
char weather_str[20] = {0};
char Car_dir;
// volatile uint8_t Car_Mode=1;
volatile uint8_t Blu_rx_buffer[RX_BUFFER_SIZE]; // 接收缓冲区
volatile uint8_t Blu_rx_len = 0;				// 接收到的数据长度
volatile uint8_t Blu_rx_ready = 0;				// 标志位：1表示收到新指令
volatile uint8_t RxBuffer[RX_MAX_LEN];			// DMA 原始接收桶
volatile uint16_t RxLen = 0;					// 实际接收长度
volatile uint8_t RecvEndFlag = 0;				// 接收完成标志
volatile static uint32_t stopcartime = 0;
/*
// === 惯性控制变量 ===
volatile int16_t Target_PWM_L = 0;  // 左轮目标速度
volatile int16_t Target_PWM_R = 0;  // 右轮目标速度
volatile int16_t Current_PWM_L = 0; // 左轮实际速度 (逐渐变化)
volatile int16_t Current_PWM_R = 0; // 右轮实际速度 (逐渐变化)
volatile uint8_t Speed_Update_Flag = 0; // 定时器触发标志
*/
volatile uint8_t Car_Mode = 1;			// 内部记录模式
volatile uint8_t Speed_Update_Flag = 0; // 内部记录时间标志
char http_buffer[512];					// 存放 HTTP 请求报文
char len_cmd[128];						// 存放长度指令
char process_buff[RX_BUFFER_SIZE];		// 【新增】用于解析的备份缓冲区
char rxbuff[RX_BUFFER_SIZE];
char Txbuff[35];
char message[256];
char dht11message[256];
char adc[10];
char onenetsend[512];
uint8_t oledflag = 0;
uint8_t txflag = 0;
volatile uint8_t rxflag = 0;
uint8_t sendflag = 0;
uint8_t weathergetflag = 0;
uint8_t lastweaflag = 0;
uint8_t btrysendflag = 0;
uint8_t stopcarflag = 0;
uint8_t SR04_send_flag = 0;
uint8_t speed_countflag = 0;
uint8_t Car_Trackflag = 0;
volatile uint8_t rx_index = 0;
volatile uint8_t new_data_available = 0;
char oled[50];
uint8_t temperature = 0;
uint8_t humidity = 0;
uint16_t Car_speed = 0;
uint8_t doubleflag;
uint8_t singleflag;
uint8_t longflag;
uint8_t keyscanflag;
uint8_t wificheckflag;
uint8_t sr04_Readflag = 0;
uint8_t Mpuflag = 0;
uint8_t Actual_Speed = 35;
long pwmvalue;
float PID_L = 0;
float PID_R = 0;
// 全局变量记录脉冲
volatile long pulse_count_L = 0;
volatile long pulse_count_R = 0;
float Speed_L = 0.0f; // 当前速度 (cm/s)
float Speed_R = 0.0f;
/* 发送电量变量 */
uint32_t Last_Send_Time = 0;
char send_buf[30];

uint32_t Last_Cmd_Time = 0; // 全局变量，记录上次收到指令的时间
char line2[] = "Host: api.seniverse.com\r\n";
char line3[] = "Connection: close\r\n";
char line4[] = "\r\n"; // 【关键】最后的空行

float Target_Yaw = 0;	   // 目标朝向
uint8_t Yaw_Lock_Flag = 0; // 锁定标志位


// MPU6050 MM;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void send_at_command(const char *cmd);
void Get_Temp(void);
uint8_t MQTTRec(char *data);
void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle);
void MQTThandle(void);
void oledshow(char *text, uint8_t x, uint8_t y);
void Get_Weather(void);
void Onenet_connect(void);
void Parse_Weather_Smart(void);
void Control_Car(uint8_t mode, uint8_t cmd, uint8_t speed);
void Car_btry(void);
void Car_Speed_Loop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM3_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_ADC2_Init();
	/* USER CODE BEGIN 2 */
	/*
	  send_at_command("AT+RST");
	  HAL_Delay (1000);
	  */

	SSD1306_Init();
	SSD1306_Clear();
	

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // servo IN1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // servo IN2

	// onenet init

	// send_at_command("AT+CWJAP=\"Neo8\",\"2271902787\"");
	// HAL_Delay(1000);

	// send_at_command("AT+CWJAP=\"Xiaomi_D44B\",\"2271902787Ab\"");
	// HAL_Delay (4000);
	// send_at_command("AT+CIPSTART=\"TCP\",\"api.seniverse.com\",80");

	//DHT11_Init();
	SR04_Init();
	Motor_Init(); // 电机初始化
	PID_Init(&Motor_PID_L, 0.6f, 0.2f, 0.0f, 100.0f);
	PID_Init(&Motor_PID_R, 0.6f, 0.2f, 0.0f, 100.0f);
	Track_PID_Init(10, 5);
	// HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	MPU6050_init(&hi2c1);
	// MPU6050_Init(&mpu, &hi2c1,
    //                  MPU6050_GYRO_500DPS,
    //                  MPU6050_ACCEL_2G,
    //                  MPU6050_DLPF_44HZ);
	 // ★ 让姿态解算收敛 (Madgwick 前400次是快速收敛期)
    // 上电后静止等待2秒让四元数稳定
    for (uint16_t i = 0; i < 400; i++)
    {
        MPU6050_Get_Angle_Plus(&mpu);
        HAL_Delay(5);
    }
	//HAL_Delay(1000);
	Encoder_Init_Start();
	 // ★ 把当前姿态设为零点 (相当于自动找 BLANCE_ANGLE!)
    MPU6050_Set_Angle0(&mpu,&BLANCE_ANGLE);
	// 3. 执行校准 (此时千万不要动小车！)
	//MPU6050_Calibrate_Z(&hi2c1);
	//HAL_Delay(1000);
	// MPU6050_init(&hi2c1);
	// KalmanFilter_Init(&KK1,0.01,0.1,0,1);
	// KalmanFilter_Init(&KK2,0.01,0.1,0,1);
	// HAL_TIM_Base_Start(&htim4);
	// HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxbuff[0], 1);

	// 1. 开启 IDLE 中断
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	// 2. 开启 DMA 接收 (CPU 可以去玩了，DMA 会干活)
	// HAL_UART_Receive_DMA(&huart1, (uint8_t*)Blu_rx_buffer, RX_BLU_SIZE);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)Blu_rx_buffer, RX_BLU_SIZE); // 手动开启串口DMA模式接收数据
	//__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	HAL_TIM_Base_Start_IT(&htim4);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
	// HAL_ADC_Start(&hadc1);
	// HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
    oledshow("ok", 0, 20);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
        //  static uint16_t L_TEST,R_TEST=0;
      //Car_Motor_Output('A',2,5000,5000);
		if(oledflag){
			oledflag=0;
			
			SSD1306_UpdateScreen();
		}

		if (Mpuflag)
		{
			Mpuflag = 0;
            MPU6050_Get_Angle_Plus(&mpu);
			Car_selfbalance(Car_speed,Car_dir);
           
		}

		// HAL_Delay(1000);

		/* main.c 的 while(1) 循环内 */

		if (myCar.Mode == 4)
		{
			if (Car_Trackflag)
			{
				Car_Trackflag = 0;
				Car_Track_Line(&myCar, &mpu); // 循迹函数
			}
		}

		if (sr04_Readflag)
		{
			sr04_Readflag = 0;
			if (myCar.Mode == 4)
			{
				// Car_AutoRun(&myCar);
			}
		}

		if (speed_countflag)
		{
			speed_countflag = 0;

			

			/*
			   // 3. PID 计算 (现在正负号对齐了，PID 就能正常工作了)
			   float PWM_L,PWM_R ;

		  // 3. 特殊处理：如果目标速度是 0，直接关断，不要用 PID 反转刹车
		  // 这能解决“松手高速甩一下”的问题
		   if (Car_speed == 0)
		   {
				PWM_L = 0;
				PWM_R = 0;

				// 只有在确定要停车时，才复位 PID，防止下次启动冲出去
				PID_Reset(&Motor_PID_L);
				PID_Reset(&Motor_PID_R);
		   }
		   else
		   {
				// 正常行驶：进行 PID 计算
				// 注意：这里绝对不要调用 PID_Reset！

					PWM_L = PID_Compute(&Motor_PID_L, speed_L, Car_speed); // 注意参数顺序：通常是(PID, 目标, 实际)
					PWM_R = PID_Compute(&Motor_PID_R, speed_R, Car_speed);

				// 可选：在这里可以加一个“防反转”逻辑
				// 如果你不希望车子为了减速而反转电机（保护牙箱），可以限制最小值为 0
				// if(Car_dir == 'A' && PWM_L < 0) PWM_L = 0;
				// if(Car_dir == 'A' && PWM_R < 0) PWM_R = 0;
		   }
		   */
			/*
		   MPU6050_Get_Angle(&MM);
		 float newroll=KalmanFilter_Update(&KK1,MM.roll);
		  float newpitch=KalmanFilter_Update(&KK2,MM.pitch);
			*/
			// MPU6050_Read_All(&hi2c1,&mpu);

			// 4. 执行输出
			// Car_Wheelspd(Car_dir,&myCar,(int16_t)PWM_L, (int16_t)PWM_R);

			// 修改你的打印语句，增加 PWM_L 和 PWM_R
			// sprintf(Txbuff, "[L:PWM=%.1f] [R:PWM=%.1f]",PWM_L, PWM_R);
			//sprintf(send_buf, "[Spd,%.1f,%.1f]", speed_L, speed_R);
			//sprintf(Txbuff, "[x:%.1f,y:%.1f,z:%.1f]", mpu.KalmanAngleX, mpu.KalmanAngleY, mpu.KalmanAngleZ);
			// sprintf(Txbuff, "[x:%.1f,y:%.1f,z:%.1f]", mpu.angle_roll,
            //        mpu.angle_pitch,
            //        mpu.angle_yaw);
				   
				//    sprintf(oled, "x:%.1f", mpu.angle_roll);
				//    oledshow(oled, 0,0);
				//    sprintf(oled, "y:%.1f", mpu.angle_pitch);
				//    oledshow(oled, 0, 20);
				//    sprintf(oled, "z:%.1f", mpu.angle_yaw);
				//    oledshow(oled, 0, 40);
				// float Angle_X = mpu.angle_roll;  
				// float Gyro_X  = mpu.gyro_dps[0]; 
				float Angle_X=mpu.roll;
				float Gyro_X=mpu.GyroX/16.4;
				sprintf(oled, "L:%.1f",Speed_L);
				   oledshow(oled, 0,20);
				   sprintf(oled, "gx:%.1f", Gyro_X);
				   oledshow(oled, 0, 30);
				    sprintf(oled, "a:%d",mpu.AccZ);
				   oledshow(oled, 0,40);

				  sprintf(oled, "x:%.1f",Angle_X);
				   oledshow(oled, 50 , 20);				  
				     sprintf(oled, "R:%.1f", Speed_R);
				   oledshow(oled, 50, 30);
				  sprintf(oled, "u:%.1f",Upright_Out);
				   oledshow(oled, 50, 40);
				// sprintf(oled, "L:%.1f",Speed_L);
				// oledshow(oled, 0,0);
				// sprintf(oled, "R:%.1f",Speed_R);
				// oledshow(oled, 0,20);
		}

		// if (keyscanflag == 1)
		// {
		// 	keyscanflag = 0;
		// 	Keyscan();
		// }

		// === 电压测量与发送 (没指令时每5秒发一次) ===
		if (btrysendflag)
		{

			btrysendflag = 0;
			Car_Report_Status();
		}

		if (Speed_Update_Flag)
		{
			Speed_Update_Flag = 0;
			// Motor_PID_R.Target=Car_speed;
			// pid =PID_Compute(&Motor_PID_R,35);
			if (myCar.Mode != 3 && myCar.Mode != 4)
			{
				//Car_Wheelspd(Car_dir, &myCar, Car_speed, Car_speed);
			}
			// Car_Speed_Handle(&myCar);
		}

		if (SR04_send_flag)
		{
			SR04_send_flag = 0;

			// 第一步：触发测距并更新全局变量
			g_Distance = SR04_Read();

			// 第二步：把距离数据打包进 send_buf
			sprintf(send_buf, "[Sonic,%.2f]\r\n", g_Distance);

			// 第三步：把打包好的 send_buf 发送出去
			HAL_UART_Transmit(&huart2, (uint8_t *)send_buf, strlen(send_buf), 100);

			// 如果你还需要发陀螺仪的数据，紧接着发 Txbuff
			HAL_UART_Transmit(&huart2, (uint8_t *)Txbuff, strlen(Txbuff), 100);
			// printf("[%.2f,%.2f,%.4f]\n",MM.pitch,MM.roll,MM.yaw);//使用串口发送数据
		}

		// 蓝牙接收处理
		//  检查是否有新数据
		if (Blu_rx_ready == 1)
		{
			Blu_rx_ready = 0; // 清除标志位

			HAL_UART_Transmit(&huart, (uint8_t *)&Blu_rx_buffer, Blu_rx_len, 100);

			
			
			// 永远只找最后一个 '['，忽略前面积压的旧数据
			//char *p = strrchr((char *)Blu_rx_buffer, '[');

			// ? 终极解析算法：寻找最后一个【完整】的指令包
    // ============================================================
			char *p = NULL; 
			// 1. 先找最后一个闭合括号 ']'，确保指令是完整的
			char *end_ptr = strrchr((char *)Blu_rx_buffer, ']'); 
			
			if (end_ptr != NULL) 
			{
				// 2. 从 ']' 的位置开始，倒退着往前找，直到找到匹配的 '['
				char *start_ptr = end_ptr;
				while (start_ptr >= (char *)Blu_rx_buffer && *start_ptr != '[') 
				{
					start_ptr--;
				}
				
				// 3. 如果找到了 '['，说明截取出了一个 100% 完美的包！
				if (*start_ptr == '[') 
				{
					p = start_ptr; // 把完美的指针交给 p，让后面的代码去解析
				}
			}
			// 1. 定义临时变量 (int 类型)
			int temp_speed = 0;
			int temp_mode = 0;
			// if(strstr((char*)Blu_rx_buffer,"[Car")!=NULL)
			// 解析接收的指令
			if (p != NULL)
			{
				if (sscanf(p, "[Car,%[^,],%d]", &Car_dir, &temp_speed) == 2)
				{

					stopcartime = 0;
					stopcarflag = 0;
					if (myCar.Mode != 3 && myCar.Mode != 4)
					{
						// 3. 安全赋值 (把 int 转回 uint16_t)
						Car_speed = (uint16_t)temp_speed;
						;
						//如果速度为0，强制复位所有状态，防止下次启动乱跑
						if (Car_speed == 0)
						{
							Car_dir = 'S';  // ? 新增：明确告诉全身，现在是停车状态！
							PID_Reset(&Motor_PID_L);
							PID_Reset(&Motor_PID_R);
							//Car_Wheelspd('S', &myCar, 0, 0); // 立即执行一次停车
						}
						// Car_Run(&myCar,Car_dir,Car_speed);
					}
					// printf("Car_dir:%c,speed:%d",Car_dir,Car_speed);
				}

				// 获取小程序控制模式
				if (sscanf(p, "[Mode,%d]", &temp_mode) == 1)
				{
					stopcartime = 0;
					//Set_Servo_Angle(&htim1, TIM_CHANNEL_1, 90);
					myCar.Mode = (uint8_t)temp_mode;
				}

				// 接收小程序指令，立马发送一次电量
				if (strstr(p, "[Connect]") != NULL)
				{
					Car_Report_Status();
				}

				// ====== 4. ? 终极三段式 PID 调参解析 ======
				char loop_name[15] = {0}; // 存环名，如 spd, track, upright
				char key_name[10] = {0};  // 存参数名，如 Kp, Kd
				float var_value = 0.0f;   // 存具体数值

				// 魔法提取：一口气抓取 环名、键名、数值 (必须成功提取3个才往下走)
				if (sscanf(p, "[%[^,],%[^,],%f]", loop_name, key_name, &var_value) == 3)
				{
					//oledshow("666", 0, 40); // 屏幕显示 666，证明解析通道彻底打通！

					// ? 极其关键的拦截网！防止 [Car,F,50] 混进来捣乱
					// 只有当环名是我们规定的这三个时，才允许往下执行
					if (strcmp(loop_name, "upright") == 0 || 
						strcmp(loop_name, "Track") == 0 || 
						strcmp(loop_name, "Spd") == 0 ||
						strcmp(loop_name, "Zero") == 0)
					{
						// 绝对安全的字符串转浮点数
					
						
						//oledshow("666", 0, 40); // 屏幕显示 666，证明成功破防！

						/* ----------- A. 直立环 (Upright) ----------- */
						if (strcmp(loop_name, "upright") == 0)
						{
							if (strcmp(key_name, "Kp") == 0) {
								Std_Kp = var_value; // ?? 填写真实的直立 Kp 变量名
							} 
							else if (strcmp(key_name, "Kd") == 0) {
								Std_Kd = var_value; // ?? 填写真实的直立 Kd 变量名
							}
							
						}
						
						/* ----------- B. 循迹环 (Track) ----------- */
						else if (strcmp(loop_name, "Track") == 0)
						{
							if (strcmp(key_name, "Kp") == 0) {
								Track_Kp = var_value;
							} 
							else if (strcmp(key_name, "Kd") == 0) {
								Track_Kd = var_value;
							}
							else if (strcmp(key_name, "Speed") == 0) {
								// Track_Speed = var_value; 
							}
							
						}
						
						/* ----------- C. 速度环 (Speed) ----------- */
						else if (strcmp(loop_name, "Spd") == 0)
						{
							if (strcmp(key_name, "Kp") == 0) {
								Speed_Kp = var_value; // ?? 取消注释并替换变量
							} 
							else if (strcmp(key_name, "Ki") == 0) {
								Speed_Ki = var_value; // ?? 取消注释并替换变量
							}
							
							// PID_Init(&Motor_PID_L, Speed_Kp, Speed_Ki, 0, 100); 
							// PID_Init(&Motor_PID_R, Speed_Kp, Speed_Ki, 0, 100);
							
						}

						//机械零点
						if (strcmp(loop_name, "Zero") == 0)
						{
							if (strcmp(key_name, "Z") == 0) {
								BLANCE_ANGLE = var_value; // ?? 取消注释并替换变量
							} 
						}
				}
				memset((char *)Blu_rx_buffer, 0, RX_BLU_SIZE);
			}
		}
			Blu_rx_len = 0;
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)Blu_rx_buffer, RX_BLU_SIZE); /// 接收完毕后重启串口DMA模式接收数据
																						  /*
																						  //舵机控制
																						  if(strstr((char*)Blu_rx_buffer,"1")!=NULL){
																							 Set_Servo_Angle(&htim2,TIM_CHANNEL_1,0);
																						 }
																			  
																						  int n;
																						 static uint32_t last_servo_time = 0; // 记录上次动舵机的时间
																						  static uint8_t last_servo_angle = 255; // 初始值设为一个不可能的角度，确保第一次能执行
																							 if (sscanf((char*)Blu_rx_buffer, "[slider,1,%d]", &n) == 1)
																						  {
																								 // 3. 安全限幅 (防止手机发来 200 把舵机烧了)
																								 if(n < 0) n = 0;
																								 if(n > 100) n = 100;
																			  
																								 // 4. 映射计算
																								 // 输入: 0~100
																								 // 输出: 0~180 度
																								 // 公式: 角度 = (数值 / 100.0) * 180
																								 uint8_t servo_angle = (uint8_t)(n * 1.6);
																										 printf("n为:%d",n);
																			  
																									 if (abs(servo_angle - last_servo_angle) > 3)
																									 {
																										  // === 新增：时间限制 ===
																										  // 只有距离上次操作超过 100ms (0.1秒) 才允许再次动舵机
																										  if (HAL_GetTick() - last_servo_time > 100)
																										  {
																											   Set_Servo_Angle(&htim2, TIM_CHANNEL_1, servo_angle);
																											   last_servo_angle = servo_angle;
																											   last_servo_time = HAL_GetTick(); // 更新时间戳
																										  }
																									 }
																			  
																						  }
																			  
																						 // 处理接收到的蓝牙指令
																						 // 假设指令是单个字符：rx_buffer[0]
																						 // 如果指令是字符串（如 "GO"），可以用 strcmp 判断
																			  
																						 // 简单示例：取第一个字节控制
																						 //Control_Car(Blu_rx_buffer[0]);
																						  //printf("数据:%d:",Blu_rx_buffer[0]);
																						 // 调试：如果你有 OLED，可以在这里打印 rx_buffer 看看收到了啥
																						   */
		}

		// 看门狗,超时强制停车,防止小车失控
		if (stopcarflag)
		{
			stopcarflag = 0;

			
				Car_speed = 0; // 必须清零！否则下个 10ms 循环又把它启动了
				Car_dir = 'S'; // 方向归位
				Car_Force_Stop(&myCar);
			
		}
		

		if (weathergetflag == 1)
		{

			lastweaflag = weathergetflag;
			weathergetflag = 0;
		}

		if (sendflag)
		{

			Onenet_connect();
			sendflag = 0;
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
	return ch;
}

void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle)
{
	uint16_t pulse_width = 500 + (angle / 180.0) * 2000;
	__HAL_TIM_SET_COMPARE(htim, Channel, pulse_width);
	HAL_Delay(15);
}

uint8_t MQTTRec(char *data)
{
	if (strstr(rxbuff, "+MQTTSUBRECV:") != NULL && strstr(rxbuff, data) != NULL)
	{
		return 1;
	}
	else
		return 0;
}

void oledshow(char *text, uint8_t x, uint8_t y)
{
	 char buf[22];
    uint8_t i = 0;
    while (text[i] != '\0' && i < 16) {
        buf[i] = text[i];
        i++;
    }
    while (i < 16) {
        buf[i] = ' ';
        i++;
    }
    buf[i] = '\0';
    
    SSD1306_GotoXY(x, y);
    SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);
	// SSD1306_GotoXY(x, y);
	
	// SSD1306_Puts(text, &Font_7x10, 1);
	//SSD1306_UpdateScreen();
}

void Onenet_connect(void)
{
	if (lastweaflag == 1)
	{
		HAL_UART_Transmit(&huart2, (uint8_t *)"AT+CIPCLOSE\r\n", 13, 100);
		HAL_Delay(500);
		snprintf(onenetsend, sizeof(onenetsend), "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n", ONENET_DEVICE, ONENET_PID, ONENET_TOKEN);
		send_at_command(onenetsend);
		// send_at_command("AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",ONENET_DEVICE,ONENET_PID,ONENET_TOKEN);
		HAL_Delay(500);
		send_at_command("AT+MQTTCONN=0,\"mqtts.heclouds.com\",1883,1\r\n");
		// 3. 连接 OneNET MQTT 服务器

		HAL_Delay(1000); // 连接需要时间，必须延时！
		snprintf(onenetsend, sizeof(onenetsend), "AT+MQTTSUB=0,\"$sys/%s/%s/thing/property/post/reply\",0\r\n", ONENET_PID, ONENET_DEVICE);
		send_at_command(onenetsend);
		// send_at_command("AT+MQTTSUB=0,\"$sys/NW4ydVqKGU/pro1/thing/property/post/reply\",0\r\n");

		// 4. 构造物模型 JSON 数据
		// 假设 OneNET 平台上有 identifier 为 "temperature" 和 "humidity" 的属性

		HAL_Delay(1000);
		snprintf(onenetsend, sizeof(onenetsend), "AT+MQTTSUB=0,\"$sys/%s/%s/thing/property/set\",0\r\n", ONENET_PID, ONENET_DEVICE);
		send_at_command(onenetsend);
		// send_at_command("AT+MQTTSUB=0,\"$sys/NW4ydVqKGU/pro1/thing/property/set\",0\r\n");
		HAL_Delay(1000);
		lastweaflag = 0;
	}

	Get_Temp();
	snprintf(onenetsend, sizeof(onenetsend), "AT+MQTTPUB=0,\"%s\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"hum\\\":{\\\"value\\\":%d}\\,\\\"temp\\\":{\\\"value\\\":%d}}}\",0,0\r\n", TOPIC_POST, humidity, temperature);
	// snprintf(onenetsend,sizeof(onenetsend),"AT+MQTTPUB=0,\"%s\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"hum\\\":{\\\"value\\\":%d}\\,\\\"temp\\\":{\\\"value\\\":%d}\\,\\\"weather\\\":{\\\"value\\\":\\\"%s\\\"}}}\",0,0\r\n",TOPIC_POST,humidity,temperature,weather_str);
	send_at_command(onenetsend);
	if (weathergetflag)
	{
		snprintf(onenetsend, sizeof(onenetsend), "AT+MQTTPUB=0,\"$sys/NW4ydVqKGU/pro1/thing/property/post\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"weather\\\":{\\\"value\\\":\\\"%s\\\"}}}\",0,0\r\n", weather_str);
		send_at_command(onenetsend);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		// 1. 保存接收到的数据长度 (关键修正！)
		Blu_rx_len = Size;

		Blu_rx_ready = 1;
		//__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);		   		// 手动关闭DMA_IT_HT中断
		// 清除接收缓存
	}
}

/*
// --- 核心优化：处理空闲中断 (IDLE) ---
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		// 检查是否是空闲中断 (数据发完了，总线停了)
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
		{
			// 1. 清除 IDLE 标志
			__HAL_UART_CLEAR_IDLEFLAG(huart);

			// 2. 停止 DMA 接收 (为了计算收到了多少字节)
			HAL_UART_DMAStop(huart);

			// 3. 计算数据长度
			// 总长度 - DMA剩余待接收长度 = 实际接收长度
			Blu_rx_len = RX_BLU_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

			// 4. 只有收到数据才处理
			if(Blu_rx_len > 0)
			{
				Blu_rx_ready = 1; // 告诉主循环：有一整包数据到了！

				// 为了配合 strstr，我们在数据末尾补一个 '\0'
				// 注意不要越界
				if(Blu_rx_len < RX_BLU_SIZE) Blu_rx_buffer[Blu_rx_len] = 0;
			}

			// 5. 重启 DMA，准备接下一包
			HAL_UART_Receive_DMA(huart, (uint8_t*)Blu_rx_buffer, RX_BLU_SIZE);
		}
	}

}
*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM4)
	{
		static uint32_t keycount = 0;
		static uint32_t btrysendtime = 0;
		static uint32_t sr04_Readtime = 0;
		static uint32_t sr04_sendtime = 0;
		static uint32_t speed_counttime = 0;
		static uint32_t oled_showtime = 0;

 			 MPU6050_Get_Angle_Plus(&mpu);
			//MPU6050_Read_All(&hi2c1, &mpu);
			//MPU6050_Update(&mpu);
			Car_Getspd(&Speed_L,&Speed_R);
		
			Car_selfbalance(Car_speed,Car_dir);
           

		if (myCar.Mode == 4)
		{
			Car_Trackflag = 1;
		}
		//Speed_Update_Flag = 1;

		if (++speed_counttime >= 1)
		{
			speed_countflag = 1;
			speed_counttime = 0;
		}

		if (++oled_showtime >= 40)
		{
			oledflag = 1;
			oled_showtime = 0;
		}


		Mpuflag = 1;
		// if (++keycount >= 1)
		// { // 每10ms扫描一次按键
		// 	keycount = 0;
		// 	keyscanflag = 1;
		 	
		// }

		if (myCar.Target_Speed == 0)
		{
			if (++btrysendtime > 1000)
			{
				btrysendtime = 0;
				btrysendflag = 1;
			}
		}
		else
			btrysendtime = 0;

		// if (myCar.Mode != 3 && myCar.Mode != 4)
		// {

		// 	if (++stopcartime > 60)
		// 	{
		// 		stopcartime = 31;
		// 		stopcarflag = 1;
		// 	}
		// }
		// else
		// {
		// 	stopcartime = 0; // 如果是模式3或4，一直喂狗，保持活跃
		// }

		if (++sr04_Readtime >= 20)
		{
			sr04_Readflag = 1;
			sr04_Readtime = 0;
		}

		// if (++sr04_sendtime >= 200)
		// {
		// 	sr04_sendtime = 0;
		// 	SR04_send_flag = 1;
		// }
		/*
		// 5秒定时
		if(++weathersendcount >= 6000)
		{
		  weathersendcount = 0;
		  weathergetflag = 1;  // 触发数据发送
		}
		if(weathergetflag){
			if(++dht11sendcount >= 500){
				dht11sendcount=0;
				sendflag=1;
			}
		}
		*/
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_6) // 假设你接的是 PA0
	{
		pulse_count_L++;
	}
	if (GPIO_Pin == GPIO_PIN_4) // 假设你接的是 PA0
	{
		pulse_count_R++;
	}
}

void send_at_command(const char *cmd)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), 100);
	HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
}
void Get_Temp(void)
{
	DHT11_Data_TypeDef DHT11_Data;
	if (DHT11_Read_TempAndHumidity(&DHT11_Data) == SUCCESS)
	{
		temperature = DHT11_Data.temperature;
		humidity = DHT11_Data.humidity;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
