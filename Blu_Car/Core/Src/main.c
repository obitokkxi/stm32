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
//#include "max7219.h"
#include "key.h"
#include "MPU6050.h"
#include "Blucar.h"
#include "PID.h"
#include "HCSR04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define RX_MAX_LEN 1024       // 定义足够大的接收缓冲区
#define RX_BUFFER_SIZE 512
#define RX_BLU_SIZE 256  // 定义缓冲区大小
#define free_state 0
#define singlepress 1
#define doublepress 2
#define longpress 3
#define WEATHER_KEY "SkauKTjXsbUijiTkw" 
#define WEATHER_CITY "guangzhou"
// OneNET 配置信息
#define ONENET_PID      "NW4ydVqKGU"       // 产品 ID
#define ONENET_DEVICE   "pro1"             // 设备名称
// 【注意】这里必须填计算出来的 Token (res=products/PID/devices/pro1)
#define ONENET_TOKEN "version=2018-10-31&res=products%2FNW4ydVqKGU%2Fdevices%2Fpro1&et=2541412770&method=md5&sign=2vCMFmP%2FtvBfhgw8YwXQDg%3D%3D" 

// OneNET 物模型上报 Topic (固定格式)
#define TOPIC_POST      "$sys/" ONENET_PID "/" ONENET_DEVICE "/thing/property/post"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_TypeDef Motor_PID_L; 
PID_TypeDef Motor_PID_R;

uint16_t pid;
 char temp_str[10] = {0};
char weather_str[20] = {0};
char Car_dir;
//volatile uint8_t Car_Mode=1;
volatile uint8_t Blu_rx_buffer[RX_BUFFER_SIZE]; // 接收缓冲区
volatile uint8_t Blu_rx_len = 0; // 接收到的数据长度
volatile uint8_t Blu_rx_ready = 0; // 标志位：1表示收到新指令
volatile uint8_t RxBuffer[RX_MAX_LEN];  // DMA 原始接收桶
volatile uint16_t RxLen = 0;            // 实际接收长度
volatile uint8_t RecvEndFlag = 0;       // 接收完成标志
volatile static uint32_t stopcartime=0;
/*
// === 惯性控制变量 ===
volatile int16_t Target_PWM_L = 0;  // 左轮目标速度
volatile int16_t Target_PWM_R = 0;  // 右轮目标速度
volatile int16_t Current_PWM_L = 0; // 左轮实际速度 (逐渐变化)
volatile int16_t Current_PWM_R = 0; // 右轮实际速度 (逐渐变化)
volatile uint8_t Speed_Update_Flag = 0; // 定时器触发标志
*/
volatile uint8_t  Car_Mode = 1;          // 内部记录模式
volatile uint8_t  Speed_Update_Flag = 0; // 内部记录时间标志
char http_buffer[512]; // 存放 HTTP 请求报文
char len_cmd[128];      // 存放长度指令
char process_buff[RX_BUFFER_SIZE]; // 【新增】用于解析的备份缓冲区
char rxbuff[RX_BUFFER_SIZE];
char Txbuff[35];
char message[256];
char dht11message[256];
char adc[10];
char onenetsend[512];	  
uint8_t oledflag=0;
uint8_t txflag=0;
volatile uint8_t rxflag=0;
uint8_t sendflag=0;
uint8_t weathergetflag=0;
uint8_t lastweaflag=0;
uint8_t btrysendflag=0;
uint8_t stopcarflag=0;
uint8_t SR04_send_flag=0;
uint8_t speed_countflag=0;
uint8_t Car_Trackflag=0;
volatile uint8_t rx_index = 0;
volatile uint8_t new_data_available = 0;
char oled[50];
uint8_t temperature = 0;
uint8_t humidity = 0;
uint16_t Car_speed=0;
uint8_t doubleflag;
uint8_t singleflag;
uint8_t longflag;
uint8_t keyscanflag;
uint8_t wificheckflag;
uint8_t sr04_Readflag=0;
uint8_t Actual_Speed=35;
long pwmvalue;
float PID_L=0;
float PID_R=0;
// 全局变量记录脉冲
volatile long pulse_count_L = 0;
volatile long pulse_count_R = 0;
float speed_L = 0.0f;         // 当前速度 (cm/s)
float speed_R = 0.0f;  
/* 发送电量变量 */
uint32_t Last_Send_Time = 0;
char send_buf[30];

uint32_t Last_Cmd_Time = 0; // 全局变量，记录上次收到指令的时间
char line2[] = "Host: api.seniverse.com\r\n";
char line3[] = "Connection: close\r\n";
char line4[] = "\r\n"; // 【关键】最后的空行
//MPU6050 MM;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void send_at_command(const char* cmd);
void Get_Temp(void);
uint8_t MQTTRec(char * data);
void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle);
void MQTThandle(void);
void oledshow(char *text,uint8_t x,uint8_t y);
void Get_Weather(void);
void Onenet_connect(void);
void Parse_Weather_Smart(void);
void Control_Car(uint8_t mode,uint8_t cmd,uint8_t speed);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /*
    send_at_command("AT+RST");
	HAL_Delay (1000);
	*/
	
	SSD1306_Init();
	SSD1306_Clear();
	oledshow("ok",0,20);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);  
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // servo IN1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // servo IN2
	
	//onenet init
	
	//send_at_command("AT+CWJAP=\"Neo8\",\"2271902787\"");
	//HAL_Delay(1000);
	
	//send_at_command("AT+CWJAP=\"Xiaomi_D44B\",\"2271902787Ab\"");
	//HAL_Delay (4000);
//send_at_command("AT+CIPSTART=\"TCP\",\"api.seniverse.com\",80");
    
	
	DHT11_Init();
	SR04_Init();
	Motor_Init(); //电机初始化
	PID_Init(&Motor_PID_L, 1.5f, 0.0f, 0.5f, 100.0f);
	PID_Init(&Motor_PID_R, 1.5f, 0.0f, 0.5f, 100.0f);
	Track_PID_Init();
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	
	//HAL_TIM_Base_Start(&htim4);  	 
	//HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxbuff[0], 1);
	
	// 1. 开启 IDLE 中断
  //__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 
  // 2. 开启 DMA 接收 (CPU 可以去玩了，DMA 会干活)
  // HAL_UART_Receive_DMA(&huart1, (uint8_t*)Blu_rx_buffer, RX_BLU_SIZE);
	
	 HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t*)Blu_rx_buffer,RX_BLU_SIZE);	//手动开启串口DMA模式接收数据
  	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);	
	
	HAL_TIM_Base_Start_IT(&htim4);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0); 
	//HAL_ADC_Start(&hadc1);
	//HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
	  /*陀螺仪	
	  MPU6050_Get_Angle(&MM);  
	  printf("pitch:%.2f\r\n",MM.pitch);
	  printf("roll:%.2f\r\n",MM.roll);
	  printf("yaw:%.2f\r\n",MM.yaw);
	  printf("%d\r\n",speed);
	  HAL_Delay(1000);
	  */
	  /* main.c 的 while(1) 循环内 */
	  
	  if(myCar.Mode == 4)
	  {
		  if(Car_Trackflag){
				Car_Trackflag=0;
			   Car_Track_Line(&myCar); // 专门写一个循迹函数
		  }
       
     }
	  
	  if(sr04_Readflag)
     {
			 sr04_Readflag = 0;
			 if(myCar.Mode != 4){
				Car_AutoRun(&myCar);
			 }
	  }
	  
	  if(speed_countflag)
	  {
			speed_countflag=0;
		 
			 Car_Getspd(pulse_count_L,pulse_count_R);
			 // 3. PID 计算 (现在正负号对齐了，PID 就能正常工作了)
			 PID_L = PID_Compute(&Motor_PID_L, speed_L,Car_speed);
			 PID_R = PID_Compute(&Motor_PID_R, speed_R,Car_speed);
			 Car_Wheelspd(Car_dir,&myCar,PID_L,PID_R);
			 // 4. 执行输出
			 //Car_Wheelspd(&myCar,PWM_L, PWM_R);
        sprintf(send_buf, "[Spd,%.1f,%.1f]", speed_L,speed_R);
		  
	  }
	  
		if(keyscanflag==1){
			keyscanflag=0;
			Keyscan();
		}
		
		// === 电压测量与发送 (没指令时每5秒发一次) ===
	 if(btrysendflag){
		 
			btrysendflag=0;
			Car_Report_Status();
	 }
	 
	 if(Speed_Update_Flag){
		 Speed_Update_Flag=0;
		 //Motor_PID_R.Target=Car_speed;
	    //pid =PID_Compute(&Motor_PID_R,35);
		 Car_Speed_Handle(&myCar);
	 }
	 
	 if(SR04_send_flag){
		 SR04_send_flag=0;
		 HAL_UART_Transmit(&huart, (uint8_t*)send_buf, strlen(send_buf), 100);
		 sprintf(send_buf, "[Sonic,%.2f]",g_Distance); 
       HAL_UART_Transmit(&huart, (uint8_t*)send_buf, strlen(send_buf), 100);
		 
	 }
		
	//蓝牙接收处理
	// 检查是否有新数据
    if(Blu_rx_ready == 1)
    {
        Blu_rx_ready = 0; // 清除标志位
			
		  HAL_UART_Transmit(&huart1, (uint8_t*)&Blu_rx_buffer, Blu_rx_len,100);
			// 永远只找最后一个 '['，忽略前面积压的旧数据
			char *p = strrchr((char*)Blu_rx_buffer, '[');
		 // 1. 定义临时变量 (int 类型)
			int temp_speed = 0;
			int temp_mode = 0;
		 //if(strstr((char*)Blu_rx_buffer,"[Car")!=NULL)
		 //解析接收的指令
		 if (p != NULL)
		 {
				if (sscanf(p, "[Car,%c,%d]",&Car_dir,&temp_speed) == 2)
				{
					
					stopcartime=0;
					stopcarflag=0;
					if(myCar.Mode != 3 && myCar.Mode != 4){
						// 3. 安全赋值 (把 int 转回 uint16_t)
						Car_speed = (uint16_t)temp_speed;;
						
						//Car_Run(&myCar,Car_dir,Car_speed);
						
					}
					//printf("Car_dir:%c,speed:%d",Car_dir,Car_speed);
					
				}
					
				//获取小程序控制模式
				if (sscanf(p, "[Mode,%d]", &temp_mode) == 1)
				{	
						stopcartime=0;
					   Set_Servo_Angle(&htim1, TIM_CHANNEL_1, 90);
					   myCar.Mode = (uint8_t)temp_mode;
						Car_Set_Motion(&myCar);
				}
				
				
            // 接收小程序指令，立马发送一次电量
				if (strstr(p, "[Connect]") != NULL)
				{
						Car_Report_Status(); 
				}
		  
				memset(Blu_rx_buffer, 0, 255);
		}
		
		Blu_rx_len = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)Blu_rx_buffer, RX_BLU_SIZE); /// 接收完毕后重启串口DMA模式接收数据
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
	
	 //看门狗,超时强制停车,防止小车失控
	 if(stopcarflag)
	 {
			stopcarflag=0;		
		 
			if(myCar.Target_PWM_L != 0 || myCar.Target_PWM_R != 0) 
			{
				myCar.Target_Speed = 0; // 更新状态变量
				myCar.Current_PWM_L = 0;
				myCar.Current_PWM_R = 0;
				
				Car_Force_Stop(&myCar);
				//HAL_UART_Transmit(&huart1, (uint8_t*)"[Info,Stop]\r\n", 13, 100);
				//HAL_UART_Transmit(&huart1, (uint8_t*)"stop\r\n", 6, 100);
			}
			
	 }
	 /*//红外
	
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) 
    {
       HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_RESET);
    }
    else
    {     
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_SET);
    }
	  
	 */
	
	if(weathergetflag==1){
		
			lastweaflag=weathergetflag;
			weathergetflag=0;
			
	
	}

	if(sendflag){
		
		Onenet_connect();
		sendflag=0;
	}
	
	  /* //红外adc
	uint16_t adc_value;
    HAL_ADC_PollForConversion(&hadc1, 50);
    if ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
    {
        adc_value = HAL_ADC_GetValue(&hadc1);
        sprintf(adc,"V: %d", adc_value);
		oledshow(adc);
    }
	if(adc_value<1000){
		Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 45);  // 45掳
	}else Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 90);  // 45掳
	HAL_Delay(500);  
	 */
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 100);
	return ch;
}

void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle) 
{
    uint16_t pulse_width = 500 + (angle/ 180.0) * 2000; 
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse_width);  
    HAL_Delay(15);  
}

uint8_t MQTTRec(char * data)
{
	 if (strstr(rxbuff, "+MQTTSUBRECV:") != NULL && strstr(rxbuff, data) != NULL) 
	 {
		return 1;
	 }else return 0;
}

void oledshow(char *text,uint8_t x,uint8_t y)
{
	
	SSD1306_GotoXY(x, y);
	SSD1306_Puts(text, &Font_11x18, 1);
	SSD1306_UpdateScreen();
}



void Onenet_connect(void)
{
		if(lastweaflag==1){
			HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CIPCLOSE\r\n", 13, 100);
			 HAL_Delay(500);
			snprintf(onenetsend,sizeof(onenetsend),"AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",ONENET_DEVICE,ONENET_PID,ONENET_TOKEN);
			send_at_command(onenetsend);
			 //send_at_command("AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",ONENET_DEVICE,ONENET_PID,ONENET_TOKEN);
				 HAL_Delay(500);
				send_at_command("AT+MQTTCONN=0,\"mqtts.heclouds.com\",1883,1\r\n");
				 // 3. 连接 OneNET MQTT 服务器
				
				 HAL_Delay(1000); // 连接需要时间，必须延时！
			snprintf(onenetsend,sizeof(onenetsend),"AT+MQTTSUB=0,\"$sys/%s/%s/thing/property/post/reply\",0\r\n",ONENET_PID,ONENET_DEVICE);
			send_at_command(onenetsend);
					//send_at_command("AT+MQTTSUB=0,\"$sys/NW4ydVqKGU/pro1/thing/property/post/reply\",0\r\n");

				 // 4. 构造物模型 JSON 数据
				 // 假设 OneNET 平台上有 identifier 为 "temperature" 和 "humidity" 的属性
				 
				  HAL_Delay(1000);
			snprintf(onenetsend,sizeof(onenetsend),"AT+MQTTSUB=0,\"$sys/%s/%s/thing/property/set\",0\r\n",ONENET_PID,ONENET_DEVICE);
			send_at_command(onenetsend);
				//send_at_command("AT+MQTTSUB=0,\"$sys/NW4ydVqKGU/pro1/thing/property/set\",0\r\n");
			HAL_Delay(1000);
			lastweaflag=0;
		}
		
		
		Get_Temp();
		snprintf(onenetsend,sizeof(onenetsend),"AT+MQTTPUB=0,\"%s\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"hum\\\":{\\\"value\\\":%d}\\,\\\"temp\\\":{\\\"value\\\":%d}}}\",0,0\r\n",TOPIC_POST,humidity,temperature);
   //snprintf(onenetsend,sizeof(onenetsend),"AT+MQTTPUB=0,\"%s\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"hum\\\":{\\\"value\\\":%d}\\,\\\"temp\\\":{\\\"value\\\":%d}\\,\\\"weather\\\":{\\\"value\\\":\\\"%s\\\"}}}\",0,0\r\n",TOPIC_POST,humidity,temperature,weather_str);
				send_at_command(onenetsend);	
		if(weathergetflag){
			 snprintf(onenetsend,sizeof(onenetsend),"AT+MQTTPUB=0,\"$sys/NW4ydVqKGU/pro1/thing/property/post\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"weather\\\":{\\\"value\\\":\\\"%s\\\"}}}\",0,0\r\n",weather_str);
			send_at_command(onenetsend);	
		}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
   if (huart->Instance == USART1)
   {	
		// 1. 保存接收到的数据长度 (关键修正！)
      Blu_rx_len = Size;
   	
   	Blu_rx_ready =1;
   	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);		   		// 手动关闭DMA_IT_HT中断
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
		static uint32_t weathersendcount = 0;
		static uint32_t dht11sendcount = 0;
		static uint32_t btrysendtime = 0;
		static uint32_t sr04_Readtime=0;
		static uint32_t sr04_sendtime=0;
		static uint32_t speed_counttime=0;
		
		if(myCar.Mode == 4){
			Car_Trackflag=1;
		}
		Speed_Update_Flag=1;
		
		if(++speed_counttime >= 5) 
		{  
			speed_countflag=1;
		   speed_counttime = 0;
		  
		}	
		
		if(++keycount >= 1) 
		{  // 每10ms扫描一次按键
		  keycount = 0;
		  keyscanflag=1;
		}	
		
		if(myCar.Target_Speed==0){
			if(++btrysendtime>500){
				btrysendtime=0;
				btrysendflag=1;
			}
		}else btrysendtime=0;
		
		if(myCar.Mode!=3 &&myCar.Mode!=4){
				
				if(++stopcartime > 30)
				{					
						stopcartime=31;
						stopcarflag=1;					
				}	
		}else 
		{
			 stopcartime = 0; // 如果是模式3或4，一直喂狗，保持活跃
		}
		
		if(++sr04_Readtime>=10){
			sr04_Readflag=1;
			sr04_Readtime=0;
		}
		
		if(++sr04_sendtime>=100){
			sr04_sendtime=0;
			SR04_send_flag=1;
			
		}
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
    if(GPIO_Pin == GPIO_PIN_6) // 假设你接的是 PA0
    {
        pulse_count_L++; 
    }
	 if(GPIO_Pin == GPIO_PIN_4) // 假设你接的是 PA0
    {
        pulse_count_R++; 
    }
}

void send_at_command(const char* cmd) 
{
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 100);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100); 
}
void Get_Temp(void)
{
	DHT11_Data_TypeDef DHT11_Data;
	if(DHT11_Read_TempAndHumidity(&DHT11_Data)==SUCCESS)
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

#ifdef  USE_FULL_ASSERT
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
