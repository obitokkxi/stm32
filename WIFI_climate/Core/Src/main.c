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
#include "i2c.h"
#include "spi.h"
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
#include "max7219.h"
#include "key.h"
#include "MPU6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 512
#define free_state 0
#define singlepress 1
#define doublepress 2
#define longpress 3
#define WEATHER_KEY "SkauKTjXsbUijiTkw" 
#define WEATHER_CITY "beijing"
// OneNET 配置信息
#define ONENET_PID      "NW4ydVqKGU"       // 产品 ID
#define ONENET_DEVICE   "pro1"             // 设备名称
// 【注意】这里必须填计算出来的 Token (res=products/PID/devices/pro1)
#define ONENET_TOKEN    "version=2018-10-31&res=products%2FNW4ydVqKGU%2Fdevices%2Fpro1&et=..." 

// OneNET 物模型上报 Topic (固定格式)
#define TOPIC_POST      "$sys/" ONENET_PID "/" ONENET_DEVICE "/thing/property/post"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char http_buffer[512]; // 存放 HTTP 请求报文
char len_cmd[64];      // 存放长度指令
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
volatile uint8_t rx_index = 0;
volatile uint8_t new_data_available = 0;
char oled[50];
uint8_t temperature = 0;
uint8_t humidity = 0;
uint16_t speed=0;
uint8_t doubleflag;
uint8_t singleflag;
uint8_t longflag;
uint8_t keyscanflag;
uint8_t wificheckflag;
long pwmvalue;


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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  /*
    send_at_command("AT+RST");
	HAL_Delay (1000);
	*/
	
	SSD1306_Init();
	SSD1306_Clear();
	oledshow("ok",0,20);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // servo IN1
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // servo IN2
	/* //mqtt init
	HAL_Delay (1000);
	send_at_command("AT+CWJAP=\"Xiaomi_D44B\",\"2271902787Ab\"");
	HAL_Delay (4000);
    send_at_command("AT+MQTTUSERCFG=0,1,\"mqttx_1631101c\",\"123\",\"123\",0,0,\"\"\r\n");
	HAL_Delay (2000);
	send_at_command("AT+MQTTCONN=0,\"broker.emqx.io\",1883,1\r\n");
	HAL_Delay (2000);
	send_at_command("AT+MQTTSUB=0,\"command\",0\r\n");
	HAL_Delay (1000);
	*/
	
	//onenet init
	HAL_Delay (1000);
	send_at_command("AT+CWJAP=\"Neo8\",\"2271902787\"");
	//send_at_command("AT+CWJAP=\"Xiaomi_D44B\",\"2271902787Ab\"");
	//HAL_Delay (4000);
//send_at_command("AT+CIPSTART=\"TCP\",\"api.seniverse.com\",80");
    HAL_Delay(2000);
	
	
	DHT11_Init();
	//陀螺仪初始化
	//MPU6050_init(&hi2c2);
	//点阵初始化
	//MAX7219_Init();

/*点阵

uint8_t smiley_pattern[] = {
    0x00,  
    0x66,  
    0x99,  
    0x18,  
    0x18,  
    0x81,  
    0x7E,  
    0x00   
};

	MAX7219_DisplayPattern(smiley_pattern);
	*/
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	//HAL_TIM_Base_Start(&htim4);  	 
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxbuff[0], 1);
	
	HAL_TIM_Base_Start_IT(&htim4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0); 
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
		if(keyscanflag==1){
			keyscanflag=0;
			Keyscan();
		}
		if(singleflag==1){
			singleflag=0;
			//HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_RESET);
		}else if(doubleflag==1){
			doubleflag=0;
			//HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_SET);
		}
	  //onenet mess handle
	
	
	
	
	/*//红外
	if (HAL_GPIO_ReadPin(red_ray_GPIO_Port, red_ray_Pin) == GPIO_PIN_RESET) 
    {
       HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_RESET);
    }
    else
    {     
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_SET);
    }
	  */
	if(rxflag==1)
    {
        // 1. 【核心修复】立刻备份数据！防止被中断里的新数据覆盖
        // 这一步极快，比 printf 快得多
        strcpy(process_buff, rxbuff); 
        
        // 2. 立刻清除标志位，允许中断继续接收新数据，互不干扰
        rxflag = 0; 
        
        // 3. 接下来所有的操作都针对 process_buff 进行
        char *p;
        char temp_str[10] = {0};
        char weather_str[20] = {0};

        // --- 提取温度 ---
        // 注意：这里改成 process_buff
        p = strstr(process_buff, "\"temperature\":\"");
        if (p != NULL)
        {
            p += 15; 
            int i = 0;
            while (*p != '\"' && i < 5)
            {
                temp_str[i++] = *p++;
            }
            // 确保字符串结束符
            temp_str[i] = '\0'; 
            //printf("解析温度: %s\r\n", temp_str);
        }

        // --- 提取天气现象 ---
        // 注意：这里也改成 process_buff
        p = strstr(process_buff, "\"text\""); 
        
        if (p != NULL)
        {
            p += 6; 
            while (*p != ':' && *p != '\0') p++;
            while (*p != '\"' && *p != '\0') p++;
            
            if (*p == '\"')
            {
                p++; 
                int i = 0;
                while (*p != '\"' && *p != '\0' && i < 19)
                {
                    weather_str[i++] = *p++;
                }
                weather_str[i] = '\0'; // 加上结束符
                
                //printf("解析天气: %s\r\n", weather_str);
                
                // 如果需要显示到 OLED
                // oledshow(weather_str); 
            }
				sprintf(message,"temp:%s",temp_str);
				oledshow(message,0,0);
				sprintf(message,"wth:%s",weather_str);
				oledshow(message,0,20);
        }
        else 
        {
             // 如果还是找不到，打印出来看看现在的 buffer 到底是个啥
             // printf("未找到天气, Buffer内容: %s\r\n", process_buff);
        }
    }
	
	if(weathergetflag==1){
		sendflag=0;
		HAL_UART_Transmit(&huart1, (uint8_t*)"AT+MQTTCLEAN=0\r\n", 16, 100);
		HAL_Delay(1000);
		send_at_command("AT+CIPSTART=\"TCP\",\"api.seniverse.com\",80");
		HAL_Delay(500);
	// 1. 先连接 (必须确认连接成功)
    // 这一步建议加上返回值判断，如果没有 CONNECT，后面发啥都没用
		//HAL_Delay(2000);
    HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CIPSTART=\"TCP\",\"api.seniverse.com\",80\r\n", 42, 100);
    HAL_Delay(1000); // 给足时间连接
		
		

   // 既然手动发是 134，我们要保证这里算出来也是 134 (或者由代码自动算准)
    int total_len = strlen(http_buffer) + strlen(line2) + strlen(line3) + strlen(line4);

    // --- 3. 发送长度指令 ---
    // 你的手动操作: AT+CIPSEND=134
    sprintf(http_buffer, "AT+CIPSEND=%d\r\n", total_len);
    HAL_UART_Transmit(&huart1, (uint8_t*)http_buffer, strlen(http_buffer), 100);

		memset(http_buffer, 0, sizeof(http_buffer));
		 sprintf(http_buffer, 
                "GET /v3/weather/now.json?key=%s&location=%s&language=en HTTP/1.1\r\n", 
                WEATHER_KEY, 
                WEATHER_CITY);
    // --- 4. 等待 ">" 符号 ---
    // 你的手动操作: 等待 > 出现
    HAL_Delay(500); 

    // --- 5. 分段发送数据 (模仿你的手动点击) ---
    
    // 第1行: GET ... (点发送)
    HAL_UART_Transmit(&huart1, (uint8_t*)http_buffer, strlen(http_buffer), 1000);
    HAL_Delay(50); // 稍微停顿一下，模拟人的手速（可选，但更稳）

    // 第2行: Host: ... (点发送)
    HAL_UART_Transmit(&huart1, (uint8_t*)line2, strlen(line2), 1000);
    HAL_Delay(50);

    // 第3行: Connection: close (点发送)
    HAL_UART_Transmit(&huart1, (uint8_t*)line3, strlen(line3), 1000);
    HAL_Delay(50);

    // 第4行: 空行 (点发送)
    HAL_UART_Transmit(&huart1, (uint8_t*)line4, strlen(line4), 1000);
	
	//HAL_Delay(2000);
		weathergetflag=0;	
	
	}

	if(sendflag){
		if(!weathergetflag){
			HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CIPCLOSE\r\n", 13, 100);
			 HAL_Delay(500);
		}
		
		
		
		Get_Temp();

		sendflag=0;
		char cmd_buff[256];
    char json_buff[256];
		// 1. 强制断开之前的连接 (无论是 HTTP 还是 MQTT，先断为敬)
    // 防止 ESP8266 报 "ALREADY CONNECTED" 错误
    
   
		
		// 格式: AT+MQTTUSERCFG=0,1,"设备名","产品ID","Token",0,0,""
    memset(cmd_buff, 0, sizeof(cmd_buff));
    send_at_command("AT+MQTTUSERCFG=0,1,\"pro1\",\"NW4ydVqKGU\",\"version=2018-10-31&res=products%2FNW4ydVqKGU%2Fdevices%2Fpro1&et=2541412770&method=md5&sign=2vCMFmP%2FtvBfhgw8YwXQDg%3D%3D\",0,0,\"\"\r\n");
    HAL_Delay(500);
	send_at_command("AT+MQTTCONN=0,\"mqtts.heclouds.com\",1883,1\r\n");
    // 3. 连接 OneNET MQTT 服务器
   
    HAL_Delay(1000); // 连接需要时间，必须延时！
		send_at_command("AT+MQTTSUB=0,\"$sys/NW4ydVqKGU/pro1/thing/property/post/reply\",0\r\n");

    // 4. 构造物模型 JSON 数据
    // 假设 OneNET 平台上有 identifier 为 "temperature" 和 "humidity" 的属性
    memset(json_buff, 0, sizeof(json_buff));
	  HAL_Delay(1000);
   send_at_command("AT+MQTTSUB=0,\"$sys/NW4ydVqKGU/pro1/thing/property/set\",0\r\n");
HAL_Delay(1000);
    // 5. 发布数据
    // 格式: AT+MQTTPUB=0,"Topic","Payload",0,0
    memset(cmd_buff, 0, sizeof(cmd_buff));
    snprintf(onenetsend,sizeof(onenetsend),"AT+MQTTPUB=0,\"$sys/NW4ydVqKGU/pro1/thing/property/post\",\"{\\\"id\\\":\\\"123\\\"\\,\\\"params\\\":{\\\"hum\\\":{\\\"value\\\":%d}\\,\\\"temp\\\":{\\\"value\\\":%d}}}\",0,0\r\n",humidity,temperature);
				send_at_command(onenetsend);	
				HAL_Delay(1000);
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

void MQTThandle(void)
{
	
	if(MQTTRec(",on")){
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_RESET);
           
	}else if(MQTTRec(",off")){
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,GPIO_PIN_SET);
			
	}else if(MQTTRec(",0")){
		
		Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 0);  // 0
			
	}else if(MQTTRec(",45")){
		speed-=360;
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,speed); 
		Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 45);  // 45
			
	}else if(MQTTRec(",90")){
		Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 90);  // 90
		
	}else if(MQTTRec(",180")){
		Set_Servo_Angle(&htim2, TIM_CHANNEL_1, 180);  // 180
			
	}else if(MQTTRec(",i")){
		speed+=360;		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,speed); 
		sprintf(oled,"speed:%d",speed);
		
	}else if(MQTTRec(",d")){
		speed=speed-360;		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,speed); 
		sprintf(oled,"speed:%d",speed);
		
	}
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) 
	{
        
        static char received_char;
        static char temp_buff[512];
        static uint16_t temp_index = 0; 
        received_char = rxbuff[0]; 
        if (received_char == '\n') 
		{ 
            if (temp_index > 0)
			{
                temp_buff[temp_index] = '\0';  
                strcpy(rxbuff, temp_buff);  
				
                rxflag = 1;        
                memset(temp_buff, 0, sizeof(temp_buff));
                temp_index = 0;
            }
        }
        else if (received_char != '\r') 
		{ 
            if (temp_index < sizeof(temp_buff) - 1) 
			{
                temp_buff[temp_index++] = received_char;
            }
            else 
			{
                memset(temp_buff, 0, sizeof(temp_buff));
                temp_index = 0;
            }
        }

        HAL_UART_Receive_IT(huart, (uint8_t*)&rxbuff[0], 1);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM4) 
	{
		static uint32_t keycount = 0;
		static uint32_t weathersendcount = 0;
		static uint32_t dht11sendcount = 0;
		if(++keycount >= 1) 
		{  // 每10ms扫描一次按键
		  keycount = 0;
		  keyscanflag=1;
		}	
		// 5秒定时
		if(++weathersendcount >= 6000) 
		{
		  weathersendcount = 0;
		  weathergetflag = 1;  // 触发数据发送
		}
		if(++dht11sendcount >= 500){
			dht11sendcount=0;
			sendflag=1;
		}
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
	else
	{
	
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
