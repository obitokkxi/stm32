#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"
#include <math.h>

// MPU6050 地址
#define MPU6050_ADDR         0xD0

// 常用寄存器
#define MPU_PWR_MGMT_1       0x6B
#define MPU_SMPLRT_DIV       0x19
#define MPU_CONFIG           0x1A
#define MPU_GYRO_CONFIG      0x1B
#define MPU_ACCEL_CONFIG     0x1C
#define MPU_ACCEL_XOUT_H     0x3B

// 参数
#define DT                   0.01f  // 10ms
#define RAD_TO_DEG           57.2957795f

// --- 适配你的工程的结构体定义 ---
typedef struct {
    int16_t Acc_X_Raw;
    int16_t Acc_Y_Raw;
    int16_t Acc_Z_Raw;
    int16_t Gyro_X_Raw;
    int16_t Gyro_Y_Raw;
    int16_t Gyro_Z_Raw;

    float Ax; // 加速度计转换后的值
    float Ay;
    float Az;
    
    float Gx; // 陀螺仪转换后的值
    float Gy;
    float Gz; // Blucar.c 需要用到这个做转向

	// --- 新增：Z轴零点偏移量 ---
    float Gz_offset;
    // --- 核心角度变量 (匹配你的报错信息) ---
    float KalmanAngleX; // Roll
    float KalmanAngleY; // Pitch (平衡车主要靠这个)
    float KalmanAngleZ; // Yaw   (Blucar.c 用这个做锁头/转向)

    // --- 卡尔曼中间变量 (隐藏在结构体里) ---
    struct {
        float Q_angle;
        float Q_gyro;
        float R_angle;
        float C_0;
        float Q_bias;
        float Angle_err;
        float PCt_0;
        float PCt_1;
        float E;
        float K_0;
        float K_1;
        float t_0;
        float t_1;
        float Pdot[4];
        float PP[2][2];
    } KalmanY; // 专门给 Y 轴用的卡尔曼参数

} MPU6050_t;

extern MPU6050_t mpu; // 你的 main.c 里可能叫 mpu 而不是 MPU6050

// --- 函数声明 (匹配你的 main.c 调用) ---
void MPU6050_Init(I2C_HandleTypeDef *hi2c);       // 修正：带参数
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct); // 修正：匹配 main.c
void MPU6050_Calibrate_Z(I2C_HandleTypeDef *hi2c); // 新增：main.c 里调用了这个

#endif