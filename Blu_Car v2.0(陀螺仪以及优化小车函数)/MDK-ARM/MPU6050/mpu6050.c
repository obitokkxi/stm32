#include "mpu6050.h"

// 定义全局变量
MPU6050_t mpu; 

// 内部函数声明
static void MPU_Write_Byte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data);
static void Kalman_Filter_Y(float Angle_Rec, float Gyro_Rec);

// --- 接口函数 ---

// 1. 初始化函数 (适配 main.c 的调用: MPU6050_Init(&hi2c1))
void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    MPU_Write_Byte(hi2c, MPU_PWR_MGMT_1, 0x00);
    HAL_Delay(50);
    MPU_Write_Byte(hi2c, MPU_GYRO_CONFIG, 0x18);
    MPU_Write_Byte(hi2c, MPU_ACCEL_CONFIG, 0x00);
    MPU_Write_Byte(hi2c, MPU_SMPLRT_DIV, 0x00);
    MPU_Write_Byte(hi2c, MPU_CONFIG, 0x04);

    mpu.KalmanY.Q_angle = 0.001f;
    mpu.KalmanY.Q_gyro  = 0.003f;
    mpu.KalmanY.R_angle = 0.5f;
    mpu.KalmanY.C_0     = 1;
    mpu.KalmanY.PP[0][0] = 1;
    mpu.KalmanY.PP[1][1] = 1;
    
    mpu.Gz_offset = 0; // 初始化为0
}

// 2. 读取并计算函数 (适配 main.c 的调用: MPU6050_Read_All(&hi2c1, &mpu))
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct) {
    uint8_t buffer[14];
    
    if(HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 14, 10) == HAL_OK) {
        
        DataStruct->Acc_X_Raw  = (int16_t)(buffer[0] << 8 | buffer[1]);
        DataStruct->Acc_Y_Raw  = (int16_t)(buffer[2] << 8 | buffer[3]);
        DataStruct->Acc_Z_Raw  = (int16_t)(buffer[4] << 8 | buffer[5]);
        DataStruct->Gyro_X_Raw = (int16_t)(buffer[8] << 8 | buffer[9]);
        DataStruct->Gyro_Y_Raw = (int16_t)(buffer[10] << 8 | buffer[11]);
        DataStruct->Gyro_Z_Raw = (int16_t)(buffer[12] << 8 | buffer[13]);

        DataStruct->Gx = DataStruct->Gyro_X_Raw / 16.4f;
        DataStruct->Gy = DataStruct->Gyro_Y_Raw / 16.4f;
        
        // --- 核心修改：减去校准后的零偏 ---
        DataStruct->Gz = (DataStruct->Gyro_Z_Raw / 16.4f) - DataStruct->Gz_offset;

        float Accel_Angle_Y = atan2(DataStruct->Acc_X_Raw, DataStruct->Acc_Z_Raw) * RAD_TO_DEG;

        Kalman_Filter_Y(Accel_Angle_Y, DataStruct->Gy);
        
        // Z轴积分 (现在减去了零偏，漂移会大大减小)
        DataStruct->KalmanAngleZ += DataStruct->Gz * DT; 
        
        DataStruct->KalmanAngleX = atan2(DataStruct->Acc_Y_Raw, DataStruct->Acc_Z_Raw) * RAD_TO_DEG;
    }
}

// 3. Z轴校准 (main.c 里调用了，给个空函数或简单的实现防止报错)
void MPU6050_Calibrate_Z(I2C_HandleTypeDef *hi2c) {
    uint8_t buffer[2];
    int16_t temp_raw;
    float sum = 0;
    int i;
    
    // 循环采样 200 次
    for(i = 0; i < 200; i++) {
        HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x47, I2C_MEMADD_SIZE_8BIT, buffer, 2, 10); // 0x47是Gyro_Z_H
        temp_raw = (int16_t)(buffer[0] << 8 | buffer[1]);
        
        // 累加转换后的角速度 (度/秒)
        sum += (float)temp_raw / 16.4f;
        HAL_Delay(5); // 稍微延时，避免读到重复数据
    }
    
    // 计算平均值，这就是零点漂移
    mpu.Gz_offset = sum / 200.0f;
    
    // 校准完毕，将 Z 轴角度清零，准备开始工作
    mpu.KalmanAngleZ = 0; 
}

// --- 内部函数实现 ---

// 写寄存器工具函数
static void MPU_Write_Byte(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

// Y轴卡尔曼滤波器
static void Kalman_Filter_Y(float Angle_Rec, float Gyro_Rec)
{
    // 1. 状态预测
    // 使用 DataStruct->KalmanAngleY
    mpu.KalmanAngleY += (Gyro_Rec - mpu.KalmanY.Q_bias) * DT;

    // 2. 协方差预测
    mpu.KalmanY.Pdot[0] = mpu.KalmanY.Q_angle - mpu.KalmanY.PP[0][1] - mpu.KalmanY.PP[1][0];
    mpu.KalmanY.Pdot[1] = -mpu.KalmanY.PP[1][1];
    mpu.KalmanY.Pdot[2] = -mpu.KalmanY.PP[1][1];
    mpu.KalmanY.Pdot[3] = mpu.KalmanY.Q_gyro;

    mpu.KalmanY.PP[0][0] += mpu.KalmanY.Pdot[0] * DT;
    mpu.KalmanY.PP[0][1] += mpu.KalmanY.Pdot[1] * DT;
    mpu.KalmanY.PP[1][0] += mpu.KalmanY.Pdot[2] * DT;
    mpu.KalmanY.PP[1][1] += mpu.KalmanY.Pdot[3] * DT;

    // 3. 计算增益
    // 这里的 Angle_Rec 就是加速度计算出来的角度
    // 修正之前的 bug: 用 mpu.KalmanAngleY 而不是 Kalman.Pitch
    mpu.KalmanY.Angle_err = Angle_Rec - mpu.KalmanAngleY; 

    mpu.KalmanY.PCt_0 = mpu.KalmanY.C_0 * mpu.KalmanY.PP[0][0];
    mpu.KalmanY.PCt_1 = mpu.KalmanY.C_0 * mpu.KalmanY.PP[1][0];
    mpu.KalmanY.E = mpu.KalmanY.R_angle + mpu.KalmanY.C_0 * mpu.KalmanY.PCt_0;

    if (mpu.KalmanY.E != 0) {
        mpu.KalmanY.K_0 = mpu.KalmanY.PCt_0 / mpu.KalmanY.E;
        mpu.KalmanY.K_1 = mpu.KalmanY.PCt_1 / mpu.KalmanY.E;
    }

    // 4. 更新状态
    mpu.KalmanY.t_0 = mpu.KalmanY.PCt_0;
    mpu.KalmanY.t_1 = mpu.KalmanY.C_0 * mpu.KalmanY.PP[0][1];

    mpu.KalmanY.PP[0][0] -= mpu.KalmanY.K_0 * mpu.KalmanY.t_0;
    mpu.KalmanY.PP[0][1] -= mpu.KalmanY.K_0 * mpu.KalmanY.t_1;
    mpu.KalmanY.PP[1][0] -= mpu.KalmanY.K_1 * mpu.KalmanY.t_0;
    mpu.KalmanY.PP[1][1] -= mpu.KalmanY.K_1 * mpu.KalmanY.t_1;

    // 5. 修正最终结果
    mpu.KalmanAngleY     += mpu.KalmanY.K_0 * mpu.KalmanY.Angle_err;
    mpu.KalmanY.Q_bias   += mpu.KalmanY.K_1 * mpu.KalmanY.Angle_err;
}