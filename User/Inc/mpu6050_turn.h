#ifndef __MPU6050_TURN_H
#define __MPU6050_TURN_H

#include "i2c.h"      // 引入你的 I2C 句柄 (包含 hi2c1)
#include <stdbool.h>  // 引入 bool 类型支持

// MPU6050 I2C地址 (AD0引脚悬空或接地为0x68，HAL库需要左移一位变为0xD0)
#define MPU6050_ADDR        0xD0

// MPU6050 关键寄存器地址
#define MPU_PWR_MGMT1_REG   0x6B
#define MPU_SMPLRT_DIV_REG  0x19
#define MPU_CONFIG_REG      0x1A
#define MPU_GYRO_CFG_REG    0x1B
#define MPU_GYRO_ZOUT_H     0x47
#define MPU_GYRO_ZOUT_L     0x48

// 角速度灵敏度 (当量程设为 ±2000°/s 时，16.4 LSB/(°/s))
#define GYRO_LSB_2000       16.4f

// 函数声明
uint8_t MPU6050_Init(void);
int16_t MPU6050_Get_Gyro_Z_Raw(void);
void MPU6050_Calibrate_Gyro_Z(void);

// 核心非阻塞判断函数
bool MPU6050_Check_Turn_Reached(int x);

#endif /* __MPU6050_TURN_H */