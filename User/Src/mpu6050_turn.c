#include "mpu6050_turn.h"
#include <math.h>

// Z轴角速度的零点偏移量（用于消除静态漂移）
static float gyro_z_offset = 0.0f;

/**
 * @brief MPU6050 初始化
 * @return 0:成功, 1:失败
 */
uint8_t MPU6050_Init(void) {
    uint8_t check, data;
    
    // 检查 MPU6050 是否响应 (读取 WHO_AM_I 寄存器 0x75)
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
    if (check != 0x68) {
        return 1; // 找不到设备
    }

    // 唤醒 MPU6050 (解除休眠状态)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_PWR_MGMT1_REG, 1, &data, 1, 1000);
    
    // 设置陀螺仪量程为 ±2000°/s (适合小车快速转弯)
    data = 0x18; 
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_GYRO_CFG_REG, 1, &data, 1, 1000);
    
    // 采样率分频 (1kHz / (1 + 1) = 500Hz)
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU_SMPLRT_DIV_REG, 1, &data, 1, 1000);
    // 开启数字低通滤波器 (DLPF)，配置为约 42Hz 带宽，有助于滤除电机振动噪声
    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &data, 1, 1000);
    
    // 静态校准Z轴 (注意：调用此函数时，小车必须放在地上保持绝对静止)
    MPU6050_Calibrate_Gyro_Z();
    
    return 0;
}

/**
 * @brief 读取 MPU6050 Z轴原始角速度数据
 */
int16_t MPU6050_Get_Gyro_Z_Raw(void) {
    uint8_t buf[2];
    int16_t raw_z;
    
    // 连续读取 Z轴的高八位和低八位
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU_GYRO_ZOUT_H, 1, buf, 2, 100);
    raw_z = (buf[0] << 8) | buf[1];
    
    return raw_z;
}

/**
 * @brief 计算 Z 轴零点漂移误差
 */
void MPU6050_Calibrate_Gyro_Z(void) {
    int32_t sum = 0;
    int sample_count = 200;
    
    for(int i = 0; i < sample_count; i++) {
        sum += MPU6050_Get_Gyro_Z_Raw();
        HAL_Delay(2); // 短暂延时保证采样稳定
    }
    gyro_z_offset = (float)sum / sample_count;
}

/**
 * @brief 非阻塞式判断小车是否转到了指定角度
 * @param x 目标角度，范围 -360 到 360 (正数代表一个方向，负数代表反方向)
 * @return true: 已达到目标角度 (同时重置状态) / false: 正在转弯中，未达到
 */
bool MPU6050_Check_Turn_Reached(int x) {
    // 使用静态变量，保持函数调用之间的状态
    static bool is_tracking = false;   // 是否正在跟踪计算角度
    static float current_angle = 0.0f; // 当前累计角度
    static uint32_t last_time = 0;     // 上一次调用的时间戳
    static float last_gyro_z_dps = 0.0f; // 需定义为静态变量

    // 如果 x 传了 0，直接认为已到达，避免逻辑卡死
    if (x == 0) {
        return true;
    }

    // 1. 第一次调用：初始化状态，记录“起始位姿”
    if (!is_tracking) {
        is_tracking = true;
        current_angle = 0.0f;           // 起始角度清零
        last_time = HAL_GetTick();      // 记录当前时间
        return false;                   // 刚开始转，肯定还没到，返回 false
    }

    // 2. 后续调用：进行时间积分计算
    uint32_t current_time = HAL_GetTick();
    float dt = (float)(current_time - last_time) / 1000.0f; // 计算时间差(秒)
    last_time = current_time;

    // 只有当时间有推进时才进行计算（防止被极快地连续调用导致 dt 为 0）
    if (dt > 0.0f) {
        int16_t raw_z = MPU6050_Get_Gyro_Z_Raw();
        // 减去零点漂移误差，转换为真实的角速度 (度/秒)
        float gyro_z_dps = ((float)raw_z - gyro_z_offset) / GYRO_LSB_2000;
        // 增加死区设置
        if (fabs(gyro_z_dps) < 0.5f) {
            gyro_z_dps = 0.0f;
        }

        
        // 积分累加：当前角度 = 之前的角度 + (角速度 * 时间差)
        //current_angle += (gyro_z_dps * dt);

        current_angle += (last_gyro_z_dps + gyro_z_dps) * dt / 2.0f;
        last_gyro_z_dps = gyro_z_dps;

    }


    // 3. 判断是否到达目标角度
    bool reached = false;
    
    // 如果目标是正数，判断当前角度是否 >= 目标值
    if (x > 0 && current_angle >= (float)x) {
        reached = true;
    } 
    // 如果目标是负数，判断当前角度是否 <= 目标值
    else if (x < 0 && current_angle <= (float)x) {
        reached = true;
    }

    // 4. 结算：如果达到了，重置状态并输出 true
    if (reached) {
        is_tracking = false; // 重置标志位，使得下一次调用成为新的“第一次调用”
        return true;
    }

    // 还没达到，继续保持跟踪状态
    return false;
}