//
// Created by wxj on 26-4-9.
//

#include "tim.h"
#include "servo.h"

/**
 * @brief 初始化舵机控制定时器
 * @note 启动 TIM1 通道 1 的 PWM 输出
 */
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

/**
 * @brief 设置电机转动角度
 * @param angle:期望的角度值
 * @note 该函数通过线下插值将角度转换为定时器的比较值
 */
void Servo_SetAngle(float angle)
{
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;
    uint16_t ccr = SERVO_MIN +
        (angle / 180.0f) * (SERVO_MAX - SERVO_MIN);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}