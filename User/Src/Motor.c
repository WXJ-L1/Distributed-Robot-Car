//
// Created by 10663 on 26-5-1.
//

#include "Motor.h"
#include "tim.h"
#include "gpio.h"

void TB6612_Init(void)
{
    HAL_StatusTypeDef ret1;
    HAL_StatusTypeDef ret2;

    ret1 = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   // PA0 -> PWMA
    ret2 = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   // PA1 -> PWMB

    if (ret1 != HAL_OK || ret2 != HAL_OK)
    {
        Error_Handler();
    }
    // 初始 PWM 清零，防止上电乱转
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    TB6612_Enable();
    // 默认停止
    Motor_Stop();
}

void TB6612_Enable(void)
{
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
}

void TB6612_Standby(void)
{
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void Motor_A_SetSpeed(int speed)
{
    if (speed > PWM_MAX)
    {
        speed = PWM_MAX;
    }
    else if (speed < -PWM_MAX)
    {
        speed = -PWM_MAX;
    }

    if (speed > 0)
    {
        // A 电机正转
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)speed);
    }
    else if (speed < 0)
    {
        // A 电机反转
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(-speed));
    }
    else
    {
        // A 电机滑行停止
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    }
}

void Motor_B_SetSpeed(int speed)
{
    if (speed > PWM_MAX)
    {
        speed = PWM_MAX;
    }
    else if (speed < -PWM_MAX)
    {
        speed = -PWM_MAX;
    }

    if (speed > 0)
    {
        // B 电机正转
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)speed);
    }
    else if (speed < 0)
    {
        // B 电机反转
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)(-speed));
    }
    else
    {
        // B 电机滑行停止
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
}

void Motor_Stop(void)
{
    Motor_A_SetSpeed(0);
    Motor_B_SetSpeed(0);
}

void Motor_Brake(void)
{
    // TB6612 短刹车：IN1 = 1, IN2 = 1
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);

    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}

void Car_Forward(int speed)
{
    Motor_A_SetSpeed(speed);
    Motor_B_SetSpeed(speed);
}

void Car_Backward(int speed)
{
    Motor_A_SetSpeed(-speed);
    Motor_B_SetSpeed(-speed);
}

void Car_Left(int speed)
{
    Motor_A_SetSpeed(-speed);
    Motor_B_SetSpeed(speed);
}

void Car_Right(int speed)
{
    Motor_A_SetSpeed(speed);
    Motor_B_SetSpeed(-speed);
}