//
// Created by wxj on 26-4-10.
// Modified: Basic Logic-based line tracking (No PD).
//

#include "linewalking.h"
#include "Motor.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#include "UART.h"

/*
 * 四路红外传感器逻辑：
 * GPIO_PIN_RESET = 0 = 检测到黑线 / 障碍
 * GPIO_PIN_SET   = 1 = 未检测到
 */
#define LINE_DETECTED     GPIO_PIN_RESET
#define LINE_NOT_DETECTED GPIO_PIN_SET

/*
 * 速度范围按你的 TB6612 PWM 设置：
 * TIM2 Period = 1000 - 1
 * 所以速度范围是 0 ~ 999
 */
#define LINE_SPEED_HIGH   800//650
#define LINE_SPEED_MID    600 //500
#define LINE_SPEED_LOW    400
#define LINE_SPEED_SEARCH 300

/**
  * @brief 读取四路循迹传感器的实时状态
  */
LineState_t Line_Read(void)
{
    LineState_t s;

    s.L1 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8);
    s.L2 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9);
    s.R1 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
    s.R2 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11);

    return s;
}

/**
  * @brief 打印四路循迹检测状态到串口1
  */
void Line_PrintState(LineState_t s)
{
    char buf[128];

    snprintf(buf, sizeof(buf),
             "Line State: L1=%d L2=%d R1=%d R2=%d\r\n",
             s.L1, s.L2, s.R1, s.R2);

    UART1_SendString(buf);
}

/**
  * @brief 四路基础循迹逻辑
  * @note  0 = 检测到黑线，1 = 未检测到黑线
  */
void LineWalking(void)
{
    LineState_t s = Line_Read();
    /*
 * 防止串口刷屏太快：每 300ms 打印一次状态
 */
    static uint32_t last_print_time = 0;

    // if (HAL_GetTick() - last_print_time >= 300)
    // {
    //     Line_PrintState(s);
    //     last_print_time = HAL_GetTick();
    // }






        // --- 1. 全白 (无线) ---
        if (s.L1 == 1 && s.L2 == 1 && s.R1 == 1 && s.R2 == 1) {
            // TODO:
        }

        // --- 2. 只有一路检测到黑线 ---
        else if (s.L1 == 0 && s.L2 == 1 && s.R1 == 1 && s.R2 == 1) {
            // 极左触发
            // TODO:
            Car_Left(LINE_SPEED_MID);
        }
        else if (s.L1 == 1 && s.L2 == 0 && s.R1 == 1 && s.R2 == 1) {
            // 中左触发
            // TODO:
            Car_Forward(LINE_SPEED_HIGH);
        }
        else if (s.L1 == 1 && s.L2 == 1 && s.R1 == 0 && s.R2 == 1) {
            // 中右触发
            // TODO:
            Car_Forward(LINE_SPEED_HIGH);
        }
        else if (s.L1 == 1 && s.L2 == 1 && s.R1 == 1 && s.R2 == 0) {
            // 极右触发
            // TODO:
            Car_Right(LINE_SPEED_MID);
        }

        // --- 3. 两路检测到黑线 (常见状态) ---
        else if (s.L1 == 1 && s.L2 == 0 && s.R1 == 0 && s.R2 == 1) {
            // 正中间直行 (L2, R1)
            // TODO:
            Car_Forward(LINE_SPEED_HIGH);
        }
        else if (s.L1 == 0 && s.L2 == 0 && s.R1 == 1 && s.R2 == 1) {
            // 左边两路 (偏离较多)
            // TODO:
            Motor_A_SetSpeed(LINE_SPEED_MID);
            Motor_B_SetSpeed(LINE_SPEED_HIGH);

        }
        else if (s.L1 == 1 && s.L2 == 1 && s.R1 == 0 && s.R2 == 0) {
            // 右边两路 (偏离较多)
            // TODO:
            Motor_A_SetSpeed(LINE_SPEED_HIGH);
            Motor_B_SetSpeed(LINE_SPEED_MID);
        }
        else if (s.L1 == 0 && s.L2 == 1 && s.R1 == 1 && s.R2 == 0) {
            // 极其罕见：两翼触发
            // TODO:
        }
        else if (s.L1 == 0 && s.L2 == 1 && s.R1 == 0 && s.R2 == 1) {
            // 罕见：L1, R1
            // TODO:
            Car_Forward(LINE_SPEED_MID);
        }
        else if (s.L1 == 1 && s.L2 == 0 && s.R1 == 1 && s.R2 == 0) {
            // 罕见：L2, R2
            // TODO:
            Car_Forward(LINE_SPEED_MID);
        }

        // --- 4. 三路检测到黑线 (丁字路口或大幅偏转) ---
        else if (s.L1 == 0 && s.L2 == 0 && s.R1 == 0 && s.R2 == 1) {
            // 左、中左、中右
            // TODO:
            Motor_A_SetSpeed(LINE_SPEED_LOW);
            Motor_B_SetSpeed(LINE_SPEED_MID);
        }
        else if (s.L1 == 1 && s.L2 == 0 && s.R1 == 0 && s.R2 == 0) {
            // 中左、中右、右
            // TODO:
            Motor_A_SetSpeed(LINE_SPEED_MID);
            Motor_B_SetSpeed(LINE_SPEED_LOW);
        }
        else if (s.L1 == 0 && s.L2 == 1 && s.R1 == 0 && s.R2 == 0) {
            // L1, R1, R2
            // TODO:
            Motor_A_SetSpeed(LINE_SPEED_HIGH);
            Motor_B_SetSpeed(LINE_SPEED_MID);
        }
        else if (s.L1 == 0 && s.L2 == 0 && s.R1 == 1 && s.R2 == 0) {
            // L1, L2, R2
            // TODO:
            Motor_A_SetSpeed(LINE_SPEED_MID);
            Motor_B_SetSpeed(LINE_SPEED_HIGH);
        }

        // --- 5. 全黑 (十字路口或停止线) ---
        else if (s.L1 == 0 && s.L2 == 0 && s.R1 == 0 && s.R2 == 0) {
            // TODO:
            Car_Forward(LINE_SPEED_MID);
        }



}