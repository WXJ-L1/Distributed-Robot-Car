//
// Created by zzy on 2026/5/5.
//

#ifndef FIRST_CAR_TURNING_H
#define FIRST_CAR_TURNING_H
#ifndef __TURNING_H
#define __TURNING_H

#include "main.h"

// 声明外部红外状态读取函数 (借用你 linewalking.c 里的数据结构)
#include <stdbool.h>

#include "linewalking.h"

// ========== 用户可以调节的转弯参数 ==========
#define TURN_SPEED_DEFAULT  600   // 原地转弯的默认速度 (依据你的PWM 0-999)
#define TURN_SPEED_MID  500
#define TURN_SPEED_LOW  350
#define TURN_SPEED_HIGH  750
#define TURN_SPEED_MAX 1000

// ========== 核心功能函数声明 ==========

// 1. 小功能：执行真正的原地左转动作
void Action_TurnLeft(int speed);

// 2. 小功能：执行真正的原地右转动作
void Action_TurnRight(int speed);

// 3. 小功能：判断四路红外是否全亮 (全没检测到黑线)
uint8_t Check_All_Infrared_Light(LineState_t state);

// 4. 大功能：红外全亮触发 + 陀螺仪闭环的智能转弯
// target_angle: 正数左转，0直行，负数右转 (单位：度)
int8_t Smart_Turning_Handler(int target_angle,int8_t start_n,int8_t end_n);
void Parse_And_Execute_Sequence(const char* input);
// 在 turning.h 中添加：
void Execute_Auto_Driving(int8_t* road, int8_t road_len, int* turn_angles, int8_t turn_angles_len);

#endif /* __TURNING_H */
#endif //FIRST_CAR_TURNING_H
