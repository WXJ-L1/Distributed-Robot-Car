//
// Created by 10663 on 26-5-1.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#define PWM_MAX 999

void TB6612_Init(void);
void TB6612_Enable(void);
void TB6612_Standby(void);

void Motor_A_SetSpeed(int speed);
void Motor_B_SetSpeed(int speed);

void Motor_Stop(void);
void Motor_Brake(void);

void Car_Forward(int speed);
void Car_Backward(int speed);
void Car_Left(int speed);
void Car_Right(int speed);

#endif //MOTOR_H
