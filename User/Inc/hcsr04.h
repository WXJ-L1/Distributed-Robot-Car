//
// Created by wxj on 26-4-9.
//

#ifndef HCSR04_C_H
#define HCSR04_C_H

#include "main.h"

typedef struct
{
    GPIO_TypeDef *TRIG_Port;
    uint16_t      TRIG_Pin;
    TIM_HandleTypeDef *htim;
    uint32_t      channel;
} HCSR04_HandleTypeDef;


void HCSR04_Init(HCSR04_HandleTypeDef *dev);
void HCSR04_Trigger(HCSR04_HandleTypeDef *dev);
float HCSR04_GetDistance(void);
void Ultrasonic_Init_All(void);
float Measure_Distance(void);

#endif //HCSR04_C_H
