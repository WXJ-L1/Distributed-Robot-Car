//
// Created by wxj on 26-4-10.
//

#ifndef LINEWALKING_H
#define LINEWALKING_H

#include "main.h"

typedef struct
{
    GPIO_PinState L1;   // 最左
    GPIO_PinState L2;   // 左中
    GPIO_PinState R1;   // 右中
    GPIO_PinState R2;   // 最右
} LineState_t;

LineState_t Line_Read(void);
void Line_PrintState(LineState_t s);
void LineWalking(void);

#endif //LINEWALKING_H
