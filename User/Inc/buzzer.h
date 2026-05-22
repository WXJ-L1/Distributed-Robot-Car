#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"

// 基础控制函数
void Buzzer_On(void);
void Buzzer_Off(void);

// 快捷提示音函数
void Buzzer_Beep(uint16_t time_ms);

#endif /* __BUZZER_H */