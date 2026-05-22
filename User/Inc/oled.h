#ifndef __OLED_H
#define __OLED_H

#include "main.h"

void OLED_Init(void);
void OLED_NewFrame(void);
void OLED_ShowFrame(void);

void OLED_ShowChar(uint8_t x, uint8_t y, char c);
void OLED_ShowString(uint8_t x, uint8_t y, char *str);

#endif