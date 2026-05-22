#include "buzzer.h"

/**
 * @brief 打开蜂鸣器 (PG7 拉高)
 */
void Buzzer_On(void) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
}

/**
 * @brief 关闭蜂鸣器 (PG7 拉低)
 */
void Buzzer_Off(void) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
}

/**
 * @brief 蜂鸣器响指定的时间后自动关闭 (阻塞式)
 * @param time_ms 响的时长，单位：毫秒
 */
void Buzzer_Beep(uint16_t time_ms) {
    Buzzer_On();
    HAL_Delay(time_ms);
    Buzzer_Off();
}