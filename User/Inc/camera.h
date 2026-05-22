#ifndef CAMERA_H
#define CAMERA_H

#include "main.h"
#include <stdio.h>
#include <string.h>

/* 配置部分 */
#define MODE_AP_STA     1
#define ESP_RX_SIZE     256  // 增大缓冲区以适应长二维码

typedef struct {
    int16_t lx, ly, rx, ry;
    int16_t cx, cy;
    uint16_t area;
    int16_t id;
} ESP32_AI_Msg;

typedef enum {
    Nornal_AI = 0,
    Cat_Dog_AI,
    FACE_AI,
    COLOR_AI,
    REFACE_AI,
    QR_AI = 5
} AI_mode;

typedef struct {
    char QR_msg[128]; // 增大二维码存储空间
} QR_AI_Msg;

/* 外部变量声明 */
extern volatile uint8_t newlines;
extern AI_mode runmode;
extern QR_AI_Msg QR_msg;
extern ESP32_AI_Msg esp32_ai_msg;
extern uint8_t camera_rx_dma_buf[ESP_RX_SIZE];
extern uint8_t cmd_flag;

/* 函数声明 */
void ESP32_Module_Config_Init(AI_mode AI_set_mode);
void Data_Deal(uint8_t RXdata);
void SET_ESP_AI_MODE(AI_mode Mode);
void Get_STAIP(void);
void Get_APIP(void);

#endif //CAMERA_H