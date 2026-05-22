//
// Created by wxj on 26-4-8.
//

// UART.h
#ifndef __UART_H
#define __UART_H

#define ESP_RX_SIZE 1024
#include <stdint.h>

// --- 串口 2 (Camera) 变量声明 ---
extern uint8_t camera_frame_buf[ESP_RX_SIZE];
extern uint16_t camera_data_len;
extern volatile uint8_t camera_frame_ready;
// --- 串口 3 (WiFi) 变量声明 ---
extern uint8_t wifi_rx_dma_buf[ESP_RX_SIZE];
extern uint8_t wifi_frame_buf[ESP_RX_SIZE];
extern uint16_t wifi_data_len;
extern uint8_t wifi_frame_ready;

void UART1_SendString(char *str);
void UART2_SendString(char *str);
void ESP8266_StartDMA(void);
void ESP8266_SendRaw(char *cmd);
uint8_t ESP8266_UDP_Send(char *data, uint16_t len);
uint16_t ESP8266_Read(char *out);
uint8_t ESP8266_WaitFor(char *ack, uint32_t timeout);
uint8_t ESP8266_SendCmd(const char *cmd, const char *ack, uint32_t timeout);
void WIFI_Init(void);
void WIFI_ProcessCachedIPD(void);
uint8_t WIFI_GetCachedIPD(char *out, uint16_t out_size);
#endif /* __UART_H */
