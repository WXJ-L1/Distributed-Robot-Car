//
// Created by 10663 on 26-5-16.
//

#ifndef __MQTT_H
#define __MQTT_H

#include <stdint.h>
#include "usart.h"

#define MQTT_TOPIC "car/broadcast"
#define MQTT_MSG_MAX_LEN 256
#define MQTT_TOPIC "car/broadcast"

uint8_t MQTT_Init(void);
uint8_t MQTT_Send(const char *topic, const char *msg);
uint8_t MQTT_StartDmaReceive(void);
uint8_t MQTT_ProcessDmaToQueue(void);
void MQTT_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif
