//
// Created by zzy on 26-5-7.
// Modified: Basic Logic-based line tracking (No PD).
//
#ifndef BROADCAST_H
#define BROADCAST_H


#include <stdint.h>
extern volatile uint8_t work_state;
// 假设定义该小车的 ID 为 1，实际项目中可从配置或硬件拨码开关读取
#define MY_CAR_ID 2
#define BIRTH_NODE 42
extern int8_t car_current_node;
extern int8_t car_final_node;
extern int8_t current_start_node;
extern int8_t current_end_node;
void Check_And_Broadcast_Task(void) ;
void Parse_Broadcast_Message(const char* json_str);
uint8_t Broadcast_Type0_State(void);
uint8_t Broadcast_Type1_Road(void);
void Broadcast_Type2_Position(void);
void Broadcast_Type4_OnlineStatus(uint8_t online_status);
void Parse_MQTT_Broadcast_Message(const char *json_str);
void Try_Apply_Path(void);
#endif // BROADCAST_H