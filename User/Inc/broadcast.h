//
// Created by zzy on 26-5-7.
// Modified: Basic Logic-based line tracking (No PD).
//
#ifndef BROADCAST_H
#define BROADCAST_H

#include <stdint.h>

/*
 * 每辆车单独改这里
 */
#define MY_CAR_ID 1
#define BIRTH_NODE 41

extern volatile uint8_t work_state;

extern int8_t car_current_node;
extern int8_t car_final_node;
extern int8_t current_start_node;
extern int8_t current_end_node;

extern uint8_t Check_And_Broadcast_Task_a;

void Check_And_Broadcast_Task(void);
void Parse_Broadcast_Message(const char* json_str);
void Parse_MQTT_Broadcast_Message(const char *json_str);

uint8_t Broadcast_Type0_State(void);
uint8_t Broadcast_Type1_Road(void);
void Broadcast_Type2_Position(void);
void Broadcast_Type4_OnlineStatus(uint8_t online_status);
void Broadcast_Debug_Angles(void);
void Try_Apply_Path(void);

uint8_t Broadcast_Type12_Service(int state,
                                 int8_t dest_node,
                                 int8_t service_node,
                                 int8_t leave_to_node,
                                 int8_t wait_node,
                                 int8_t owner_car_id);

#endif