//
// Created by zzy on 26-5-7.
// Modified: Basic Logic-based line tracking (No PD).
//

#ifndef MAP_H
#define MAP_H

#define MAX_NODES 50
#define INF 127
#include <stdint.h>

extern int16_t Global_Map[MAX_NODES][MAX_NODES];
extern int16_t Temp_Map[MAX_NODES][MAX_NODES];
extern int Angle_Matrix[MAX_NODES][MAX_NODES];

// 新增：全局路径数组与路径长度
extern int8_t car_road[MAX_NODES];
extern int8_t car_road_len;

void Map_Init(void);
void Reset_Temp_Map(void);

// 新增：获取路径算法
uint8_t Get_Path_Algorithm(int8_t start_node, int8_t end_node);
// 在 map.h 末尾添加：
void Update_Temp_Map_By_Road(int8_t* path_nodes, int8_t length);
// 在 map.h 的末尾添加：

// 全局相对转弯角度数组及其长度
extern int relative_turn_angles[MAX_NODES];
extern int8_t relative_turn_angles_len;

// 记录上一次任务的上一个节点编号（初始为-1）
extern int8_t last_visited_node;

// 声明转弯计算函数
void Calculate_Relative_Turn_Angles(void);
// 根据其他小车的位置动态更新临时图
void Update_Temp_Map_By_Position(int8_t start_node, int8_t end_node);

#endif // MAP_H