//
// Created by zzy on 2026/5/8.
//

#ifndef GROUNDTRUE_H
#define GROUNDTRUE_H
#include <stdint.h>

// 定义单向路径边的结构体，全部使用整型
typedef struct {
    int8_t start_node;   // 头节点
    int8_t end_node;     // 尾节点
    int16_t weight;       // 权值 (距离或时间，整数)
    int angle;        // 绝对角度: [-180, 180]的整数
} MapEdge_t;

extern const MapEdge_t map_edges[];
extern const int16_t num_edges;

#endif // GROUNDTRUE_H
