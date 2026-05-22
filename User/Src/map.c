//
// Created by zzy on 26-5-7.
// Modified: Basic Logic-based line tracking (No PD).
//

#include "map.h"
#include <stddef.h>
#include <stdio.h>

#include "groundtrue.h"
#include "UART.h"

// 定义三个矩阵
int16_t Global_Map[MAX_NODES][MAX_NODES];
int16_t Temp_Map[MAX_NODES][MAX_NODES];
int Angle_Matrix[MAX_NODES][MAX_NODES];

/**
 * @brief 将手写的 map_edges 数组转化为邻接矩阵
 */
void Map_Init(void) {
    // 1. 初始化所有矩阵
    for(int i = 0; i < MAX_NODES; i++) {
        for(int j = 0; j < MAX_NODES; j++) {
            if (i == j) {
                Global_Map[i][j] = 0;
                Angle_Matrix[i][j] = 0;
            } else {
                Global_Map[i][j] = INF;
                Angle_Matrix[i][j] = INF;
            }
        }
    }

    // 2. 遍历手写地图数组，构建全局静态图和转弯矩阵
    for(int i = 0; i < num_edges; i++) {
        int8_t u = map_edges[i].start_node;
        int8_t v = map_edges[i].end_node;
        int16_t w = map_edges[i].weight;
        int a = map_edges[i].angle;

        // 赋值单向边
        Global_Map[u][v] = w;
        Angle_Matrix[u][v] = a;

        // 如果物理地图是双向的，后续可以加上反向赋值逻辑，例如:
        // Global_Map[v][u] = w;
        // 反向角度计算...
    }

    // 3. 初始化时，将临时图重置为与全局静态图一致
    Reset_Temp_Map();
}

/**
 * @brief 重置临时图为全局静态图
 */
void Reset_Temp_Map(void) {
    for(int i = 0; i < MAX_NODES; i++) {
        for(int j = 0; j < MAX_NODES; j++) {
            Temp_Map[i][j] = Global_Map[i][j];
        }
    }
}


// 定义全局路径数组和长度
int8_t car_road[MAX_NODES];
int8_t car_road_len = 0;

/**
 * @brief 获取路径算法 (使用 Dijkstra 算法)
 * @param start_node 初始节点
 * @param end_node 终节点
 * @return 1代表找到，0代表没找到
 */
uint8_t Get_Path_Algorithm(int8_t start_node, int8_t end_node) {
    int16_t dist[MAX_NODES];     // 存储起始点到各点的最短距离
    int8_t visited[MAX_NODES];  // 标记是否已找到最短路径
    int8_t parent[MAX_NODES];   // 用于回溯路径的父节点数组

    // 1. 初始化
    for (int8_t i = 0; i < MAX_NODES; i++) {
        dist[i] = INF;
        visited[i] = 0;
        parent[i] = -1;
        car_road[i] = -1;    // 清空上次的路径记录
    }
    car_road_len = 0;
    dist[start_node] = 0;

    // 2. Dijkstra 核心循环 (基于 Temp_Map) [cite: 7, 9]
    for (int8_t count = 0; count < MAX_NODES; count++) {
        int16_t min_dist = INF;
        int8_t u = -1;

        // 寻找未访问集合中距离最小的顶点
        for (int8_t i = 0; i < MAX_NODES; i++) {
            if (!visited[i] && dist[i] < min_dist) {
                min_dist = dist[i];
                u = i;
            }
        }

        // 如果找不到可达顶点，或者已经找到了终点，就可以提前退出
        if (u == -1) break;
        if (u == end_node) {
            visited[u] = 1;
            break;
        }

        visited[u] = 1;

        // 更新相邻顶点的距离
        for (int8_t v = 0; v < MAX_NODES; v++) {
            // 注意：依赖 Temp_Map 寻找路径 [cite: 7, 9]
            if (!visited[v] && Temp_Map[u][v] != INF) {
                if (dist[u] + Temp_Map[u][v] < dist[v]) {
                    dist[v] = dist[u] + Temp_Map[u][v];
                    parent[v] = u;
                }
            }
        }
    }

    // 3. 判断是否找到到达终点的路径
    if (dist[end_node] == INF) {
        // 3. 判断是否找到到达终点的路径
        if (dist[end_node] == INF) {
            // ===========================================================
            // 【新增逻辑】：寻路失败，使用串口打印当前 Temp_Map 状态
            // ===========================================================
            char debug_buf[128];
            sprintf(debug_buf, "\r\n[Error] Path Failed! %d -> %d\r\nReachable Nodes: ", start_node, end_node);
            UART1_SendString(debug_buf);

            // 遍历所有节点，只要距离不是 INF，说明起点能走到它
            int reached_count = 0;
            for (int i = 0; i < MAX_NODES; i++) {
                if (dist[i] != INF) {
                    sprintf(debug_buf, "%d(d:%d) ", i, dist[i]);
                    UART1_SendString(debug_buf);
                    reached_count++;
                }
            }

            sprintf(debug_buf, "\r\nTotal Reached: %d / %d\r\n", reached_count, MAX_NODES);
            UART1_SendString(debug_buf);
            // ===========================================================
            return 0; // 0代表没找到
        }


        return 0; // 0代表没找到
    }

    // 4. 回溯生成路径数组并存入全局变量
    int8_t temp_path[MAX_NODES];
    int8_t temp_len = 0;
    int8_t curr = end_node;

    while (curr != -1) {
        temp_path[temp_len++] = curr;
        curr = parent[curr];
    }

    // 因为回溯是逆序的(从尾到头)，需要反转后放入全局变量 car_road 中
    for (int i = 0; i < temp_len; i++) {
        car_road[i] = temp_path[temp_len - 1 - i];
    }
    car_road_len = temp_len;

    return 1; // 1代表找到
}

/**
 * @brief 根据广播的路径更新临时图
 * @param path_nodes 接收到的路径数组
 * @param length 路径数组的长度
 */
void Update_Temp_Map_By_Road(int8_t* path_nodes, int8_t length) {
    if (path_nodes == NULL || length <= 0) return;

    // 遍历路径中的每一个节点
    for (int8_t k = 0; k < length; k++) {
        int8_t target_node = path_nodes[k];

        // 保护：确保节点编号在合法范围内
        if (target_node >= 0 && target_node < MAX_NODES) {
            // 将临时图中该节点对应的列的权值均设为最大值 (INF)
            for (int8_t i = 0; i < MAX_NODES; i++) {
                Temp_Map[i][target_node] = INF;
            }
        }
    }
}

// 定义全局相对转弯角度数组
int relative_turn_angles[MAX_NODES];
int8_t relative_turn_angles_len = 0;

// -1代表系统刚启动，还没有上一个节点
int8_t last_visited_node = -1;

/**
 * @brief 根据全局路径数组计算相对转弯角度
 */
/**
 * @brief 根据全局路径数组计算相对转弯角度
 */
void Calculate_Relative_Turn_Angles(void) {
    relative_turn_angles_len = 0;

    // 保护机制：如果路径不存在或只有一个节点，无需转弯
    if (car_road_len < 2) {
        return;
    }

    // 5.2 从第一个节点开始迭代循环。如果不是最后一个节点，则进入逻辑
    for (int8_t i = 0; i < car_road_len - 1; i++) {
        int8_t curr_node = car_road[i];
        int8_t next_node = car_road[i + 1];
        int8_t prev_node = -1;

        // 5.2.1 读取上一个节点的编号
        if (i == 0) {
            // 第一个节点的上一个节点来自于上一次任务的节点
            prev_node = last_visited_node;
        } else {
            // 后续节点的上一个节点即为路径数组中的前一个
            prev_node = car_road[i - 1];
        }

        if (prev_node == -1) {
            // 5.2.2 如果上一个节点是-1，添加0度到相对转弯角度数组
            relative_turn_angles[relative_turn_angles_len++] = 0;
        } else {
            // 5.2.3 通过当前节点，读取两个绝对角度
            int angle1 = Angle_Matrix[curr_node][prev_node]; // 看向来时路的绝对角度
            int angle2 = Angle_Matrix[curr_node][next_node]; // 看向去时路的绝对角度

            // 【关键修改】：小车当前的车头绝对朝向，是来时路角度的反方向（加或减 180 度）
           int car_heading = angle1 + 180;

            // 将车头朝向初步限制在 [-180, 180] 范围内，方便逻辑理解
            while (car_heading > 180) {
                car_heading -= 360;
            }
            while (car_heading < -180) {
                car_heading += 360;
            }

            // 计算从“车头朝向”转到“目标绝对角度”的相对转弯角度
            int diff = angle2 - car_heading;

            // 将最终的相对转弯角度限制在闭区间 [-180, 180] 之间
            // 正数代表逆时针转，负数代表顺时针转
            while (diff > 180) {
                diff -= 360;
            }
            while (diff < -180) {
                diff += 360;
            }

            // 添加这个相对角度到相对转弯角度数组
            relative_turn_angles[relative_turn_angles_len++] = diff;
        }
    }

    // 迭代结束后，将本次路径的倒数第二个节点保存下来。
    // 这样当小车在终点接到下一个新任务时，它就知道刚才是从哪个节点开过来的了。
    // if (car_road_len > 1) {
    //     last_visited_node = car_road[car_road_len - 2];
    // }
}

/**
 * @brief 根据其他小车的位置动态更新临时图
 * @param start_node 头节点编号
 * @param end_node 尾节点编号 (或-1代表在头节点上)
 */
void Update_Temp_Map_By_Position(int8_t start_node, int8_t end_node) {
    // 保护：确保节点编号在合法范围内
    if (start_node < 0 || start_node >= MAX_NODES) return;

    // 如果其他小车在节点上 (end_node == -1) [cite: 22, 51]
    if (end_node == -1) {
        // 把该节点(start_node)对应的列权值设为最大值
        for (int8_t i = 0; i < MAX_NODES; i++) {
            Temp_Map[i][start_node] = INF;
        }
    }
    // 如果其他小车在线上 (end_node != -1) [cite: 22, 51]
    else if (end_node >= 0 && end_node < MAX_NODES) {
        // 1. 把这段线的尾节点(end_node)对应的列权值设为最大值
        for (int8_t i = 0; i < MAX_NODES; i++) {
            Temp_Map[i][end_node] = INF;
        }
        // 2. 并把头节点(start_node)的权值恢复 (从全局静态图中读取原本的权值)
        for (int8_t i = 0; i < MAX_NODES; i++) {
            Temp_Map[i][start_node] = Global_Map[i][start_node];
        }
    }
}