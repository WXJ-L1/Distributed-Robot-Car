//
// Created by zzy on 2026/5/8.
//
//
// Created by zzy on 2026/5/8.
//

#include "car_queue.h"
#include <stdio.h>

#include "UART.h"

QueueNode_t Car_Queue[MAX_QUEUE_CARS];
int queue_length = 0;


// 定义全局变量，初始化为0代表没有接受到type0的更新，1代表接受到type0的更新

int8_t AcceptUpdate = 0;

/**
 * @brief 获取当前队列队头的小车ID
 * @return 队头车ID，如果队列为空则返回-1
 */
int Get_Head_CarId(void) {
    if (queue_length > 0) {
        return Car_Queue[0].car_id;
    }
    return -1;
}

/**
 * @brief 初始化队列
 */
void Car_Queue_Init(void) {
    queue_length = 0;
    for (int i = 0; i < MAX_QUEUE_CARS; i++) {
        Car_Queue[i].car_id = -1;
        Car_Queue[i].work = 0;
    }
}

/**
 * @brief 从队列中移除指定小车
 * @param target_car_id 需要移除的小车ID
 */
void Car_Queue_Remove(int target_car_id) {
    int index = -1;
    // 查找目标小车在队列中的位置
    for (int i = 0; i < queue_length; i++) {
        if (Car_Queue[i].car_id == target_car_id) {
            index = i;
            break;
        }
    }

    // 如果找到了，将其后面的元素往前移一位
    if (index != -1) {
        for (int i = index; i < queue_length - 1; i++) {
            Car_Queue[i] = Car_Queue[i + 1];
        }
        queue_length--;
    }
}

/**
 * @brief 根据接收到的 type0 信息更新排队队列
 * @param received_car_id 接收到的小车ID (临时变量)
 * @param received_work   接收到的任务状态 (临时变量)
 */
/**
 * @brief 根据接收到的 type0 信息更新排队队列
 * @return 1: 成功加入或更新了队列; 0: 未加入队列(如满载、已存在或为非排队状态)
 */
uint8_t Car_Queue_Update(int received_car_id, int8_t received_work) {
    // 只有 1(配送) 和 3(回去) 才需要排队。
    if (received_work != 1 && received_work != 3) {
        Car_Queue_Remove(received_car_id);
        return 0; // 退出排队，返回0
    }

    for (int i = 0; i < queue_length; i++) {
        if (Car_Queue[i].car_id == received_car_id) {
            if (Car_Queue[i].work == received_work) {
                return 0; // 已在队列且状态未变，无需重复加入
            } else {
                Car_Queue_Remove(received_car_id);
                break;
            }
        }
    }

    if (queue_length >= MAX_QUEUE_CARS) return 0; // 队列满保护

    int insert_index = queue_length;

    /*
     * 稳定排序规则：
     * 1. work=1 排在 work=3 前面；
     * 2. 同一个 work 内，car_id 小的排前面。
     * 这样两辆车无论先收到谁的 MQTT，最终队列顺序都一致。
     */
    for (int i = 0; i < queue_length; i++) {
        if (received_work < Car_Queue[i].work) {
            insert_index = i;
            break;
        }

        if (received_work == Car_Queue[i].work &&
            received_car_id < Car_Queue[i].car_id) {
            insert_index = i;
            break;
            }
    }

    for (int i = queue_length; i > insert_index; i--) {
        Car_Queue[i] = Car_Queue[i - 1];
    }

    Car_Queue[insert_index].car_id = received_car_id;
    Car_Queue[insert_index].work = received_work;
    queue_length++;

    Car_Queue_Print();

    return 1; // 成功加入排队
}

/**
 * @brief 打印当前队列情况
 */
void Car_Queue_Print(void) {
    char print_buf[64];
    UART1_SendString("[Queue] Current -> ");
    if (queue_length == 0) {
        UART1_SendString("Empty\r\n");
    } else {
        for (int i = 0; i < queue_length; i++) {
            sprintf(print_buf, "[ID:%d W:%d] ", Car_Queue[i].car_id, Car_Queue[i].work);
            UART1_SendString(print_buf);
        }
    }
    UART1_SendString("\r\n");
}

/**
 * @brief 获取队列中下一辆车的ID
 * @param current_car_id 当前小车ID
 * @return 下一辆车的ID
 */
int Get_Next_CarId_In_Queue(int8_t current_car_id) {
    if (queue_length == 0) {
        return -1; // 队列为空
    }

    for (int i = 0; i < queue_length; i++) {
        if (Car_Queue[i].car_id == current_car_id) {
            // 从队列中得到下一辆车的id，如果到结尾则跳到队头 [cite: 5]
            if (i == queue_length - 1) {
                return Car_Queue[0].car_id;
            } else {
                return Car_Queue[i + 1].car_id;
            }
        }
    }

    // 如果由于某种异常（比如本车不在排队队列中），默认返回队头保护一下
    return Car_Queue[0].car_id;
}

/**
 * @brief 【新增】检查在队列中，my_id 是否排在 sender_id 的下一位（循环检查）
 */
int8_t Is_Next_Car(int8_t sender_id, int8_t my_id) {
    if (queue_length == 0) return 0;

    int sender_idx = -1;
    // 找到 sender_id 的位置
    for (int i = 0; i < queue_length; i++) {
        if (Car_Queue[i].car_id == sender_id) {
            sender_idx = i;
            break;
        }
    }

    if (sender_idx != -1) {
        // 下一辆车的索引，如果是队尾，通过取余自动跳到队头 (0)
        int next_idx = (sender_idx + 1) % queue_length;
        if (Car_Queue[next_idx].car_id == my_id) {
            return 1; // 确实轮到自己了
        }
    }

    return 0; // 没轮到自己
}