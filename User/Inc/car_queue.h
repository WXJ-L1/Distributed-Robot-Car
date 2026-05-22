//
// Created by zzy on 2026/5/8.
//

#ifndef CAR_QUEUE_H
#define CAR_QUEUE_H

#include <stdint.h>

#define MAX_QUEUE_CARS 10

typedef struct {
    int car_id;
    int8_t work;
} QueueNode_t;

extern QueueNode_t Car_Queue[MAX_QUEUE_CARS];
extern int queue_length;

// 新增全局变量声明，用于记录下一个准备申请路径的车
extern int8_t AcceptUpdate;

void Car_Queue_Init(void);
// 修改返回值类型为 uint8_t
uint8_t Car_Queue_Update(int received_car_id, int8_t received_work);
void Car_Queue_Remove(int target_car_id);
void Car_Queue_Print(void);

// 新增获取队头小车ID的函数声明
int Get_Head_CarId(void);
/**
 * @brief 获取队列中当前小车的下一辆车ID
 * @param current_car_id 当前小车ID
 * @return 下一辆车的ID，如果队列为空则返回-1
 */
int Get_Next_CarId_In_Queue(int8_t current_car_id);

// 【新增】：判断在队列中，my_id 是否排在 sender_id 的后面
int8_t Is_Next_Car(int8_t sender_id, int8_t my_id);
#endif // CAR_QUEUE_H
