//
// Created by 10663 on 26-5-22.
//

#ifndef SERVICE_WAIT_H
#define SERVICE_WAIT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERVICE_WAIT_IDLE              0
#define SERVICE_WAIT_GOING_WAIT_NODE   1
#define SERVICE_WAIT_AT_WAIT_NODE      2

/*
 * type12 state 定义
 */
#define SERVICE_TYPE12_SERVING         1   // 前车正在目的地服务
#define SERVICE_TYPE12_LEAVE_PLAN      2   // 前车公布离开方向和等待点
#define SERVICE_TYPE12_WAIT_ACCEPT     3   // 后车确认：我要去等待点
#define SERVICE_TYPE12_WAITING_NODE    4   // 后车已到等待点

void ServiceWait_Reset(void);
/*
 * 目的地 / 服务节点 / 等待点查询
 */
uint8_t ServiceWait_IsDestSupported(int8_t dest_node);
int8_t ServiceWait_GetServiceNode(int8_t dest_node);
int8_t ServiceWait_GetLeftWaitNode(int8_t dest_node);
int8_t ServiceWait_GetRightWaitNode(int8_t dest_node);
/*
 * 根据前车 leave_to_node，选择相反方向的 wait_node
 */
int8_t ServiceWait_GetWaitNodeByLeaveTo(int8_t dest_node, int8_t leave_to_node);

/*
 * 根据前车当前目的地和下一目标，推算前车离开 service_node 后往哪边走
 * 例如：
 * current_dest = 21, next_target = 41/42/43 -> leave_to_node = 0
 * current_dest = 21, next_target = 24/22/25/23 -> leave_to_node = 4
 */
int8_t ServiceWait_GetLeaveToNodeByNextTarget(int8_t current_dest_node,
                                              int8_t next_target_node);

/*
 * 启动服务等待流程：
 * 后车收到前车 type12 state=2 后调用。
 */
uint8_t ServiceWait_Start(int8_t owner_car_id,
                          int8_t real_dest_node,
                          int8_t service_node,
                          int8_t wait_node,
                          int8_t owner_leave_to_node);

/*
 * 状态查询
 */
uint8_t ServiceWait_IsActive(void);
uint8_t ServiceWait_IsGoingToWaitNode(void);
uint8_t ServiceWait_IsAtWaitNode(void);
/*
 * 后车到达等待点后调用
 */
void ServiceWait_MarkArrivedWaitNode(void);

/*
 * 收到 type2 后调用，用于判断前车是否已经离开 service_node。
 * 例如：
 * 前车 car1 从 21 离开后走 21 -> 1 -> 0；
 * 后车在 4 等待；
 * 当收到 car1 的 type2: start_node=1,end_node=0，
 * 返回 1，表示后车可以从 4 -> 1 -> 21。
 */
uint8_t ServiceWait_CheckOwnerLeftEntry(int8_t car_id,
                                        int8_t start_node,
                                        int8_t end_node);
/*
 * getter
 */
int8_t ServiceWait_GetMode(void);
int8_t ServiceWait_GetOwnerCarId(void);
int8_t ServiceWait_GetRealDestNode(void);
int8_t ServiceWait_GetServiceNodeValue(void);
int8_t ServiceWait_GetWaitNodeValue(void);
int8_t ServiceWait_GetOwnerLeaveToNode(void);

#ifdef __cplusplus
}
#endif

#endif
