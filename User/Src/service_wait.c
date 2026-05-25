//
// Created by 10663 on 26-5-22.
//
#include "service_wait.h"
#include "map.h"

typedef struct
{
    int8_t dest_node;        // 目的地节点，例如 21
    int8_t service_node;     // 主路服务节点，例如 21 对应 1
    int8_t left_wait_node;   // 左侧等待点
    int8_t right_wait_node;  // 右侧等待点
} ServiceRule_t;

static const ServiceRule_t service_rules[] = {
    {21, 1, 0, 4},
    {24, 4, 1, 2},
    {22, 2, 4, 5},
    {25, 5, 2, 3},
    {23, 3, 5, -1}
};

#define SERVICE_RULE_COUNT  ((uint8_t)(sizeof(service_rules) / sizeof(service_rules[0])))
/*
 * 主路顺序表，用于判断某个目标在当前服务节点左边还是右边。
 */
static const int8_t main_line_nodes[] = {
    0, 1, 4, 2, 5, 3
};

#define MAIN_LINE_COUNT  ((uint8_t)(sizeof(main_line_nodes) / sizeof(main_line_nodes[0])))
/*
 * 当前本车服务等待状态
 */
static uint8_t sw_mode = SERVICE_WAIT_IDLE;
static int8_t sw_owner_car_id = -1;          // 当前占用目的地的前车
static int8_t sw_real_dest_node = -1;        // 本车真正想去的目的地，例如 21
static int8_t sw_service_node = -1;          // 目的地对应主路服务节点，例如 1
static int8_t sw_wait_node = -1;             // 本车要去等待的节点，例如 4
static int8_t sw_owner_leave_to_node = -1;   // 前车离开 service_node 后去的方向，例如 0 或 4

static int8_t ServiceWait_FindRuleIndex(int8_t dest_node){
    for(uint8_t i = 0; i < SERVICE_RULE_COUNT; i++){
        if(service_rules[i].dest_node == dest_node){
            return (int8_t)i;
        }
    }

    return -1;
}

static int8_t ServiceWait_FindMainLineIndex(int8_t node){
    for(uint8_t i = 0; i < MAIN_LINE_COUNT; i++){
        if(main_line_nodes[i] == node){
            return (int8_t)i;
        }
    }

    return -1;
}

/*
 * 把目的地节点或主路节点统一转换成主路服务节点。
 *
 * 例如：
 * 21 -> 1
 * 24 -> 4
 * 22 -> 2
 * 0  -> 0
 * 1  -> 1
 * 4  -> 4
 * 41/42/43 -> 0
 */
static int8_t ServiceWait_ConvertToMainNode(int8_t node){
    /*
     * 出生点 41/42/43 都是从 0 接入主路。
     */
    if(node == 41 || node == 42 || node == 43){
        return 0;
    }

    /*
     * 如果本身就是主路节点，直接返回。
     */
    if(ServiceWait_FindMainLineIndex(node) >= 0){
        return node;
    }

    /*
     * 如果是目的地节点，返回它对应的 service_node。
     */
    int8_t idx = ServiceWait_FindRuleIndex(node);
    if(idx >= 0){
        return service_rules[idx].service_node;
    }

    return -1;
}

void ServiceWait_Reset(void){
    sw_mode = SERVICE_WAIT_IDLE;
    sw_owner_car_id = -1;
    sw_real_dest_node = -1;
    sw_service_node = -1;
    sw_wait_node = -1;
    sw_owner_leave_to_node = -1;
}

uint8_t ServiceWait_IsDestSupported(int8_t dest_node){
    return (ServiceWait_FindRuleIndex(dest_node) >= 0) ? 1 : 0;
}

int8_t ServiceWait_GetServiceNode(int8_t dest_node){
    int8_t idx = ServiceWait_FindRuleIndex(dest_node);
    if(idx < 0){
        return -1;
    }

    return service_rules[idx].service_node;
}

int8_t ServiceWait_GetLeftWaitNode(int8_t dest_node){
    int8_t idx = ServiceWait_FindRuleIndex(dest_node);

    if(idx < 0){
        return -1;
    }
    return service_rules[idx].left_wait_node;
}

int8_t ServiceWait_GetRightWaitNode(int8_t dest_node){
    int8_t idx = ServiceWait_FindRuleIndex(dest_node);
    if(idx < 0){
        return -1;
    }
    return service_rules[idx].right_wait_node;
}

int8_t ServiceWait_GetWaitNodeByLeaveTo(int8_t dest_node, int8_t leave_to_node){
    int8_t idx = ServiceWait_FindRuleIndex(dest_node);
    if(idx < 0){
        return -1;
    }

    /*
     * 前车往左走，后车去右边等。
     */
    if(leave_to_node == service_rules[idx].left_wait_node){
        return service_rules[idx].right_wait_node;
    }
    /*
     * 前车往右走，后车去左边等。
     */
    if(leave_to_node == service_rules[idx].right_wait_node){
        return service_rules[idx].left_wait_node;
    }

    return -1;
}

int8_t ServiceWait_GetLeaveToNodeByNextTarget(int8_t current_dest_node,
                                              int8_t next_target_node){
    int8_t idx = ServiceWait_FindRuleIndex(current_dest_node);
    if(idx < 0){
        return -1;
    }

    int8_t current_service_node = service_rules[idx].service_node;
    int8_t next_main_node = ServiceWait_ConvertToMainNode(next_target_node);

    if(next_main_node < 0){
        return -1;
    }

    int8_t current_index = ServiceWait_FindMainLineIndex(current_service_node);
    int8_t next_index = ServiceWait_FindMainLineIndex(next_main_node);

    if(current_index < 0 || next_index < 0)
    {
        return -1;
    }

    /*
     * 下一目标在当前 service_node 左侧。
     */
    if(next_index < current_index)
    {
        return service_rules[idx].left_wait_node;
    }

    /*
     * 下一目标在当前 service_node 右侧。
     */
    if(next_index > current_index)
    {
        return service_rules[idx].right_wait_node;
    }

    /*
     * 如果 next_target 仍然在同一个 service_node 附近，
     * 无法判断明确离开方向。
     */
    return -1;
}

uint8_t ServiceWait_Start(int8_t owner_car_id,
                          int8_t real_dest_node,
                          int8_t service_node,
                          int8_t wait_node,
                          int8_t owner_leave_to_node)
{
    if(owner_car_id < 0)
    {
        return 0;
    }

    if(!ServiceWait_IsDestSupported(real_dest_node))
    {
        return 0;
    }

    if(service_node < 0 || service_node >= MAX_NODES)
    {
        return 0;
    }

    if(wait_node < 0 || wait_node >= MAX_NODES)
    {
        return 0;
    }

    if(owner_leave_to_node < 0 || owner_leave_to_node >= MAX_NODES)
    {
        return 0;
    }

    sw_mode = SERVICE_WAIT_GOING_WAIT_NODE;

    sw_owner_car_id = owner_car_id;
    sw_real_dest_node = real_dest_node;
    sw_service_node = service_node;
    sw_wait_node = wait_node;
    sw_owner_leave_to_node = owner_leave_to_node;

    return 1;
}

uint8_t ServiceWait_IsActive(void)
{
    return (sw_mode != SERVICE_WAIT_IDLE) ? 1 : 0;
}

uint8_t ServiceWait_IsGoingToWaitNode(void)
{
    return (sw_mode == SERVICE_WAIT_GOING_WAIT_NODE) ? 1 : 0;
}

uint8_t ServiceWait_IsAtWaitNode(void)
{
    return (sw_mode == SERVICE_WAIT_AT_WAIT_NODE) ? 1 : 0;
}

void ServiceWait_MarkArrivedWaitNode(void)
{
    if(sw_mode == SERVICE_WAIT_GOING_WAIT_NODE)
    {
        sw_mode = SERVICE_WAIT_AT_WAIT_NODE;
    }
}

uint8_t ServiceWait_CheckOwnerLeftEntry(int8_t car_id,
                                        int8_t start_node,
                                        int8_t end_node)
{
    if(sw_mode != SERVICE_WAIT_AT_WAIT_NODE)
    {
        return 0;
    }

    if(car_id != sw_owner_car_id)
    {
        return 0;
    }

    /*
     * 核心判断：
     * 前车从 service_node 离开，并且方向等于它之前公布的 leave_to_node。
     *
     * 例：
     * 前车在 21 服务，service_node=1，leave_to_node=0；
     * 后车在 4 等待；
     * 收到 type2: start_node=1,end_node=0；
     * 说明前车已经离开 1，后车可以从 4 -> 1 -> 21。
     */
    if(start_node == sw_service_node &&
       end_node == sw_owner_leave_to_node)
    {
        return 1;
    }

    return 0;
}

int8_t ServiceWait_GetMode(void){
    return (int8_t)sw_mode;
}

int8_t ServiceWait_GetOwnerCarId(void){
    return sw_owner_car_id;
}

int8_t ServiceWait_GetRealDestNode(void)
{
    return sw_real_dest_node;
}

int8_t ServiceWait_GetServiceNodeValue(void)
{
    return sw_service_node;
}

int8_t ServiceWait_GetWaitNodeValue(void)
{
    return sw_wait_node;
}

int8_t ServiceWait_GetOwnerLeaveToNode(void)
{
    return sw_owner_leave_to_node;
}