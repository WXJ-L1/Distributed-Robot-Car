//
// Created by 10663 on 26-5-22.
//
#include "service_wait.h"
#include "map.h"

typedef struct
{
    int8_t dest_node;
    int8_t service_node;
    int8_t left_wait_node;
    int8_t right_wait_node;
} ServiceRule_t;

static const ServiceRule_t service_rules[] = {
    {21, 1, 0, 4},
    {24, 4, 1, 2},
    {22, 2, 4, 5},
    {25, 5, 2, 3},
    {23, 3, 5, -1}
};

#define SERVICE_RULE_COUNT  ((uint8_t)(sizeof(service_rules) / sizeof(service_rules[0])))

static const int8_t main_line_nodes[] = {
    0, 1, 4, 2, 5, 3
};

#define MAIN_LINE_COUNT  ((uint8_t)(sizeof(main_line_nodes) / sizeof(main_line_nodes[0])))

static uint8_t sw_mode = SERVICE_WAIT_IDLE;
static int8_t sw_owner_car_id = -1;
static int8_t sw_real_dest_node = -1;
static int8_t sw_service_node = -1;
static int8_t sw_wait_node = -1;
static int8_t sw_owner_leave_to_node = -1;

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

static int8_t ServiceWait_ConvertToMainNode(int8_t node){
    if(node == 41 || node == 42 || node == 43){
        return 0;
    }

    if(ServiceWait_FindMainLineIndex(node) >= 0){
        return node;
    }

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

    if(leave_to_node == service_rules[idx].left_wait_node){
        return service_rules[idx].right_wait_node;
    }
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
    if(current_index < 0 || next_index < 0){
        return -1;
    }

    if(next_index < current_index){
        return service_rules[idx].left_wait_node;
    }
    if(next_index > current_index){
        return service_rules[idx].right_wait_node;
    }

    return -1;
}

uint8_t ServiceWait_Start(int8_t owner_car_id,
                          int8_t real_dest_node,
                          int8_t service_node,
                          int8_t wait_node,
                          int8_t owner_leave_to_node){
    if(owner_car_id < 0){
        return 0;
    }
    if(!ServiceWait_IsDestSupported(real_dest_node)){
        return 0;
    }
    if(service_node < 0 || service_node >= MAX_NODES){
        return 0;
    }
    if(wait_node < 0 || wait_node >= MAX_NODES){
        return 0;
    }
    if(owner_leave_to_node < 0 || owner_leave_to_node >= MAX_NODES){
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

uint8_t ServiceWait_WaitLeavePlan(int8_t owner_car_id,
                                  int8_t real_dest_node,
                                  int8_t service_node){
    if(owner_car_id < 0){
        return 0;
    }
    if(!ServiceWait_IsDestSupported(real_dest_node)){
        return 0;
    }
    if(service_node < 0 || service_node >= MAX_NODES){
        return 0;
    }
    if(sw_mode == SERVICE_WAIT_GOING_WAIT_NODE ||
       sw_mode == SERVICE_WAIT_AT_WAIT_NODE){
        return 0;
    }

    sw_mode = SERVICE_WAIT_WAIT_LEAVE_PLAN;
    sw_owner_car_id = owner_car_id;
    sw_real_dest_node = real_dest_node;
    sw_service_node = service_node;
    sw_wait_node = -1;
    sw_owner_leave_to_node = -1;

    return 1;
}

uint8_t ServiceWait_IsActive(void){
    return (sw_mode != SERVICE_WAIT_IDLE) ? 1 : 0;
}

uint8_t ServiceWait_IsGoingToWaitNode(void){
    return (sw_mode == SERVICE_WAIT_GOING_WAIT_NODE) ? 1 : 0;
}

uint8_t ServiceWait_IsAtWaitNode(void){
    return (sw_mode == SERVICE_WAIT_AT_WAIT_NODE) ? 1 : 0;
}

uint8_t ServiceWait_IsWaitingLeavePlan(void){
    return (sw_mode == SERVICE_WAIT_WAIT_LEAVE_PLAN) ? 1 : 0;
}

void ServiceWait_MarkArrivedWaitNode(void){
    if(sw_mode == SERVICE_WAIT_GOING_WAIT_NODE){
        sw_mode = SERVICE_WAIT_AT_WAIT_NODE;
    }
}

uint8_t ServiceWait_CheckOwnerLeftEntry(int8_t car_id,
                                        int8_t start_node,
                                        int8_t end_node){
    if(sw_mode != SERVICE_WAIT_AT_WAIT_NODE){
        return 0;
    }
    if(car_id != sw_owner_car_id){
        return 0;
    }

    if(start_node == sw_service_node &&
       end_node == sw_owner_leave_to_node){
        return 1;
    }

    // Tolerate one missed edge report: if we already see owner departing
    // from leave_to_node, entrance is considered released.
    if(start_node == sw_owner_leave_to_node){
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

int8_t ServiceWait_GetRealDestNode(void){
    return sw_real_dest_node;
}

int8_t ServiceWait_GetServiceNodeValue(void){
    return sw_service_node;
}

int8_t ServiceWait_GetWaitNodeValue(void){
    return sw_wait_node;
}

int8_t ServiceWait_GetOwnerLeaveToNode(void){
    return sw_owner_leave_to_node;
}

uint8_t ServiceWait_IsSameOwnerAndDest(int8_t owner_car_id,
                                       int8_t dest_node){
    if(sw_mode == SERVICE_WAIT_IDLE){
        return 0;
    }
    if(sw_owner_car_id != owner_car_id){
        return 0;
    }
    if(sw_real_dest_node != dest_node){
        return 0;
    }

    return 1;
}
