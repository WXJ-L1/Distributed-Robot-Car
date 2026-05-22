//
// Created by zzy on 2026/5/7.
//

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "broadcast.h"
#include "buzzer.h"
#include "car_queue.h"
#include "cmsis_os2.h"
#include "map.h"
#include "MQTT.h"
#include "stm32f4xx_hal.h"
#include "UART.h"

extern uint8_t ESP8266_UDP_SendString(char *str);
extern uint8_t ESP8266_UDP_Send(char *data, uint16_t len);
extern void Calculate_Relative_Turn_Angles(void);

/*
 * 小车工作状态变量
 * 0: 空闲
 * 1: 配送排队中
 * 2: 已完成(到站倒计时待命)
 * 3: 返航排队中
 * 4: 配送行驶中
 * 5: 回去行驶中
 */
volatile uint8_t work_state = 0;

int8_t car_current_node = BIRTH_NODE;
int8_t car_final_node = BIRTH_NODE;
int8_t current_start_node = BIRTH_NODE;
int8_t current_end_node = -1;

uint8_t Check_And_Broadcast_Task_a = 0;
uint32_t last_type5_send_tick = 0;
uint8_t type5_retry_count = 0;

#define PATH_RETRY_INTERVAL_MS      1000
#define PATH_APPLY_WAIT_MS          1000
#define ROAD_SYNC_WAIT_MS           300
#define TYPE0_RESEND_INTERVAL_MS    1200
#define TYPE1_SYNC_TIMEOUT_MS       3000

static uint32_t last_type0_broadcast_tick = 0;
static uint32_t last_path_retry_tick = 0;
static uint8_t is_applying_path = 0;
static uint8_t pending_path_apply = 0;
static uint32_t pending_path_apply_tick = 0;
static uint8_t waiting_type6 = 0;
static uint32_t last_type1_sync_tick = 0;
static uint8_t road_sync_required = 0;
static uint32_t road_sync_required_tick = 0;
static uint8_t other_car_running = 0;
static int other_running_car_id = -1;
static uint8_t other_running_path_synced = 0;
static uint8_t type0_announced_ok = 0;

uint8_t Broadcast_Type0_State(void);
uint8_t Broadcast_Type1_Road(void);
void Broadcast_Debug_Angles(void);
void Broadcast_Type2_Position(void);
void Try_Apply_Path(void);

static void Enter_Path_Wait_Window(const char *reason){
    pending_path_apply = 1;
    pending_path_apply_tick = HAL_GetTick();
    last_path_retry_tick = HAL_GetTick();
    if(reason != NULL){
        UART1_SendString(reason);
        UART1_SendString("\r\n");
    }
}

static uint8_t Broadcast_Type0_Running(uint8_t running_state){
    char type0_buffer[96];
    snprintf(type0_buffer, sizeof(type0_buffer),
             "{\"type\":0,\"carId\":%d,\"work\":%d}",
             MY_CAR_ID,
             running_state);
    UART1_SendString("[Send Type0 Running] ");
    UART1_SendString(type0_buffer);
    UART1_SendString("\r\n");
    if(MQTT_Send("car/broadcast", type0_buffer)){
        UART1_SendString("[MQTT Type0 Running SEND OK]\r\n");
        return 1;
    }
    else{
        UART1_SendString("[MQTT Type0 Running SEND FAIL]\r\n");
        return 0;
    }
}

static uint8_t Broadcast_Type6_Token(void){
    char type6_buffer[64];
    snprintf(type6_buffer, sizeof(type6_buffer),
             "{\"type\":6,\"carId\":%d}",
             MY_CAR_ID);
    UART1_SendString("[Send Type6 Token] ");
    UART1_SendString(type6_buffer);
    UART1_SendString("\r\n");
    if(MQTT_Send("car/broadcast", type6_buffer)){
        UART1_SendString("[MQTT Type6 SEND OK]\r\n");
        return 1;
    }
    else{
        UART1_SendString("[MQTT Type6 SEND FAIL]\r\n");
        return 0;
    }
}

/**
 * @brief 检查状态并在已完成时启动倒计时待命
 */
void Check_And_Broadcast_Task(void){
    char json_buffer[128];
    static uint32_t arrive_time_tick = 0;
    static uint8_t is_arrived_reported = 0;
    static uint8_t is_home_reported = 0;
    if(work_state == 2 && Check_And_Broadcast_Task_a == 0){
        Check_And_Broadcast_Task_a = 1;
        arrive_time_tick = HAL_GetTick();
        is_arrived_reported = 0;
    }
    else if(work_state == 2 && Check_And_Broadcast_Task_a == 1){
        if(is_arrived_reported == 0){
            is_arrived_reported = 1;
            snprintf(json_buffer, sizeof(json_buffer),
                     "{\"type\":7,\"carId\":%d,\"state\":2,\"node\":%d}",
                     MY_CAR_ID,
                     car_current_node);
            UART1_SendString("[UDP Send Delivery Done] ");
            UART1_SendString(json_buffer);
            UART1_SendString("\r\n");
            ESP8266_UDP_Send(json_buffer, strlen(json_buffer));
        }
        if(HAL_GetTick() - arrive_time_tick >= 5000){
            UART1_SendString("[System] 5s Timeout! No command received, Buzzer Alarm & returning automatically...\r\n");
            Buzzer_On();
            osDelay(100);
            Buzzer_Off();
            car_final_node = BIRTH_NODE;
            work_state = 3;
            Check_And_Broadcast_Task_a = 0;
            waiting_type6 = 0;
            road_sync_required = 0;
            road_sync_required_tick = 0;
            last_type1_sync_tick = 0;
            type0_announced_ok = 0;
            last_type0_broadcast_tick = 0;
            Enter_Path_Wait_Window("[Path] Auto return accepted, wait for queue sync before applying");
            Broadcast_Type0_State();
        }
    }
    else if(work_state == 0){
        if(is_home_reported == 0){
            is_home_reported = 1;
            snprintf(json_buffer, sizeof(json_buffer),
                     "{\"type\":7,\"carId\":%d,\"state\":0,\"node\":%d}",
                     MY_CAR_ID,
                     BIRTH_NODE);
            UART1_SendString("[UDP Send Return Done] ");
            UART1_SendString(json_buffer);
            UART1_SendString("\r\n");
            ESP8266_UDP_Send(json_buffer, strlen(json_buffer));
            Check_And_Broadcast_Task_a = 0;
            last_type5_send_tick = 0;
            type5_retry_count = 0;
            last_path_retry_tick = 0;
            last_type0_broadcast_tick = 0;
            pending_path_apply = 0;
            waiting_type6 = 0;
            road_sync_required = 0;
            road_sync_required_tick = 0;
            last_type1_sync_tick = 0;
            type0_announced_ok = 0;
            AcceptUpdate = 0;
            Car_Queue_Update(MY_CAR_ID, 0);
            UART1_SendString("[System] Back Home. All variables fully reset. Ready for next loop!\r\n");
        }
    }

    else if(work_state == 1 || work_state == 3){
        uint32_t now = HAL_GetTick();
        is_home_reported = 0;
        if(type0_announced_ok == 0 && now - last_type0_broadcast_tick >= TYPE0_RESEND_INTERVAL_MS){
            last_type0_broadcast_tick = now;
            Broadcast_Type0_State();
        }
        if(pending_path_apply){
            if(now - pending_path_apply_tick < PATH_APPLY_WAIT_MS){
                return;
            }
            if(last_type1_sync_tick != 0 &&
               now - last_type1_sync_tick < ROAD_SYNC_WAIT_MS){
                return;
            }
            if(road_sync_required){
                if(now - road_sync_required_tick < TYPE1_SYNC_TIMEOUT_MS){
                    UART1_SendString("[Path] Wait type1 road sync before applying\r\n");
                    if(type0_announced_ok == 0 &&
                       now - last_type0_broadcast_tick >= TYPE0_RESEND_INTERVAL_MS){
                        last_type0_broadcast_tick = now;
                        Broadcast_Type0_State();
                    }
                    return;
                }
                UART1_SendString("[Path] Type1 sync timeout, try with current Type2 map\r\n");
                road_sync_required = 0;
                road_sync_required_tick = 0;
                other_running_path_synced = 1;
            }
            if(type0_announced_ok == 0){
                UART1_SendString("[Path] Type0 not sent OK, resend and wait again\r\n");
                Broadcast_Type0_State();
                pending_path_apply_tick = HAL_GetTick();
                last_path_retry_tick = HAL_GetTick();
                return;
            }
            pending_path_apply = 0;
            if(Get_Head_CarId() == MY_CAR_ID){
                UART1_SendString("[Path] Queue sync done, I am head, apply path\r\n");
                Try_Apply_Path();
            }
            else{
                waiting_type6 = 1;
                UART1_SendString("[Path] Queue sync done, I am not head, wait type6\r\n");
            }
            return;
        }
        if(waiting_type6){
            return;
        }
        if(Get_Head_CarId() == MY_CAR_ID){
            if(now - last_path_retry_tick >= PATH_RETRY_INTERVAL_MS){
                last_path_retry_tick = now;
                if(type0_announced_ok == 0){
                    UART1_SendString("[Path] Periodic retry blocked: Type0 not announced, resend Type0\r\n");
                    Broadcast_Type0_State();
                    return;
                }
                UART1_SendString("[Path] Periodic retry apply path.\r\n");
                Try_Apply_Path();
            }
        }
    }
    else if(work_state == 4 || work_state == 5){
        is_home_reported = 0;
    }
}

/**
 * @brief 广播 type0 信息：小车排队与当前 task 状态
 */
uint8_t Broadcast_Type0_State(void){
    char json_buffer[128];
    int8_t head_id;
    if(work_state != 1 && work_state != 3){
        return 0;
    }
    Car_Queue_Update(MY_CAR_ID, work_state);
    head_id = Get_Head_CarId();
    snprintf(json_buffer, sizeof(json_buffer),
             "{\"type\":0,\"carId\":%d,\"work\":%d,\"head_carId\":%d}",
             MY_CAR_ID,
             work_state,
             head_id);
    UART1_SendString("[Send Type0] ");
    UART1_SendString(json_buffer);
    UART1_SendString("\r\n");
    if(MQTT_Send("car/broadcast", json_buffer)){
        type0_announced_ok = 1;
        UART1_SendString("[MQTT Type0 SEND OK]\r\n");
        return 1;
    }
    else{
        type0_announced_ok = 0;
        UART1_SendString("[MQTT Type0 SEND FAIL]\r\n");
        return 0;
    }
}

/**
 * @brief 广播 type1 信息：申请路径成功后，将路径广播给其他小车
 */
uint8_t Broadcast_Type1_Road(void){
    char json_buffer[512];
    char road_buf[256];
    uint16_t offset = 0;
    memset(json_buffer, 0, sizeof(json_buffer));
    memset(road_buf, 0, sizeof(road_buf));
    offset += snprintf(&road_buf[offset], sizeof(road_buf) - offset, "[");
    for(int8_t i = 0; i < car_road_len; i++){
        if(offset >= sizeof(road_buf) - 8){
            break;
        }
        offset += snprintf(&road_buf[offset],
                           sizeof(road_buf) - offset,
                           "%d",
                           car_road[i]);
        if(i < car_road_len - 1)
        {
            offset += snprintf(&road_buf[offset],
                               sizeof(road_buf) - offset,
                               ",");
        }
    }
    snprintf(&road_buf[offset], sizeof(road_buf) - offset, "]");
    snprintf(json_buffer, sizeof(json_buffer),
             "{\"type\":1,\"carId\":%d,\"road\":%s}",
             MY_CAR_ID,
             road_buf);
    UART1_SendString("[Send Type1 Road] ");
    UART1_SendString(json_buffer);
    UART1_SendString("\r\n");
    if(MQTT_Send("car/broadcast", json_buffer)){
        UART1_SendString("[MQTT Type1 SEND OK]\r\n");
        return 1;
    }
    else{
        UART1_SendString("[MQTT Type1 SEND FAIL]\r\n");
        return 0;
    }
}

/**
 * @brief 调试用：通过 MQTT 打印当前路线和角度数组
 */
void Broadcast_Debug_Angles(void){
    char json_buffer[512];
    char road_buf[160];
    char angle_buf[160];
    uint16_t offset;
    int i;
    memset(json_buffer, 0, sizeof(json_buffer));
    memset(road_buf, 0, sizeof(road_buf));
    memset(angle_buf, 0, sizeof(angle_buf));
    offset = 0;
    offset += snprintf(&road_buf[offset], sizeof(road_buf) - offset, "[");
    for(i = 0; i < car_road_len; i++){
        if(offset >= sizeof(road_buf) - 8)
        {
            break;
        }
        offset += snprintf(&road_buf[offset],
                           sizeof(road_buf) - offset,
                           "%d",
                           car_road[i]);
        if(i < car_road_len - 1){
            offset += snprintf(&road_buf[offset],
                               sizeof(road_buf) - offset,
                               ",");
        }
    }
    snprintf(&road_buf[offset], sizeof(road_buf) - offset, "]");
    offset = 0;
    offset += snprintf(&angle_buf[offset], sizeof(angle_buf) - offset, "[");
    for(i = 0; i < relative_turn_angles_len; i++){
        if(offset >= sizeof(angle_buf) - 12)
        {
            break;
        }
        offset += snprintf(&angle_buf[offset],
                           sizeof(angle_buf) - offset,
                           "%d",
                           relative_turn_angles[i]);

        if(i < relative_turn_angles_len - 1){
            offset += snprintf(&angle_buf[offset],
                               sizeof(angle_buf) - offset,
                               ",");
        }
    }
    snprintf(&angle_buf[offset], sizeof(angle_buf) - offset, "]");
    snprintf(json_buffer,
             sizeof(json_buffer),
             "{\"type\":8,\"carId\":%d,\"road_len\":%d,\"angle_len\":%d,\"road\":%s,\"angles\":%s,\"last_node\":%d}",
             MY_CAR_ID,
             car_road_len,
             relative_turn_angles_len,
             road_buf,
             angle_buf,
             last_visited_node);
    UART1_SendString("[Send Type8 Debug Angles] ");
    UART1_SendString(json_buffer);
    UART1_SendString("\r\n");
    MQTT_Send("car/broadcast", json_buffer);
}

/**
 * @brief 广播 type2 信息：自动广播小车自己的位置
 */
void Broadcast_Type2_Position(void){
    char json_buffer[128];
    if(work_state != 4 && work_state != 5){
        return;
    }
    // if(current_end_node == -1){
    //     return;
    // }
    snprintf(json_buffer, sizeof(json_buffer),
             "{\"type\":2,\"carId\":%d,\"start_node\":%d,\"end_node\":%d}",
             MY_CAR_ID,
             current_start_node,
             current_end_node);
    UART1_SendString("[Send Type2] ");
    UART1_SendString(json_buffer);
    UART1_SendString("\r\n");
    if(ESP8266_UDP_Send(json_buffer, strlen(json_buffer))){
        UART1_SendString("[UDP Type2 SEND OK]\r\n");
    }
    else{
        UART1_SendString("[UDP Type2 SEND FAIL]\r\n");
    }
    if(MQTT_Send("car/broadcast", json_buffer)){
        UART1_SendString("[MQTT Type2 SEND OK]\r\n");
    }
    else{
        UART1_SendString("[MQTT Type2 SEND FAIL]\r\n");
    }
}

void Broadcast_Type4_OnlineStatus(uint8_t online_status){
    char json_buffer[128];
    int8_t current_battery = 80;
    snprintf(json_buffer, sizeof(json_buffer),
             "{\"type\":4,\"carId\":%d,\"online\":%d,\"battery\":%d}",
             MY_CAR_ID,
             online_status,
             current_battery);
    ESP8266_UDP_SendString(json_buffer);
}

/**
 * @brief 核心路径申请与防死锁调度逻辑
 */
void Try_Apply_Path(void){
    int8_t start_node = car_current_node;
    int8_t end_node = -1;
    uint8_t path_found = 0;
    uint8_t next_work_state = 0;
    uint32_t now = HAL_GetTick();
    if(pending_path_apply){
        if(now - pending_path_apply_tick < PATH_APPLY_WAIT_MS){
            UART1_SendString("[Path] Block Try_Apply_Path: queue sync waiting\r\n");
            return;
        }
        if(last_type1_sync_tick != 0 &&
           now - last_type1_sync_tick < ROAD_SYNC_WAIT_MS){
            UART1_SendString("[Path] Block Try_Apply_Path: road sync waiting\r\n");
            return;
        }
    }
    if(road_sync_required){
        UART1_SendString("[Path] Block Try_Apply_Path: wait type1 road\r\n");
        return;
    }

    if(type0_announced_ok == 0){
        UART1_SendString("[Path] Block Try_Apply_Path: Type0 not announced by MQTT\r\n");
        return;
    }

    if(is_applying_path){
        UART1_SendString("[Path] Ignore duplicate Try_Apply_Path\r\n");
        return;
    }
    is_applying_path = 1;

    if(work_state != 1 && work_state != 3){
        char state_err[64];
        snprintf(state_err, sizeof(state_err),
                 "[Path] Invalid work_state: %d, abort path planning\r\n",
                 work_state);
        UART1_SendString(state_err);
        goto exit_apply;
    }

    if(work_state == 1){
        end_node = car_final_node;
        next_work_state = 4;
    }
    else if(work_state == 3){
        end_node = BIRTH_NODE;
        next_work_state = 5;
    }
    if(end_node < 0 || end_node >= MAX_NODES){
        UART1_SendString("[Path] Invalid end_node, abort path planning\r\n");
        goto exit_apply;
    }
    path_found = Get_Path_Algorithm(start_node, end_node);
    if(path_found == 1){
        UART1_SendString("[Info] 4.2 Path Found! Configuring routes...\r\n");
        last_path_retry_tick = 0;
        Calculate_Relative_Turn_Angles();
        Broadcast_Debug_Angles();
        if(!Broadcast_Type1_Road()){
            UART1_SendString("[Path] Type1 send failed, keep waiting, do not drive\r\n");
            Enter_Path_Wait_Window("[Path] Re-enter wait window because Type1 failed");
            goto exit_apply;
        }

        if(!Broadcast_Type0_Running(next_work_state)){
            UART1_SendString("[Path] Type0 running send failed, keep waiting, do not drive\r\n");
            Enter_Path_Wait_Window("[Path] Re-enter wait window because running type0 failed");
            goto exit_apply;
        }
        if(!Broadcast_Type6_Token()){
            UART1_SendString("[Path] Type6 send failed, but Type0 running was sent. Continue driving.\r\n");
        }

        work_state = next_work_state;
        Car_Queue_Update(MY_CAR_ID, 0);
        AcceptUpdate = 0;
        pending_path_apply = 0;
        waiting_type6 = 0;
        road_sync_required = 0;
        road_sync_required_tick = 0;
        last_type1_sync_tick = 0;
        type0_announced_ok = 0;
        UART1_SendString("[Path] MQTT protocol complete. State -> Running\r\n");
        goto exit_apply;
    }
    else
    {
        UART1_SendString("[Info] 4.3 Path NOT Found! Check next car...\r\n");

        /*
         * 申请失败：
         * 1. 不改变 work_state，任务不丢；
         * 2. 不退出队列，自己仍然在队列里；
         * 3. 如果队列里有下一辆车，才发送 type6；
         * 4. 如果队列里只有自己，不发 type6，延迟后自己重试。
         */
        Car_Queue_Update(MY_CAR_ID, work_state);

        last_path_retry_tick = HAL_GetTick();

        int next_id = Get_Next_CarId_In_Queue(MY_CAR_ID);

        char fail_buf[96];
        snprintf(fail_buf, sizeof(fail_buf),
                 "[Path] Apply failed. My next car is:%d\r\n",
                 next_id);
        UART1_SendString(fail_buf);

        if(next_id != -1 && next_id != MY_CAR_ID)
        {
            if(Broadcast_Type6_Token())
            {
                waiting_type6 = 1;
                pending_path_apply = 0;
                UART1_SendString("[Path] Yield this round, wait next Type6\r\n");
            }
            else
            {
                waiting_type6 = 0;
                UART1_SendString("[Path] Type6 send failed, keep myself retrying\r\n");
            }
        }
        else
        {
            /*
             * 队列里只有自己，不发 type6。
             */
            waiting_type6 = 0;
            Enter_Path_Wait_Window("[Path] Only me in queue, retry later");
        }

        goto exit_apply;
    }
    exit_apply:
    is_applying_path = 0;
}

/**
 * @brief 解析上位机 UDP 发来的任务消息
 */
void Parse_Broadcast_Message(const char *json_str){
    int type = -1;
    int car_id = -1;
    int final_node = -1;
    char debug_buf[128];
    if(json_str == NULL){
        return;
    }
    const char *type_ptr = strstr(json_str, "\"type\":");
    if(type_ptr == NULL){
        UART1_SendString("[UDP Parse] Invalid message: no type\r\n");
        return;
    }
    sscanf(type_ptr, "\"type\":%d", &type);
    if(type != 5){
        UART1_SendString("[UDP Parse] Ignore non-type5 message\r\n");
        return;
    }
    const char *id_ptr = strstr(json_str, "\"carId\":");
    if(id_ptr != NULL){
        sscanf(id_ptr, "\"carId\":%d", &car_id);
    }
    if(car_id != MY_CAR_ID){
        snprintf(debug_buf, sizeof(debug_buf),
                 "[UDP Parse] Ignore Type5. target:%d my:%d\r\n",
                 car_id,
                 MY_CAR_ID);
        UART1_SendString(debug_buf);
        return;
    }
    const char *fn_ptr = strstr(json_str, "\"final_node\":");
    if(fn_ptr != NULL){
        sscanf(fn_ptr, "\"final_node\":%d", &final_node);
    }
    snprintf(debug_buf, sizeof(debug_buf),
             "[UDP Parse] Accept Type5 -> carId:%d final_node:%d\r\n",
             car_id,
             final_node);
    UART1_SendString(debug_buf);
    if(final_node < 0 || final_node >= MAX_NODES){
        snprintf(debug_buf, sizeof(debug_buf),
                 "[UDP Parse] Invalid final_node:%d\r\n",
                 final_node);
        UART1_SendString(debug_buf);
        return;
    }
    /*
     * 只要不是实际行驶中，就允许新命令覆盖。
     */
    if(work_state != 4 && work_state != 5){
        if(work_state == 1 || work_state == 3){
            Car_Queue_Update(MY_CAR_ID, 0);
            AcceptUpdate = 0;
        }
        car_final_node = final_node;
        Check_And_Broadcast_Task_a = 0;
        waiting_type6 = 0;
        pending_path_apply = 0;
        road_sync_required = 0;
        road_sync_required_tick = 0;
        last_type1_sync_tick = 0;
        type0_announced_ok = 0;
        last_type0_broadcast_tick = 0;
        last_path_retry_tick = HAL_GetTick();
        if(final_node == BIRTH_NODE){
            UART1_SendString("[UDP Parse] Action: Manual Call back to BIRTH_NODE\r\n");
            work_state = 3;
        }
        else{
            UART1_SendString("[UDP Parse] Action: Heading to a NEW destination node\r\n");
            work_state = 1;
        }
        /*
         * 如果收到新任务时，已经知道有别的车正在跑，
         * 但还没有同步到它的 type1 road，就必须等待 type1。
         */
        if(other_car_running && other_running_path_synced == 0){
            road_sync_required = 1;
            road_sync_required_tick = HAL_GetTick();
            UART1_SendString("[Path] New task while other car running, wait type1 road\r\n");
        }
        Enter_Path_Wait_Window("[Path] Type5 accepted, wait for queue sync before applying");
        Broadcast_Type0_State();
    }
    else{
        snprintf(debug_buf, sizeof(debug_buf),
                 "[UDP Parse] Type5 ignored, car is running! work_state:%d\r\n",
                 work_state);
        UART1_SendString(debug_buf);
    }
}

/**
 * @brief 解析 MQTT 收到的小车间广播消息
 */
void Parse_MQTT_Broadcast_Message(const char *json_str){
    int type = -1;
    int car_id = -1;
    char debug_buf[128];
    if(json_str == NULL){
        return;
    }
    const char *type_ptr = strstr(json_str, "\"type\":");
    if(type_ptr == NULL){
        UART1_SendString("[MQTT Parse] Invalid message: no type\r\n");
        return;
    }
    sscanf(type_ptr, "\"type\":%d", &type);
    const char *id_ptr = strstr(json_str, "\"carId\":");
    if(id_ptr != NULL){
        sscanf(id_ptr, "\"carId\":%d", &car_id);
    }
    snprintf(debug_buf, sizeof(debug_buf),
             "[MQTT Parse] type:%d carId:%d\r\n",
             type,
             car_id);
    UART1_SendString(debug_buf);

    if(type == 0){
        int work = -1;
        const char *work_ptr = strstr(json_str, "\"work\":");
        if(work_ptr != NULL){
            sscanf(work_ptr, "\"work\":%d", &work);
        }

        snprintf(debug_buf, sizeof(debug_buf),
                 "[MQTT Parse] Type0 -> carId:%d work:%d\r\n",
                 car_id,
                 work);
        UART1_SendString(debug_buf);
        if(car_id == MY_CAR_ID){
            UART1_SendString("[MQTT Parse] Ignore my own Type0\r\n");
            return;
        }
        if(work == 1 || work == 3){
            uint8_t updated = Car_Queue_Update(car_id, work);
            if(updated){
                AcceptUpdate = 1;
                if(work_state == 1 || work_state == 3){
                    Enter_Path_Wait_Window("[MQTT Parse] Type0 really updated queue, delay apply");
                }
            }
            else{
                Car_Queue_Print();
            }
            if(work_state == 4 || work_state == 5){
                UART1_SendString("[MQTT Parse] Other car waiting, send Type2 position only\r\n");
                Broadcast_Type2_Position();
            }
        }

        else if(work == 0){
            Car_Queue_Update(car_id, 0);
            AcceptUpdate = 0;
            if(car_id == other_running_car_id){
                other_car_running = 0;
                other_running_car_id = -1;
                other_running_path_synced = 0;
                UART1_SendString("[MQTT Parse] Other running car cleared\r\n");
            }
            if((work_state == 1 || work_state == 3) &&
               Get_Head_CarId() == MY_CAR_ID){
                waiting_type6 = 0;
                Enter_Path_Wait_Window("[MQTT Parse] Other car exit queue, I may apply");
            }
        }

        else if(work == 4 || work == 5){
            UART1_SendString("[MQTT Parse] Other car running, remember running car and remove from queue\r\n");
            Car_Queue_Update(car_id, 0);
            Car_Queue_Print();
            other_car_running = 1;
            if(other_running_car_id != car_id){
                other_running_car_id = car_id;
                other_running_path_synced = 0;
            }
            else{
                other_running_car_id = car_id;
            }

            if((work_state == 1 || work_state == 3) &&
               other_running_path_synced == 0){
                road_sync_required = 1;
                road_sync_required_tick = HAL_GetTick();

                Enter_Path_Wait_Window("[MQTT Parse] Other running detected, wait road sync");
            }
        }
        else{
            UART1_SendString("[MQTT Parse] Ignore invalid Type0 work\r\n");
        }
    }

    else if(type == 1){
        UART1_SendString("Type1 work\r\n");
        if(car_id == MY_CAR_ID){
            UART1_SendString("[MQTT Parse] Ignore my own Type1\r\n");
            return;
        }
        const char *road_ptr = strstr(json_str, "\"road\"");
        if(road_ptr != NULL){
            road_ptr = strchr(road_ptr, '[');
        }
        if(road_ptr != NULL){
            UART1_SendString("Type1 work1\r\n");
            int8_t road_nodes[MAX_NODES];
            int8_t road_len = 0;
            memset(road_nodes, -1, sizeof(road_nodes));
            road_ptr++;
            const char *end_ptr = strchr(road_ptr, ']');
            if(end_ptr != NULL && end_ptr > road_ptr){
                char temp_road[256];
                size_t copy_len = (size_t)(end_ptr - road_ptr);
                if(copy_len >= sizeof(temp_road)){
                    copy_len = sizeof(temp_road) - 1;
                }
                strncpy(temp_road, road_ptr, copy_len);
                temp_road[copy_len] = '\0';
                char *token = strtok(temp_road, ",");
                while(token != NULL && road_len < MAX_NODES){
                    road_nodes[road_len++] = (int8_t)atoi(token);
                    token = strtok(NULL, ",");
                }
            }
            snprintf(debug_buf, sizeof(debug_buf),
                     "[MQTT Parse] Type1 -> carId:%d road_len:%d\r\n",
                     car_id,
                     road_len);
            UART1_SendString(debug_buf);
            if(road_len > 0){
                UART1_SendString("Type1 work2\r\n");
                Update_Temp_Map_By_Road(road_nodes, road_len);
                last_type1_sync_tick = HAL_GetTick();
                road_sync_required = 0;
                other_car_running = 1;
                other_running_car_id = car_id;
                other_running_path_synced = 1;
                UART1_SendString("[MQTT Parse] Temp map updated by Type1\r\n");
            }
        }
    }

    else if(type == 2){
        int start_node = -1;
        int end_node = -1;
        const char *start_ptr = strstr(json_str, "\"start_node\":");
        if(start_ptr != NULL){
            sscanf(start_ptr, "\"start_node\":%d", &start_node);
        }

        const char *end_ptr = strstr(json_str, "\"end_node\":");
        if(end_ptr != NULL){
            sscanf(end_ptr, "\"end_node\":%d", &end_node);
        }

        if(car_id != MY_CAR_ID && start_node != -1){
            //调用时再强转
            Update_Temp_Map_By_Position((int8_t)start_node, (int8_t)end_node);;
            if(work_state == 1 || work_state == 3){
                road_sync_required = 0;
                road_sync_required_tick = 0;
                other_car_running = 1;
                other_running_car_id = car_id;
                other_running_path_synced = 1;
                last_type1_sync_tick = HAL_GetTick();
            }
            snprintf(debug_buf, sizeof(debug_buf),
                     "[MQTT Parse] Type2 -> car:%d pos:%d->%d\r\n",
                     car_id,
                     start_node,
                     end_node);
            UART1_SendString(debug_buf);
        }
        else if(car_id == MY_CAR_ID){
            UART1_SendString("[MQTT Parse] Ignore my own Type2\r\n");
        }
    }
    else if(type == 4){
        int online = -1;
        int battery = -1;
        const char *online_ptr = strstr(json_str, "\"online\":");
        if(online_ptr != NULL){
            sscanf(online_ptr, "\"online\":%d", &online);
        }
        const char *battery_ptr = strstr(json_str, "\"battery\":");
        if(battery_ptr != NULL){
            sscanf(battery_ptr, "\"battery\":%d", &battery);
        }
        snprintf(debug_buf, sizeof(debug_buf),
                 "[MQTT Parse] Type4 -> carId:%d online:%d battery:%d\r\n",
                 car_id,
                 online,
                 battery);
        UART1_SendString(debug_buf);
    }

    else if(type == 6){
        snprintf(debug_buf, sizeof(debug_buf),
                 "[MQTT Parse] Type6 -> from carId:%d\r\n",
                 car_id);
        UART1_SendString(debug_buf);
        if(car_id == MY_CAR_ID){
            UART1_SendString("[MQTT Parse] Ignore my own Type6\r\n");
            return;
        }

        if(work_state != 1 && work_state != 3){
            UART1_SendString("[MQTT Parse] Type6 received, but I am not waiting\r\n");
            return;
        }

        if(Is_Next_Car((int8_t)car_id, MY_CAR_ID)){
            AcceptUpdate = 0;
            waiting_type6 = 0;
            pending_path_apply = 0;
            UART1_SendString("[MQTT Parse] Type6: I am next car, try apply path\r\n");

            Try_Apply_Path();
            return;
        }
        if(Get_Head_CarId() == MY_CAR_ID){
            AcceptUpdate = 0;
            waiting_type6 = 0;
            UART1_SendString("[MQTT Parse] Type6: I am queue head, apply later\r\n");
            Enter_Path_Wait_Window("[MQTT Parse] Type6 head apply");
            return;
        }
        AcceptUpdate = 0;
        waiting_type6 = 1;
        pending_path_apply = 0;
        UART1_SendString("[MQTT Parse] Type6: not my turn, keep waiting\r\n");
    }
    else if(type == 5){
        UART1_SendString("[MQTT Parse] Ignore Type5, task command should come from UDP\r\n");
    }
    else{
        snprintf(debug_buf, sizeof(debug_buf),
                 "[MQTT Parse] Unhandled type:%d carId:%d\r\n",
                 type,
                 car_id);
        UART1_SendString(debug_buf);
    }
}