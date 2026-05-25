//
// Created wxj on 26-4-8.
//

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "Motor.h"
#include "FreeRTOS.h"
#include "UART.h"
#include "oled.h"
#include "MQTT.h"
#include "mpu6050_turn.h"
#include "buzzer.h"
#include "turning.h"
#include "map.h"
#include "broadcast.h"
#include "service_wait.h"
//**********************************************
#define MQTT_TOPIC  "car/broadcast"

uint8_t system_ready = 0;
char global_target[512] = "a";

extern osMessageQueueId_t getQueueHandle;   // 用于UDP发送队列
extern osMessageQueueId_t cmdQueueHandle;   // UDP type5
extern osMessageQueueId_t mqttQueueHandle;  // MQTT type0/type1/type2/type4/type6
extern volatile uint8_t esp_is_busy;

// 引入 turning.h 里的自动执行函数声明
extern void Execute_Auto_Driving(int8_t* road, int8_t road_len, int* turn_angles, int8_t turn_angles_len);
// 【声明外部变量】告诉 MAIN.c，这个变量在 broadcast.c 里面定义过了，直接用就行
extern uint8_t Check_And_Broadcast_Task_a;

/**
 * @brief 检查 getQueueHandle 队列并发送 UDP 数据
 * @note  需在任务循环中周期性调用，非阻塞读取队列
 */
void SendFromGetQueueHandle(void)
{
    char *msg_to_send = NULL;
    if (osMessageQueueGet(getQueueHandle, &msg_to_send, NULL, 0) == osOK){
        if (msg_to_send != NULL){
            uint16_t len = strlen(msg_to_send);
            if(ESP8266_UDP_Send(msg_to_send, len)){
            }
            else{
            }
            vPortFree(msg_to_send);
            msg_to_send = NULL;
        }
    }
}

/**
 * @brief  通过 UDP 发送字符串数据（非阻塞/队列模式）
 * @param  str: 要发送的内容
 * @retval 1: 入队成功, 0: 入队失败或内存不足
 */
uint8_t ESP8266_UDP_SendString(char *str){
    if (str == NULL) {
        return 0;
    }
    uint16_t len = strlen(str);
    // 1. 动态分配内存（+1 是为了存放字符串结束符 '\0'）
    char *tx_msg = (char *)pvPortMalloc(len + 1);
    if (tx_msg == NULL) {
        return 0; // 内存分配失败
    }
    // 2. 拷贝数据到动态内存中
    strcpy(tx_msg, str);
    // 3. 将指针放入队列，不等待（0ms超时），如果队列满了则丢弃并释放内存
    if (osMessageQueuePut(getQueueHandle, &tx_msg, 0, 0) != osOK) {
        vPortFree(tx_msg);
        return 0;
    }
    return 1;
}

static void Copy_Clean_String(char *dst, uint16_t dst_size, const char *src){
    int i = 0;
    int j = 0;
    if(dst == NULL || src == NULL || dst_size == 0){
        return;
    }
    memset(dst, 0, dst_size);
    while(src[i] != '\0' && j < (int)(dst_size - 1)){
        if(src[i] != '\r' && src[i] != '\n'){
            dst[j++] = src[i];
        }
        i++;
    }
    dst[j] = '\0';
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  通用 UDP 数据处理任务
  * @details 接收 ESP8266 的 UDP 数据包，并全量存储到 global_target 中
  */
void StartDefaultTask(void *argument){
    char *p_udp_data = NULL;
    char *p_mqtt_data = NULL;
    char udp_target[512];
    char mqtt_target[512];
    OLED_Init();
    WIFI_Init();
    MQTT_Init();
    Map_Init();
    TB6612_Init();
    if (MPU6050_Init() != 0) {
        printf("MPU6050 Init Failed!\r\n");
        Buzzer_On();
        osDelay(100);
        Buzzer_Off();
        osDelay(100);
        Buzzer_On();
        osDelay(100);
        Buzzer_Off();
    } else {
        UART1_SendString("MPU6050 Init Success!\r\n");
        Buzzer_Beep(100);
    }
    osDelay(100);
    system_ready = 1;
    /* USER CODE BEGIN StartDefaultTask */
    for(;;)
    {
        WIFI_ProcessCachedIPD();
        MQTT_ProcessDmaToQueue();
        if(osMessageQueueGet(cmdQueueHandle, &p_udp_data, NULL, 0) == osOK){
            if(p_udp_data != NULL){
                Copy_Clean_String(udp_target,
                                  sizeof(udp_target),
                                  p_udp_data);
                UART1_SendString("[UDP CMD] ");
                UART1_SendString(udp_target);
                UART1_SendString("\r\n");
                Parse_Broadcast_Message(udp_target);
                vPortFree(p_udp_data);
                p_udp_data = NULL;
            }
        }

        if(osMessageQueueGet(mqttQueueHandle, &p_mqtt_data, NULL, 0) == osOK){
            if(p_mqtt_data != NULL){
                Copy_Clean_String(mqtt_target,
                                  sizeof(mqtt_target),
                                  p_mqtt_data);
                // UART1_SendString("[MQTT CLEAN RAW] ");
                // UART1_SendString(mqtt_target);
                // UART1_SendString("\r\n");
                Parse_MQTT_Broadcast_Message(mqtt_target);
                vPortFree(p_mqtt_data);
                p_mqtt_data = NULL;
            }
        }
        osDelay(10);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the servo_hcsr024 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument){
    while(!system_ready) osDelay(200);
    osDelay(100);
    UART1_SendString("[Task06] IN for_loop...\r\n");
    for(;;)
    {
        // 1. 读取当前的 work_state
        // 2. 只有当状态为 4(配送中) 或 5(回去中) 时，才触发物理行驶
        if (work_state == 4 || work_state == 5) {
            UART1_SendString("[Task06] Starting Auto Driving...\r\n");
            // ==========================================
            // 【新增】：打印 relative_turn_angles 数组
            // ==========================================
            char print_buf[64]; // 定义一个字符串缓冲区
            sprintf(print_buf, "[Map] Calculated Angles (Len: %d): ", relative_turn_angles_len);
            UART1_SendString(print_buf);
            // 遍历数组并逐个转换打印
            for (int i = 0; i < relative_turn_angles_len; i++) {
                sprintf(print_buf, "%d ", relative_turn_angles[i]); // 把整数和空格格式化成字符串
                UART1_SendString(print_buf);
            }
            UART1_SendString("\r\n");
            // ==========================================
            // 3. 调用自动行驶函数
            // 注意：这个函数会一直阻塞在这里运行，直到小车到达终点
            Execute_Auto_Driving(car_road, car_road_len, relative_turn_angles, relative_turn_angles_len);
            if(car_road_len > 1){
                last_visited_node = car_road[car_road_len - 2];
            }
            // 4. 物理行驶彻底结束，进行状态结算
            if (work_state == 4) {
                car_current_node = car_final_node;
                current_start_node = car_current_node;
                current_end_node = -1;
                /*
                 * 此时 work_state 还是 4，可以发送最终 type2。
                 */
                Broadcast_Type2_Position();
                if(ServiceWait_IsGoingToWaitNode()){
                    /*
                     * 这次到达的是等待点，不是真实目的地。
                     */
                    ServiceWait_MarkArrivedWaitNode();
                    work_state = 6;
                    Broadcast_Type12_Service(SERVICE_TYPE12_WAITING_NODE,
                                             ServiceWait_GetRealDestNode(),
                                             ServiceWait_GetServiceNodeValue(),
                                             -1,
                                             ServiceWait_GetWaitNodeValue(),
                                             ServiceWait_GetOwnerCarId());

                    UART1_SendString("[Task06] Arrived wait node. State -> 6\r\n");
                }
                else{
                    Check_And_Broadcast_Task_a = 0;
                    work_state = 2;
                    UART1_SendString("[Task06] Mission Completed. State -> 2 (Done).\r\n");
                }
            }
            else if (work_state == 5) {
                car_current_node = BIRTH_NODE;
                current_start_node = BIRTH_NODE;
                current_end_node = -1;
                Broadcast_Type2_Position();
                work_state = 0;
                UART1_SendString("[Task06] Return Completed. State -> 0 (Idle).\r\n");
            }
        }
        osDelay(200);
    }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartTask04(void *argument){
    while(!system_ready) osDelay(20);
    Broadcast_Type4_OnlineStatus(1);
    UART1_SendString("[Task04] Mission .\r\n");
    /* USER CODE BEGIN StartTask03 */
    /* Infinite loop */
    for(;;)
    {
        osDelay(20);
        Check_And_Broadcast_Task();
        // static uint32_t last_type2_tick = 0;
        // uint32_t now = HAL_GetTick();
        // if (now - last_type2_tick >= 1000) {
        //     last_type2_tick = now;
        //     //Broadcast_Type2_Position();
        // }
        osDelay(20);
        SendFromGetQueueHandle();
    }
    /* USER CODE END StartTask03 */
}
