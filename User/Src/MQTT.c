//
// Created by 10663 on 26-5-16.
//

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "main.h"
#include "usart.h"
#include "oled.h"
#include "MQTT.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

/* ===================== 用户配置区 ===================== */

#define MQTT_UART              huart6
#define MQTT_UART_INSTANCE     USART6
// 连接 WiFi"TP-LINK_509""TP509lab@."
#define MQTT_WIFI_SSID         "Xiao"
#define MQTT_WIFI_PASSWORD     "123456789"

#define MQTT_BROKER            "broker.emqx.io"
#define MQTT_PORT              1883
#define MQTT_CLIENT_ID         "CAR2_MQTT_CLIENT"

#define MQTT_DMA_RX_SIZE       128
#define MQTT_IRQ_BUF_SIZE      512
#define MQTT_PARSE_BUF_SIZE    1024
#define MQTT_CMD_RX_SIZE       512

#ifndef MQTT_MSG_MAX_LEN
#define MQTT_MSG_MAX_LEN       256
#endif

static osMutexId_t mqtt_send_mutex = NULL;
static const osMutexAttr_t mqtt_send_mutex_attr = {
    .name = "mqttSendMutex"
};

/* ===================== 外部队列 ===================== */
extern osMessageQueueId_t mqttQueueHandle;

/* ===================== 内部变量 ===================== */
static uint8_t mqtt_dma_rx_buf[MQTT_DMA_RX_SIZE];

static char mqtt_irq_buf[MQTT_IRQ_BUF_SIZE];
static volatile uint16_t mqtt_irq_len = 0;
static volatile uint8_t mqtt_irq_ready = 0;
static char mqtt_cmd_rx_buf[MQTT_CMD_RX_SIZE];
static uint16_t mqtt_cmd_rx_len = 0;
static uint8_t mqtt_is_ready = 0;

/* ===================== 内部函数声明 ===================== */

static void MQTT_Show(const char *msg);
static void MQTT_ClearCmdRx(void);
static uint8_t MQTT_SendRaw(const char *str);
static uint8_t MQTT_WaitString(const char *wait, uint32_t timeout);
static uint8_t MQTT_SendWait(const char *cmd, const char *wait, uint32_t timeout);
static int MQTT_FindJsonEnd(const char *buf, uint16_t len);

/* ===================== OLED 显示 ===================== */
static void MQTT_Show(const char *msg){
    OLED_ShowString(0, 4, "                ");
    OLED_ShowString(0, 4, msg);
    OLED_ShowFrame();
}

/* ===================== AT 指令阻塞收发 ===================== */
static void MQTT_ClearCmdRx(void){
    memset(mqtt_cmd_rx_buf, 0, sizeof(mqtt_cmd_rx_buf));
    mqtt_cmd_rx_len = 0;
}

static uint8_t MQTT_SendRaw(const char *str){
    if(str == NULL){
        return 0;
    }
    if(strlen(str) == 0){
        return 0;
    }

    if(HAL_UART_Transmit(&MQTT_UART,
                         (uint8_t *)str,
                         strlen(str),
                         1000) != HAL_OK){
        return 0;
    }
    return 1;
}

static uint8_t MQTT_WaitString(const char *wait, uint32_t timeout){
    uint8_t ch;
    uint32_t start;
    if(wait == NULL){
        return 1;
    }

    start = HAL_GetTick();
    while((HAL_GetTick() - start) < timeout){
        if(HAL_UART_Receive(&MQTT_UART, &ch, 1, 20) == HAL_OK){
            if(mqtt_cmd_rx_len < sizeof(mqtt_cmd_rx_buf) - 1){
                mqtt_cmd_rx_buf[mqtt_cmd_rx_len++] = (char)ch;
                mqtt_cmd_rx_buf[mqtt_cmd_rx_len] = '\0';
            }
            if(strstr(mqtt_cmd_rx_buf, wait) != NULL){
                return 1;
            }
            if(strstr(mqtt_cmd_rx_buf, "ERROR") != NULL){
                return 0;
            }
            if(strstr(mqtt_cmd_rx_buf, "FAIL") != NULL){
                return 0;
            }
        }
    }
    return 0;
}

static uint8_t MQTT_SendWait(const char *cmd, const char *wait, uint32_t timeout){
    MQTT_ClearCmdRx();
    if(!MQTT_SendRaw(cmd)){
        return 0;
    }
    return MQTT_WaitString(wait, timeout);
}

/* ===================== DMA 接收启动 ===================== */
uint8_t MQTT_StartDmaReceive(void){
    HAL_UART_DMAStop(&MQTT_UART);
    __disable_irq();
    memset(mqtt_dma_rx_buf, 0, sizeof(mqtt_dma_rx_buf));
    memset(mqtt_irq_buf, 0, sizeof(mqtt_irq_buf));
    mqtt_irq_len = 0;
    mqtt_irq_ready = 0;
    __enable_irq();
    if(HAL_UARTEx_ReceiveToIdle_DMA(&MQTT_UART,
                                    mqtt_dma_rx_buf,
                                    MQTT_DMA_RX_SIZE) != HAL_OK){
        return 0;
    }

    if(MQTT_UART.hdmarx != NULL){
        __HAL_DMA_DISABLE_IT(MQTT_UART.hdmarx, DMA_IT_HT);
    }
    return 1;
}

/* ===================== UART DMA 空闲中断回调 ===================== */
void MQTT_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    uint16_t copy_len;
    uint16_t free_len;
    if(huart->Instance != MQTT_UART_INSTANCE){
        return;
    }

    if(Size > MQTT_DMA_RX_SIZE){
        Size = MQTT_DMA_RX_SIZE;
    }

    if(Size > 0){
        free_len = MQTT_IRQ_BUF_SIZE - 1 - mqtt_irq_len;
        if(Size <= free_len){
            copy_len = Size;
        }
        else{
            copy_len = free_len;
        }

        if(copy_len > 0){
            memcpy(&mqtt_irq_buf[mqtt_irq_len],
                   mqtt_dma_rx_buf,
                   copy_len);
            mqtt_irq_len += copy_len;
            mqtt_irq_buf[mqtt_irq_len] = '\0';
            mqtt_irq_ready = 1;
        }
        else{
            mqtt_irq_len = 0;
            mqtt_irq_buf[0] = '\0';
        }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&MQTT_UART,
                                 mqtt_dma_rx_buf,
                                 MQTT_DMA_RX_SIZE);

    if(MQTT_UART.hdmarx != NULL){
        __HAL_DMA_DISABLE_IT(MQTT_UART.hdmarx, DMA_IT_HT);
    }
}

/* ===================== JSON 查找 ===================== */
static int MQTT_FindJsonEnd(const char *buf, uint16_t len){
    uint16_t i;
    int depth = 0;
    uint8_t in_string = 0;
    uint8_t escape = 0;
    uint8_t started = 0;
    for(i = 0; i < len; i++){
        char c = buf[i];
        if(escape){
            escape = 0;
            continue;
        }
        if(c == '\\'){
            if(in_string){
                escape = 1;
            }
            continue;
        }

        if(c == '"'){
            in_string = !in_string;
            continue;
        }

        if(in_string){
            continue;
        }

        if(c == '{'){
            depth++;
            started = 1;
        }
        else if(c == '}'){
            if(started){
                depth--;
                if(depth == 0){
                    return i;
                }
            }
        }
    }
    return -1;
}

/* ===================== MQTT 初始化 ===================== */

uint8_t MQTT_Init(void){
    char cmd[180];
    if(mqtt_send_mutex == NULL)
    {
        mqtt_send_mutex = osMutexNew(&mqtt_send_mutex_attr);
    }
    mqtt_is_ready = 0;
    HAL_UART_DMAStop(&MQTT_UART);
    MQTT_Show("MQTT INIT");
    HAL_Delay(3000);

    MQTT_Show("AT");
    if(!MQTT_SendWait("AT\r\n", "OK", 3000)){
        MQTT_Show("AT FAIL");
        return 0;
    }

    MQTT_Show("MODE");
    if(!MQTT_SendWait("AT+CWMODE=0\r\n", "OK", 3000)){
        MQTT_Show("MODE FAIL");
        return 0;
    }

    MQTT_Show("WIFI");
    snprintf(cmd,
             sizeof(cmd),
             "AT+CWJAP=%s,%s\r\n",
             MQTT_WIFI_SSID,
             MQTT_WIFI_PASSWORD);

    if(!MQTT_SendWait(cmd, "+CWJAP:1", 30000)){
        MQTT_Show("WIFI FAIL");
        return 0;
    }

    MQTT_Show("CLEAN");
    MQTT_SendWait("AT+MQTTCLEAN\r\n", "OK", 3000);
    HAL_Delay(500);
    MQTT_Show("MQTT ID");
    snprintf(cmd,
             sizeof(cmd),
             "AT+MQTTLONGCLIENTID=%s\r\n",
             MQTT_CLIENT_ID);

    if(!MQTT_SendWait(cmd, "OK", 5000)){
        MQTT_Show("ID FAIL");
        return 0;
    }

    MQTT_Show("MQTT CONN");
    snprintf(cmd,
             sizeof(cmd),
             "AT+MQTTCONN=%s,%d,1\r\n",
             MQTT_BROKER,
             MQTT_PORT);

    if(!MQTT_SendWait(cmd, "+MQTTCONNECTED", 30000)){
        MQTT_Show("CONN FAIL");
        return 0;
    }

    MQTT_Show("SUB");
    snprintf(cmd,
             sizeof(cmd),
             "AT+MQTTSUB=%s,0\r\n",
             MQTT_TOPIC);

    if(!MQTT_SendWait(cmd, "OK", 5000)){
        MQTT_Show("SUB FAIL");
        return 0;
    }

    if(!MQTT_StartDmaReceive()){
        MQTT_Show("DMA FAIL");
        return 0;
    }
    mqtt_is_ready = 1;
    MQTT_Show("MQTT OK");
    return 1;
}

/* ===================== MQTT 发布 ===================== */
uint8_t MQTT_Send(const char *topic, const char *msg)
{
    char cmd[180];
    uint16_t len;
    uint8_t ret = 0;

    if(topic == NULL || msg == NULL)
    {
        return 0;
    }

    len = (uint16_t)strlen(msg);
    if(len == 0)
    {
        return 0;
    }

    if(!mqtt_is_ready)
    {
        return 0;
    }

    if(mqtt_send_mutex == NULL)
    {
        mqtt_send_mutex = osMutexNew(&mqtt_send_mutex_attr);
        if(mqtt_send_mutex == NULL)
        {
            return 0;
        }
    }

    /*
     * 关键：整个 MQTT 发布流程必须加锁。
     * 防止 type0/type1/type2/type6 同时发，导致 AT 指令串台。
     */
    if(osMutexAcquire(mqtt_send_mutex, 3000) != osOK)
    {
        return 0;
    }

    /*
     * 停止 DMA，改用阻塞方式等待 AT 响应。
     */
    HAL_UART_DMAStop(&MQTT_UART);
    MQTT_ClearCmdRx();

    /*
     * 如果你当前固件使用的是：
     * AT+MQTTPUBRAW=car/broadcast,长度,0,0
     * 就保留下面这个格式。
     */
    snprintf(cmd,
             sizeof(cmd),
             "AT+MQTTPUBRAW=%s,%u,0,0\r\n",
             topic,
             len);

    /*
     * 等待 ESP 返回 >，说明可以开始输入 payload。
     */
    if(!MQTT_SendWait(cmd, ">", 5000))
    {
        goto exit_mqtt_send;
    }

    /*
     * 发送真正的 JSON payload。
     */
    if(HAL_UART_Transmit(&MQTT_UART,
                         (uint8_t *)msg,
                         len,
                         3000) != HAL_OK)
    {
        goto exit_mqtt_send;
    }

    /*
     * 关键修复：
     * 发送 payload 后，必须继续等待模块发布完成。
     * 不要立刻打开 DMA。
     */
    MQTT_ClearCmdRx();

    if(!MQTT_WaitString("OK", 5000))
    {
        goto exit_mqtt_send;
    }

    ret = 1;

    exit_mqtt_send:

        /*
         * 无论成功失败，都恢复 DMA 接收。
         */
        MQTT_StartDmaReceive();

    osMutexRelease(mqtt_send_mutex);

    return ret;
}

/* ===================== DMA 数据解析入队 ===================== */
uint8_t MQTT_ProcessDmaToQueue(void){
    static char parse_buf[MQTT_PARSE_BUF_SIZE];
    static uint16_t parse_len = 0;
    char chunk[MQTT_IRQ_BUF_SIZE];
    uint16_t chunk_len;
    char *json_start;
    int json_end_index;

    uint16_t prefix_len;
    uint16_t json_len;
    uint16_t remain_len;

    char *p_msg;
    uint8_t put_count = 0;

    if(!mqtt_irq_ready){
        return 0;
    }

    memset(chunk, 0, sizeof(chunk));
    __disable_irq();
    chunk_len = mqtt_irq_len;
    if(chunk_len > sizeof(chunk) - 1){
        chunk_len = sizeof(chunk) - 1;
    }
    memcpy(chunk, mqtt_irq_buf, chunk_len);
    chunk[chunk_len] = '\0';
    mqtt_irq_len = 0;
    mqtt_irq_buf[0] = '\0';
    mqtt_irq_ready = 0;
    __enable_irq();

    if(chunk_len == 0){
        return 0;
    }

    if((parse_len + chunk_len) >= MQTT_PARSE_BUF_SIZE){
        parse_len = 0;
        parse_buf[0] = '\0';
        return 0;
    }

    memcpy(&parse_buf[parse_len], chunk, chunk_len);
    parse_len += chunk_len;
    parse_buf[parse_len] = '\0';

    while(parse_len > 0){
        json_start = strchr(parse_buf, '{');

        if(json_start == NULL){
            parse_len = 0;
            parse_buf[0] = '\0';
            break;
        }

        prefix_len = (uint16_t)(json_start - parse_buf);

        if(prefix_len > 0){
            memmove(parse_buf,
                    json_start,
                    parse_len - prefix_len);

            parse_len -= prefix_len;
            parse_buf[parse_len] = '\0';
        }

        json_end_index = MQTT_FindJsonEnd(parse_buf, parse_len);

        if(json_end_index < 0){
            break;
        }

        json_len = (uint16_t)json_end_index + 1;
        if(json_len >= MQTT_MSG_MAX_LEN){
            remain_len = parse_len - json_len;

            if(remain_len > 0){
                memmove(parse_buf,
                        &parse_buf[json_len],
                        remain_len);

                parse_len = remain_len;
                parse_buf[parse_len] = '\0';
            }
            else{
                parse_len = 0;
                parse_buf[0] = '\0';
            }

            continue;
        }
        p_msg = (char *)pvPortMalloc(json_len + 1);
        if(p_msg == NULL){
            return put_count;
        }

        memcpy(p_msg, parse_buf, json_len);
        p_msg[json_len] = '\0';

        if(osMessageQueuePut(mqttQueueHandle, &p_msg, 0, 0) != osOK){
            vPortFree(p_msg);
            return put_count;
        }

        put_count++;
        remain_len = parse_len - json_len;
        if(remain_len > 0){
            memmove(parse_buf,
                    &parse_buf[json_len],
                    remain_len);

            parse_len = remain_len;
            parse_buf[parse_len] = '\0';
        }
        else{
            parse_len = 0;
            parse_buf[0] = '\0';
        }
    }
    return put_count;
}