//
// Created by wxj on 26-4-8.
//

#include "UART.h"
#include <stdio.h>
#include "usart.h"
#include <string.h>
#include "cmsis_os2.h"
#include "oled.h"
#include "FreeRTOS.h"

extern osMessageQueueId_t cmdQueueHandle;
#define WIFI_IPD_CACHE_NUM  8

uint8_t wifi_rx_dma_buf[ESP_RX_SIZE];
uint8_t wifi_frame_buf[ESP_RX_SIZE];
uint16_t wifi_data_len = 0;
uint8_t wifi_frame_ready = 0;
volatile uint8_t esp_is_busy = 0;

static char wifi_ipd_cache[WIFI_IPD_CACHE_NUM][ESP_RX_SIZE];
static volatile uint8_t wifi_ipd_head = 0;
static volatile uint8_t wifi_ipd_tail = 0;
static volatile uint8_t wifi_ipd_count = 0;
static volatile uint8_t wifi_ipd_drop_count = 0;

uint16_t ESP8266_Read(char *out);
static void WIFI_SaveIPD(const char *buf);
static void WIFI_FlushPendingFrameToCache(void);
static uint8_t ESP8266_WaitString(const char *target, uint32_t timeout);

/**
 * @brief 通过UART1发送一个字符串
 * @param str:为发送字符串
 * @note 使用阻塞发送，超时时间为1000ms
 */
void UART1_SendString(char *str)
{
    HAL_StatusTypeDef ret;
    ret = HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
    if(ret != HAL_OK){
        // 发送失败处理
    }
}

/**
 * @brief 通过UART3发送一个字符串
 * @param str:为发送字符串
 * @note 使用阻塞发送，超时时间为1000ms
 */
void UART3_SendString(char *str){
    HAL_StatusTypeDef ret;
    ret = HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 1000);
    if(ret != HAL_OK){
        // 发送失败处理
    }
}

/**
 * @brief 保存 +IPD 数据到缓存队列
 * @note 队列满时丢弃最旧的数据，保留最新的数据
 */
static void WIFI_SaveIPD(const char *buf){
    if(buf == NULL)
        return;
    if(strstr(buf, "+IPD") == NULL)
        return;
    if(wifi_ipd_count >= WIFI_IPD_CACHE_NUM){
        wifi_ipd_tail++;
        if(wifi_ipd_tail >= WIFI_IPD_CACHE_NUM)
            wifi_ipd_tail = 0;
        wifi_ipd_count--;
        wifi_ipd_drop_count++;
    }
    strncpy(wifi_ipd_cache[wifi_ipd_head], buf, ESP_RX_SIZE - 1);
    wifi_ipd_cache[wifi_ipd_head][ESP_RX_SIZE - 1] = '\0';
    wifi_ipd_head++;
    if(wifi_ipd_head >= WIFI_IPD_CACHE_NUM)
        wifi_ipd_head = 0;
    wifi_ipd_count++;
}


/**
 * @brief 读取一条缓存的 +IPD 数据
 * @param out 输出缓冲区
 * @param out_size 输出缓冲区大小
 * @retval 1: 读到数据；0: 没有缓存数据
 */
uint8_t WIFI_GetCachedIPD(char *out, uint16_t out_size){
    if(out == NULL || out_size == 0)
        return 0;
    if(wifi_ipd_count == 0)
        return 0;
    strncpy(out, wifi_ipd_cache[wifi_ipd_tail], out_size - 1);
    out[out_size - 1] = '\0';
    memset(wifi_ipd_cache[wifi_ipd_tail], 0, ESP_RX_SIZE);
    wifi_ipd_tail++;
    if(wifi_ipd_tail >= WIFI_IPD_CACHE_NUM)
        wifi_ipd_tail = 0;
    wifi_ipd_count--;
    return 1;
}

/**
 * @brief 统一处理 UDP 接收数据
 * @note 处理缓存的 +IPD，也处理当前 wifi_frame_ready 中的新 +IPD
 */
void WIFI_ProcessCachedIPD(void){
    char buf[ESP_RX_SIZE];
    if(!esp_is_busy && wifi_frame_ready){
        uint16_t len = ESP8266_Read(buf);
        if(len >= ESP_RX_SIZE){
            len = ESP_RX_SIZE - 1;
        }
        buf[len] = '\0';
        WIFI_SaveIPD(buf);
    }

    while(WIFI_GetCachedIPD(buf, sizeof(buf))){
        UART1_SendString("\r\n[CACHED IPD]\r\n");
        UART1_SendString(buf);
        UART1_SendString("\r\n");
        char *p = strstr(buf, "+IPD,");
        if(p != NULL){
            char *data_ptr = strchr(p, ':');
            if(data_ptr != NULL){
                data_ptr++;
                if(strstr(data_ptr, "\"type\":5") == NULL){
                    UART1_SendString("[UDP] Ignore non-type5\r\n");
                    continue;
                }
                uint16_t data_len = strlen(data_ptr);
                char *q_data = (char *)pvPortMalloc(data_len + 1);
                if(q_data != NULL){
                    strcpy(q_data, data_ptr);
                    if(osMessageQueuePut(cmdQueueHandle, &q_data, 0, 0) != osOK){
                        vPortFree(q_data);
                    }
                }
            }
        }
    }
}

/**
 * @brief 如果当前已有未处理的 ESP8266 串口数据，先读出来并缓存 +IPD
 * @note 用于发送 AT 指令前，避免 wifi_frame_ready = 0 直接清掉数据
 */
static void WIFI_FlushPendingFrameToCache(void){
    char buf[ESP_RX_SIZE];
    if(!wifi_frame_ready)
        return;
    uint16_t len = ESP8266_Read(buf);
    if(len >= ESP_RX_SIZE)
        len = ESP_RX_SIZE - 1;
    buf[len] = '\0';
    UART1_SendString("\r\n[FLUSH BEFORE SEND]\r\n");
    UART1_SendString(buf);
    UART1_SendString("\r\n");
    WIFI_SaveIPD(buf);
}

/**
 * @brief 等待 ESP8266 返回指定字符串
 * @param target 目标字符串，例如 ">"、"SEND OK"、"OK"
 * @param timeout 超时时间
 * @retval 1: 等到目标字符串；0: 超时或失败
 */
static uint8_t ESP8266_WaitString(const char *target, uint32_t timeout){
    char buf[ESP_RX_SIZE];
    uint32_t tick = HAL_GetTick();
    while(HAL_GetTick() - tick < timeout){
        if(wifi_frame_ready){
            uint16_t len = ESP8266_Read(buf);
            if(len >= ESP_RX_SIZE)
                len = ESP_RX_SIZE - 1;
            buf[len] = '\0';
            UART1_SendString(buf);
            WIFI_SaveIPD(buf);
            if(strstr(buf, target)){
                return 1;
            }
            if(strstr(buf, "ERROR") ||
               strstr(buf, "FAIL")  ||
               strstr(buf, "busy")){
                return 0;
            }
        }
        osDelay(5);
    }

    return 0;
}

/**
 * @brief 等待上位机应用层 ACK
 * @note 目前如果上位机没有回复 ACK，不要主动调用这个函数
 */
static uint8_t ESP8266_WaitAppAck(uint16_t seq, uint32_t timeout){
    char buf[ESP_RX_SIZE];
    char ack_str[32];
    uint32_t tick = HAL_GetTick();
    sprintf(ack_str, "ACK=%d", seq);
    while(HAL_GetTick() - tick < timeout){
        if(wifi_frame_ready){
            uint16_t len = ESP8266_Read(buf);
            if(len >= ESP_RX_SIZE)
                len = ESP_RX_SIZE - 1;
            buf[len] = '\0';
            UART1_SendString("\r\n[ESP RX]\r\n");
            UART1_SendString(buf);
            WIFI_SaveIPD(buf);
            if(strstr(buf, ack_str)){
                UART1_SendString("\r\n[APP ACK OK]\r\n");
                return 1;
            }
        }
        osDelay(5);
    }
    UART1_SendString("\r\n[APP ACK TIMEOUT]\r\n");
    return 0;
}

/**
 * @brief 初始化并启动ESP8266的DMA接收模式
 * @note 配合UART IDLE空闲中断使用，可实现不定长数据自动接收
 */
void ESP8266_StartDMA(void){
    memset(wifi_frame_buf, 0, ESP_RX_SIZE);
    wifi_data_len = 0;
    wifi_frame_ready = 0;
    HAL_UART_Receive_DMA(&huart3, wifi_rx_dma_buf, ESP_RX_SIZE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

/**
 * @brief 向ESP发送原始字符串/指令
 * @param cmd 待发送的字符串
 */
void ESP8266_SendRaw(char *cmd)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), 1000);
}

/**
 * @brief 从接收缓冲区读取数据并重置状态
 * @param out 目标缓冲区指针
 * @retval 返回读取到的字节长度
 */
uint16_t ESP8266_Read(char *out){
    uint16_t len = wifi_data_len;
    if(len >= ESP_RX_SIZE)
        len = ESP_RX_SIZE - 1;
    memcpy(out, wifi_frame_buf, len);
    wifi_data_len = 0;
    wifi_frame_ready = 0;
    return len;
}

/**
 * @brief 发送AT指令并判断执行状态
 * @param cmd AT指令字符串
 * @param ack 成功关键字
 * @param timeout 等待时间
 * @retval 1：指令执行成功；0：失败或超时
 */
uint8_t ESP8266_SendCmd(const char *cmd, const char *ack, uint32_t timeout){
    char buf[ESP_RX_SIZE];
    uint32_t tick;
    HAL_StatusTypeDef ret;
    WIFI_FlushPendingFrameToCache();
    esp_is_busy = 1;
    wifi_frame_ready = 0;
    ret = HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), 1000);
    if(ret != HAL_OK){
        esp_is_busy = 0;
        return 0;
    }
    tick = HAL_GetTick();
    while(HAL_GetTick() - tick < timeout){
        if(wifi_frame_ready){
            uint16_t len = ESP8266_Read(buf);
            if(len >= ESP_RX_SIZE)
                len = ESP_RX_SIZE - 1;
            buf[len] = '\0';
            UART1_SendString(buf);
            WIFI_SaveIPD(buf);
            if(strstr(buf, ack)){
                esp_is_busy = 0;
                return 1;
            }
            if(strstr(buf, "ERROR") ||
               strstr(buf, "FAIL")  ||
               strstr(buf, "busy")){
                esp_is_busy = 0;
                return 0;
            }
        }
        osDelay(10);
    }
    esp_is_busy = 0;
    return 0;
}


/**
 * @brief UDP发送数据
 * @param data 待发送数据
 * @param len 数据长度
 * @retval 1：ESP8266 返回 SEND OK；0：发送失败
 *
 * @note 保持原函数接口不变，其他地方不用改调用方式
 */
uint8_t ESP8266_UDP_Send(char *data, uint16_t len){
    char cmd[32];
    HAL_StatusTypeDef ret;
    uint8_t retry;
    uint32_t tick;
    if(data == NULL || len == 0){
        UART1_SendString("UDP DATA ERROR\r\n");
        return 0;
    }
    WIFI_FlushPendingFrameToCache();
    tick = HAL_GetTick();
    while(esp_is_busy){
        if(HAL_GetTick() - tick > 2000){
            UART1_SendString("ESP BUSY TIMEOUT\r\n");
            return 0;
        }
        osDelay(5);
    }
    esp_is_busy = 1;
    for(retry = 0; retry < 3; retry++){
        sprintf(cmd, "AT+CIPSEND=%d\r\n", len);
        wifi_frame_ready = 0;
        ret = HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), 1000);
        if(ret != HAL_OK){
            UART1_SendString("CIPSEND UART FAIL\r\n");
            osDelay(100);
            continue;
        }
        if(!ESP8266_WaitString(">", 2000)){
            UART1_SendString("WAIT > FAIL\r\n");
            osDelay(100);
            continue;
        }
        ret = HAL_UART_Transmit(&huart3, (uint8_t*)data, len, 1000);
        if(ret != HAL_OK){
            UART1_SendString("UDP DATA UART FAIL\r\n");
            osDelay(100);
            continue;
        }
        if(ESP8266_WaitString("SEND OK", 3000)){
            UART1_SendString("UDP SEND OK\r\n");
            esp_is_busy = 0;
            return 1;
        }
        UART1_SendString("WAIT SEND OK FAIL, RETRY\r\n");
        osDelay(100);
    }
    esp_is_busy = 0;
    UART1_SendString("UDP SEND FAILED\r\n");
    return 0;
}


/**
 * @brief WIFI 与 UDP 通讯初始化
 */
void WIFI_Init(void){
    ESP8266_StartDMA();
    ESP8266_SendCmd("AT+RST\r\n", "OK", 5000);
    osDelay(3000);
    ESP8266_SendCmd("AT+CWMODE_CUR=1\r\n", "OK", 1500);
    ESP8266_SendCmd("AT+CWDHCP_DEF=1,1\r\n", "OK", 1000);
    // 连接 WiFi"TP-LINK_509""TP509lab@."
    if (ESP8266_SendCmd("AT+CWJAP_CUR=\"Xiao\",\"123456789\"\r\n", "WIFI GOT IP", 20000)){
        OLED_ShowString(0, 0, "WIFI: OK");
        uint8_t ip_ready = 0;
        for(uint8_t i = 0; i < 5; i++){
            osDelay(2000);
            // 匹配获取 station IP 的关键字，兼容所有路由网段
            if(ESP8266_SendCmd("AT+CIFSR\r\n", "STAIP", 1000)){
                ip_ready = 1;
                break;
            }
        }
        if(!ip_ready){
            OLED_ShowString(0, 0, "IP ERROR");
            return;
        }
    }
    else{
        OLED_ShowString(0, 0, "WIFI: FAIL");
        return;
    }

    ESP8266_SendCmd("AT+CIPMUX=0\r\n", "OK", 1000);
    if(ESP8266_SendCmd("AT+CIPSTART=\"UDP\",\"192.168.135.96\",8080,10001,0\r\n", "OK", 5000)){
        OLED_ShowString(0, 2, "UDP: OK");
        osDelay(500);
        if(ESP8266_SendCmd("AT+CIPSEND=11\r\n", ">", 1000)){
            ESP8266_SendCmd("Robot System Online", "SEND OK", 1000);
        }
    }
    else{
        OLED_ShowString(0, 2, "UDP: FAIL");
        return;
    }
    OLED_ShowFrame();
}