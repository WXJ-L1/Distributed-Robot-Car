#include "camera.h"

#include "cmsis_os2.h"

extern UART_HandleTypeDef huart2;

/* 全局变量定义 */
AI_mode runmode = Nornal_AI;
QR_AI_Msg QR_msg;
ESP32_AI_Msg esp32_ai_msg;
uint8_t camera_rx_dma_buf[ESP_RX_SIZE];
uint8_t data_buff[ESP_RX_SIZE];
uint8_t recv_buf[ESP_RX_SIZE];
volatile uint8_t newlines = 0;
uint8_t cmd_flag = 0;
uint8_t g_new_flag = 0;
uint8_t g_index = 0;

/* 内部发送辅助 */
static void UART2_Send(char *str) {
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
}

void ESP32_Module_Config_Init(AI_mode AI_set_mode) {
    printf("ESP32 Initializing...\r\n");
    // 1. 开启 DMA 接收与空闲中断
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, camera_rx_dma_buf, ESP_RX_SIZE);
    osDelay(2000); // 等待模块稳定
    // 2. 配置模式 (这里简化为直接发送指令)
    UART2_Send("wifi_mode:2"); osDelay(500);
    // 3. 设置具体的 AI 模式
    SET_ESP_AI_MODE(AI_set_mode);
    // 4. 获取 IP 用于确认连接
    Get_APIP(); osDelay(1000);
    Get_STAIP(); osDelay(1000);
    // 5. 重要：根据模式锁定解析标志位
    if(AI_set_mode == QR_AI) cmd_flag = 4;
    else cmd_flag = 0;
    printf("ESP32 Config Done!\r\n");
}

void SET_ESP_AI_MODE(AI_mode Mode) {
    char buf[20];
    sprintf(buf, "ai_mode:%d", Mode);
    UART2_Send(buf);
    runmode = Mode;
    osDelay(1000);
}

void Get_STAIP(void) { UART2_Send("sta_ip"); cmd_flag = 1; }
void Get_APIP(void) { UART2_Send("ap_ip"); cmd_flag = 1; }

/* ---------------- 数据解析逻辑 (中断中调用) ---------------- */

void recv_QR_data(char QRdata) {
    if (QRdata == '$' && g_new_flag == 0) {
        g_new_flag = 1;
        g_index = 0;
        memset(recv_buf, 0, sizeof(recv_buf));
        return;
    }
    if(g_new_flag == 1) {
        if (QRdata == '#') {
            g_new_flag = 0;
            memcpy(QR_msg.QR_msg, recv_buf, g_index);
            QR_msg.QR_msg[g_index] = '\0';
            newlines = 1;
        } else if (QRdata == '$') {
            g_index = 0;
        } else {
            recv_buf[g_index++] = QRdata;
        }
        if(g_index >= (ESP_RX_SIZE - 1)) g_new_flag = 0; // 防止溢出
    }
}

void Data_Deal(uint8_t RXdata) {
    if(cmd_flag == 4) recv_QR_data(RXdata);
    else if(cmd_flag == 1) { /* 处理 IP 数据 */ }
}