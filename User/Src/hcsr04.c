//
// Created by wxj on 26-4-9.
//

#include "hcsr04.h"
#include <stdint.h>
#include <stdio.h>
#include "cmsis_os2.h"
#include "tim.h"
#include "UART.h"

static volatile uint32_t ic_val1 = 0;
static volatile uint32_t ic_val2 = 0;
static volatile uint8_t  is_first_capture = 0;
static volatile float    distance_cm = -1.0f;

HCSR04_HandleTypeDef hcsr04;

/**
 * @brief 初始化HCSR04超声波传感器
 * @param dev:HCSR04设备结构体指针 GPIO端口以及定时器配置
 * note 1.确保Trig引脚处于低电平
 *      2.开启定时器输入捕获中断，准备接收Echo引脚的回响脉冲
 */
void HCSR04_Init(HCSR04_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->TRIG_Port, dev->TRIG_Pin, GPIO_PIN_RESET);
    HAL_TIM_IC_Start_IT(dev->htim, dev->channel);
}

void Ultrasonic_Init_All(void)
{
    hcsr04.TRIG_Port = GPIOB;
    hcsr04.TRIG_Pin  = GPIO_PIN_2;
    hcsr04.htim      = &htim3;
    hcsr04.channel   = TIM_CHANNEL_3;
    HCSR04_Init(&hcsr04);
}

static void Delay_us(uint32_t us)
{
    uint32_t count = us * 10;
    while(count--) __NOP();
}

/**
  * @brief  触发一次超声波测距信号
  * @param  dev: HCSR04 设备结构体指针
  * @note   该函数遵循 HC-SR04 标准时序：
  * 1. 先拉低确保电平干净
  * 2. 发送至少 10us 的高电平脉冲
  * 3. 拉低等待模块内部发出 8 个 40kHz 的超声波周期
  * @retval 无
  */
void HCSR04_Trigger(HCSR04_HandleTypeDef *dev)
{
    HAL_GPIO_WritePin(dev->TRIG_Port, dev->TRIG_Pin, GPIO_PIN_RESET);
    Delay_us(2);
    HAL_GPIO_WritePin(dev->TRIG_Port, dev->TRIG_Pin, GPIO_PIN_SET);
    Delay_us(10);
    HAL_GPIO_WritePin(dev->TRIG_Port, dev->TRIG_Pin, GPIO_PIN_RESET);
}

float HCSR04_GetDistance(void)
{
    return distance_cm;
}

/**
  * @brief  执行多次测量并计算平均距离
  * @note   该函数通过三次采样取平均值的方法来滤除环境噪声导致的突变误差。
  * 每次测量间隔 60ms+20ms，总耗时约 240ms。
  * @retval float: 平均距离（单位：cm）
  */
float Measure_Distance(void)
{
    float sum = 0;
    char buf[64];
    for(int i = 0; i < 3; i++)
    {
        HCSR04_Trigger(&hcsr04);
        osDelay(60);
        sum += HCSR04_GetDistance();
        osDelay(20);
    }
    float distance = sum / 3.0f;
    sprintf(buf, "Distance: %d cm\r\n", (int)(distance + 0.5f));
    return distance;
}

/**
 * @brief 定时器输入捕获回调函数
 * @param htim :定时器句柄
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            if(is_first_capture == 0)
            {
                ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
                is_first_capture = 1;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            else
            {
                ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
                uint32_t diff;
                if(ic_val2 > ic_val1)
                    diff = ic_val2 - ic_val1;
                else
                    diff = (65535 - ic_val1) + ic_val2;
                distance_cm = diff * 0.0343f / 2.0f;
                is_first_capture = 0;
                __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
            }
        }
    }
}
