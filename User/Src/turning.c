//
// Created by zzy on 26-05-05.
//
//
#include "turning.h"
#include "Motor.h"
#include "mpu6050_turn.h"
#include "buzzer.h"
#include <stdio.h>

#include <string.h>
#include <stdlib.h>

#include "broadcast.h"
#include "cmsis_os2.h"
#include "UART.h"
/**
 * @brief 小功能：执行原地左转 (左轮后退，右轮前进)
 * @param speed 转弯速度 (0-999)
 */
void Action_TurnLeft(int speed) {
    Motor_A_SetSpeed(-speed); // 假设 A 是左轮
    Motor_B_SetSpeed(speed);  // 假设 B 是右轮
}

/**
 * @brief 小功能：执行原地右转 (左轮前进，右轮后退)
 * @param speed 转弯速度 (0-999)
 */
void Action_TurnRight(int speed) {
    Motor_A_SetSpeed(speed);
    Motor_B_SetSpeed(-speed);
}

/**
 * @brief 小功能：判断四路红外是否全亮 (全白)
 * @param state 当前的红外传感器状态结构体
 * @return 1: 全亮 (都没检测到黑线), 0: 有黑线
 * @note 根据你的 linewalking.c，1 代表未检测到黑线 (白区)
 */
uint8_t Check_All_Infrared_Light(LineState_t state) {
    // 检查是否四个探头都输出了 1
    if (state.L1 == 0 && state.L2 == 0 && state.R1 == 0 && state.R2 == 0) {
        return 1;
    }
    return 0;
}

/**
 * @brief 大功能：智能转弯主处理函数
 * @param target_angle 目标角度 (正数左转，0停止/直走，负数右转)
 * @note 此函数应该在 FreeRTOS 任务中被持续循环调用
 */
uint8_t turn_stage = 0;
uint8_t force_turn_stage = 0;

int8_t Smart_Turning_Handler(int target_angle,int8_t start_n,int8_t end_n) {


    // ----------------------------------------------------
    // 阶段 0：正常行驶，等待四路红外全白的“触发信号”
    // ----------------------------------------------------
    if (turn_stage == 0) {
        LineState_t current_line = Line_Read();

        // 检测到路口[cite: 20]
        if (Check_All_Infrared_Light(current_line)||force_turn_stage==1) {
            char debug_buf[64];
            sprintf(debug_buf, "[Turn] Triggered! Target: %d degrees\r\n", target_angle);
            UART1_SendString(debug_buf);

            Motor_Stop();
            osDelay(500);

            if (!(start_n==-1 && end_n==-1)) {
                current_start_node = end_n;
                current_end_node = -1;       // -1 代表正待在节点上，不在两点之间的线上 [cite: 23, 51]
                Broadcast_Type2_Position();
            }




            // 如果相对角度为 0，说明不需要转弯，直接冲过路口白线区
            if (target_angle == 0) {
                if (force_turn_stage==0) {
                    Car_Forward(TURN_SPEED_MID);
                    osDelay(1200);   // 延时冲出十字路口的盲区，具体时间依车速而定
                }

                turn_stage = 2; // 直行结束，直接跳转到完成阶段
                return 1;
            }
            // 否则，启动底盘原地转弯[cite: 20]
            else {

                if (force_turn_stage==0) {
                    Car_Forward(TURN_SPEED_MID);
                    osDelay(1200);   // 延时冲出十字路口的盲区，具体时间依车速而定
                }

                if (target_angle > 0) Action_TurnLeft(TURN_SPEED_HIGH);
                else Action_TurnRight(TURN_SPEED_HIGH);
                turn_stage = 1;
                return 1;
            }
        } else {
            // 没有遇到路口，执行常规寻线[cite: 20]
            LineWalking();
            return 0;
        }
    }

    // ----------------------------------------------------
    // 阶段 1：正在转弯，实时读取 MPU6050 判断[cite: 20]
    // ----------------------------------------------------
    else if (turn_stage == 1) {
        // 调用 MPU6050 非阻塞判断函数[cite: 20]
        if (MPU6050_Check_Turn_Reached(target_angle)) {

            char debug_buf[64];
            sprintf(debug_buf, "[Turn] Reached %d. Stop!\r\n", target_angle);
            UART1_SendString(debug_buf);
            Motor_Stop();
            Buzzer_Beep(200);

            // 转完弯后，往前冲一下回到线上
            Car_Forward(TURN_SPEED_MID);
            osDelay(350);

            turn_stage = 2;
        }
        return 1; // 仍在转弯中
    }

    // ----------------------------------------------------
    // 阶段 2：转弯完成瞬间，发送结束信号
    // ----------------------------------------------------
    else if (turn_stage == 2) {
        turn_stage = 0; // 内部重置状态机
        force_turn_stage=0;
        return 2;       // 返回 2，通知外部主任务“动作结束，可以更新节点了”
    }

    return 0;
}





// ==========================================
// 请确保 turning.c 顶部已经包含了 linewalking.h
// ==========================================

// 引入 broadcast.c 中定义的全局位置变量
extern int8_t current_start_node;
extern int8_t current_end_node;
extern int8_t car_current_node;

// 外部读取红外状态的函数（由 linewalking.c 提供）
extern LineState_t Line_Read(void);

/**
 * @brief 起步时驶离起点十字区域，防止刚启动就把起点识别成下一个路口
 */
static void Leave_Start_Node(void)
{
    uint32_t start_tick = HAL_GetTick();
    LineState_t line;

    UART1_SendString("[Auto] Leave start node begin\r\n");

    /*
     * 起步先强制往前走，离开起点十字白区。
     * 这段期间不要调用 Smart_Turning_Handler，
     * 否则会把起点误判成第一个路口。
     */
    Car_Forward(TURN_SPEED_MID);
    osDelay(1000);

    /*
     * 继续往前，直到红外不再全白。
     * 加超时保护，防止传感器异常卡死。
     */
    while((HAL_GetTick() - start_tick) < 2500)
    {
        line = Line_Read();

        if(Check_All_Infrared_Light(line) == 0)
        {
            UART1_SendString("[Auto] Leave start node done\r\n");
            break;
        }

        Car_Forward(TURN_SPEED_MID);
        osDelay(20);
    }
}
/**
 * @brief 自动行驶序列执行函数
 * @param road               路径节点数组 (对应 map.c 中的 car_road)
 * @param road_len           路径节点数组长度 (对应 car_road_len)
 * @param turn_angles        转弯相对角度数组 (对应 relative_turn_angles)
 * @param turn_angles_len    转弯角度数组长度 (对应 relative_turn_angles_len)
 */
void Execute_Auto_Driving(int8_t* road, int8_t road_len, int* turn_angles, int8_t turn_angles_len) {
    // 安全保护：如果没有路径或者路径节点不足2个，则直接返回
    if (road == NULL || turn_angles == NULL || road_len < 2 || turn_angles_len == 0) {
        printf("[Error] Invalid road or angles array.\r\n");
        return;
    }
    // =================================================================
    // 【新增】：起步航向对齐与“蒙眼”驶出盲区
    // =================================================================
    int initial_angle = turn_angles[0];
    char debug_buf[64];
    // 1. 如果第一个角度不是直行，先原地转弯
    if (initial_angle != 0) {

        sprintf(debug_buf, "[Auto] Initial Turning: %d\r\n", initial_angle);
        UART1_SendString(debug_buf);
        force_turn_stage=1;
        while (1) {
            int8_t status = Smart_Turning_Handler(initial_angle,-1,-1);
            if (status == 2) {
                force_turn_stage=0;
                break;

            } // 只要等于2（转弯硬控中），就代表转完了
            osDelay(20);
        }
    }

    // 2. 强制寻线行驶 300ms（约15个周期），彻底把车身开出起点的十字线！
    // 此时屏蔽全白检测，防止刚起步就被系统误杀
    // UART1_SendString("[Auto] Leaving start node...\r\n");
    //
    // LineWalking(); // 0代表直行寻线
    // osDelay(500);
    Leave_Start_Node();

    int8_t a=0;
    int8_t b=0;

    // 遍历每一段需要行驶的路径
    for (int8_t i = 1; i < turn_angles_len; i++) {
        int target_angle = turn_angles[i];
        int8_t start_n = road[i-1];       // 本段路程的起点
        int8_t end_n = road[i];     // 本段路程的终点（即将到达的节点）

        sprintf(debug_buf, "[Auto] Segment %d: Node %d -> Node %d, Turn: %d\r\n", i, start_n, end_n, target_angle);
        UART1_SendString(debug_buf);
        a=0;
        b=0;
        while (1) {
            // 1. 读取实时红外状态
            LineState_t line = Line_Read();

            // 2. 调用智能转弯与寻线处理逻辑
            // 返回值 0:正常寻线中; 1:路口转弯处理中; 2:路口动作已彻底完成
            int8_t status = Smart_Turning_Handler(target_angle,start_n,end_n);
            // if (status==0) {
            //     UART1_SendString("000000000");
            // }
            // else if (status==1) {
            //     UART1_SendString("11111111111111");
            // }
            // else if (status==2) {
            //     UART1_SendString("222222222222222");
            // }

            // 3. 根据红外状态和当前动作状态，更新节点的全局变量 [cite: 22, 23]
            // 如果触发了全白（红外检测到路口），或者系统正处于转弯硬控阶段（status==1）
            if (Check_All_Infrared_Light(line) == 1 || status == 1) {
                // 在节点上：说明已经到达了本段的终点 [cite: 22, 23]
                current_start_node = end_n;
                current_end_node = -1;       // -1 代表正待在节点上，不在两点之间的线上 [cite: 23, 51]
                if (a==0) {
                    //Broadcast_Type2_Position();
                    a++;
                }

            } else {
                // 在线上：正在从上一节点前往下一节点 [cite: 22, 23]
                current_start_node = start_n;
                current_end_node = end_n;
                if (b==0) {
                    Broadcast_Type2_Position();
                    b++;
                }
            }

            // 4. 如果返回 2，说明这个路口的转弯/直行冲刺动作已彻底完成
            if (status == 2) {
                // 确保跨段瞬间的状态准确
                //current_start_node = end_n;
                //current_end_node = -1;
                a=0;
                b=0;
                break; // 跳出当前路段的 while 循环，进入 for 循环的下一段路径
            }

            // 延时 20ms，与 Smart_Turning_Handler 内部的控制周期保持一致
            osDelay(20);
        }
    }


    // =================================================================
    // 【修复点】角度全部用完，执行最后一段寻迹直奔终点！
    // =================================================================
    int8_t final_start = road[road_len - 2];
    int8_t final_dest = road[road_len - 1];

    sprintf(debug_buf, "[Auto] Final Dash: Node %d -> Destination Node %d\r\n",
            final_start,
            final_dest);
    UART1_SendString(debug_buf);

    /*
     * 最后一段开始时，必须广播 type2。
     * 否则其他车无法释放 final_start，例如：
     * 1 -> 21 时释放 1
     * 0 -> 42 时释放 0
     */
    current_start_node = final_start;
    current_end_node = final_dest;
    Broadcast_Type2_Position();

    while (1) {
        LineState_t line = Line_Read();

        LineWalking();

        if (Check_All_Infrared_Light(line) == 1) {
            Motor_Stop();
            break;
        }

        osDelay(20);
    }

    car_current_node = road[road_len - 1];

    Motor_Stop();

    current_start_node = car_current_node;
    current_end_node = -1;
    Broadcast_Type2_Position();

    sprintf(debug_buf,
            "[Auto] Mission Finished! car_current_node updated to: %d\r\n",
            car_current_node);
    UART1_SendString(debug_buf);
}