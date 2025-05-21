/**
 ******************************************************************************
 * @file    Receiver_task.c
 * @author  SailTechnology
 * @version V1.0.0
 * @date    2025/4/27
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "Receiver_task.h"
#include "usart.h"   // 包含 huart2 的定义
#include "cmsis_os.h"



// --- 内部宏定义 ---
#define SBUS_START_BYTE 0x0F

// --- 全局变量 ---
uint8_t sbus_dma_buf[SBUS_FRAME_LEN];
SBUS_Data_t sbus_data;
volatile uint8_t sbus_frame_ready = 0;
int RC_Throttle,RC_Yaw,RC_Pitch,RC_Roll,RC_Arm,RC_Kill,RC_Lost,RC_Throttle_Manual =0;
//RC_Arm 高电平为解锁
//RC_Kill 高电平为急停
//RC_Lost 高电平为丢失信号
//前三个通道均为-500到500
//manual模式的thr为0到1000

int RC_deadzone = 5;



// --- 初始化函数 ---
void sbus_init(void)
{
    HAL_UART_Receive_DMA(&huart3, sbus_dma_buf, SBUS_FRAME_LEN);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}

// 映射函数
int map_channel(int val)
{
    // 限制输入范围到 [172, 1810]
    if (val < 172) val = 172;
    if (val > 1810) val = 1810;

    // 映射到 [1000, 2000]
    int out = ((val - 172) * 1000) / (1810 - 172) + 1000;

    // 死区处理：输出在 [1495, 1505] 之间直接设为 1500
    if (out > 1495 && out < 1505)
        out = 1500;

    return out;
}

	
int map_channel_manual(int val)
{
    // 限制到 [172,1810]
    if (val < 172) val = 172;
    if (val > 1810) val = 1810;
    int out = ((val - 172) * 1000) / (1810 - 172); // 线性映射到[0,1000]
    return out;
}

// --- 解析一帧SBUS数据 ---
void parse_sbus_frame(uint8_t *frame)
{
    if (frame[0] != SBUS_START_BYTE)
        return;  // 起始位错误，不解析

    sbus_data.channels[0]  = ((frame[1]    | frame[2] << 8) & 0x07FF);
    sbus_data.channels[1]  = ((frame[2] >> 3 | frame[3] << 5) & 0x07FF);
    sbus_data.channels[2]  = ((frame[3] >> 6 | frame[4] << 2 | frame[5] << 10) & 0x07FF);
    sbus_data.channels[3]  = ((frame[5] >> 1 | frame[6] << 7) & 0x07FF);
    sbus_data.channels[4]  = ((frame[6] >> 4 | frame[7] << 4) & 0x07FF);
    sbus_data.channels[5]  = ((frame[7] >> 7 | frame[8] << 1 | frame[9] << 9) & 0x07FF);
    sbus_data.channels[6]  = ((frame[9] >> 2 | frame[10] << 6) & 0x07FF);
    sbus_data.channels[7]  = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF);
    sbus_data.channels[8]  = ((frame[12]    | frame[13] << 8) & 0x07FF);
    sbus_data.channels[9]  = ((frame[13] >> 3 | frame[14] << 5) & 0x07FF);
    sbus_data.channels[10] = ((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF);
    sbus_data.channels[11] = ((frame[16] >> 1 | frame[17] << 7) & 0x07FF);
    sbus_data.channels[12] = ((frame[17] >> 4 | frame[18] << 4) & 0x07FF);
    sbus_data.channels[13] = ((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF);
    sbus_data.channels[14] = ((frame[20] >> 2 | frame[21] << 6) & 0x07FF);
    sbus_data.channels[15] = ((frame[21] >> 5 | frame[22] << 3) & 0x07FF);

    sbus_data.lost_frame = (frame[23] >> 2) & 0x01;
    sbus_data.failsafe   = (frame[23] >> 3) & 0x01;
		


		// 注意！这里是映射后的
		RC_Throttle = map_channel(sbus_data.channels[2]);
		RC_Throttle_Manual = map_channel_manual(sbus_data.channels[2]);
		RC_Yaw      = map_channel(sbus_data.channels[3]);
		RC_Pitch    = map_channel(sbus_data.channels[1]);
		RC_Roll     = map_channel(sbus_data.channels[0]);

		// 开关量可以保留原来的
		if (sbus_data.channels[4] > 500) RC_Arm = 1; else RC_Arm = 0;
		if (sbus_data.channels[5] > 500) RC_Kill = 1; else RC_Kill = 0;

		RC_Lost = sbus_data.failsafe;
}



// --- FreeRTOS任务：专门处理解析 ---
void sbus_task(void *argument)
{
    for (;;)
    {
        if (sbus_frame_ready)
        {
            sbus_frame_ready = 0;
            parse_sbus_frame(sbus_dma_buf);
        }

        osDelay(1);
    }
}
