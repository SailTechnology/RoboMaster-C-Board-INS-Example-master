/**
 ******************************************************************************
 * @file    Receiver_task.h
 * @author  SailTechnology
 * @version V1.0.0
 * @date    2025/4/27
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _RECEIVER_TASK_H
#define _RECEIVER_TASK_H

#include "main.h"


// --- 宏定义 ---
#define SBUS_FRAME_LEN 25   // SBUS数据一帧25字节

// --- 数据结构 ---
typedef struct
{
    uint16_t channels[16];  // 16通道数据
    uint8_t  lost_frame;    // 是否丢帧
    uint8_t  failsafe;      // 是否进入failsafe
} SBUS_Data_t;

// --- 外部变量 ---
extern SBUS_Data_t sbus_data;
extern uint8_t sbus_dma_buf[SBUS_FRAME_LEN];  //声明DMA缓冲区

extern volatile uint8_t sbus_frame_ready;  //告诉别人有这个变量
// --- 函数声明 ---
void sbus_init(void);              // 初始化函数
void parse_sbus_frame(uint8_t *frame); // 解析一帧数据
void sbus_task(void *argument);    // FreeRTOS任务函数
int map_channel(int val);

extern int RC_Throttle,RC_Yaw,RC_Pitch,RC_Roll,RC_Arm,RC_Kill,RC_Lost,RC_Throttle_Manual;




#endif
