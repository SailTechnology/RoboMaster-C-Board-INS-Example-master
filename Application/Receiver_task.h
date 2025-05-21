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


// --- �궨�� ---
#define SBUS_FRAME_LEN 25   // SBUS����һ֡25�ֽ�

// --- ���ݽṹ ---
typedef struct
{
    uint16_t channels[16];  // 16ͨ������
    uint8_t  lost_frame;    // �Ƿ�֡
    uint8_t  failsafe;      // �Ƿ����failsafe
} SBUS_Data_t;

// --- �ⲿ���� ---
extern SBUS_Data_t sbus_data;
extern uint8_t sbus_dma_buf[SBUS_FRAME_LEN];  //����DMA������

extern volatile uint8_t sbus_frame_ready;  //���߱������������
// --- �������� ---
void sbus_init(void);              // ��ʼ������
void parse_sbus_frame(uint8_t *frame); // ����һ֡����
void sbus_task(void *argument);    // FreeRTOS������
int map_channel(int val);

extern int RC_Throttle,RC_Yaw,RC_Pitch,RC_Roll,RC_Arm,RC_Kill,RC_Lost,RC_Throttle_Manual;




#endif
