/**
 ******************************************************************************
 * @file    Control_task.h
 * @author  SailTechnology
 * @version V1.0.0
 * @date    2025/4/27
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _CONTROL_TASK_H
#define _CONTROL_TASK_H

#include "main.h"
#include "stm32f4xx_hal.h"
void Motor_cali(void);							//У׼����
void Motor_init(void);              // ��ʼ������
void Motor_set(int motor1,int motor2,int motor3,int motor4 );              // �趨�ĸ������ת�٣�0-1000




#endif
