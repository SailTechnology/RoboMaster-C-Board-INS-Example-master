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
void Motor_cali(void);							//校准函数
void Motor_init(void);              // 初始化函数
void Motor_set(int motor1,int motor2,int motor3,int motor4 );              // 设定四个电机的转速，0-1000




#endif
