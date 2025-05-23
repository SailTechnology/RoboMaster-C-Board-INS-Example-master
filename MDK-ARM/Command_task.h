#ifndef __COMMAND_TASK_H
#define __COMMAND_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief ���п��������ʼ������
  * ��ʼ��PID���������������б�Ҫ��Ӳ������
  */
void Command_Task_Init(void);

/**
  * @brief ���п�������ѭ������
  * ���ڵ�����ִ����̬���Ƽ��㣬������PWM
  */
void Command_Task_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMMAND_TASK_H */
