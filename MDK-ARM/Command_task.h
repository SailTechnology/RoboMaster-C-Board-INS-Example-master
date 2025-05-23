#ifndef __COMMAND_TASK_H
#define __COMMAND_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief 飞行控制任务初始化函数
  * 初始化PID控制器参数并进行必要的硬件配置
  */
void Command_Task_Init(void);

/**
  * @brief 飞行控制任务循环函数
  * 周期调用以执行姿态控制计算，输出电机PWM
  */
void Command_Task_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMMAND_TASK_H */
