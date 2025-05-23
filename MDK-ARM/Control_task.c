/*
 ******************************************************************************
 * @file    Control_task.c
 * @author  SailTechnology
 * @version V1.0.0
 * @date    2025/4/27
 * @brief
 ******************************************************************************
 * @attention
 *
 *****
*/
#include "Control_task.h"
#include "tim.h"
#include "cmsis_os.h"
//满占空比是20000，此处用5%到10%就是1000到2000


//校准电调的时候需要用一次就可以了
void Motor_cali(void){
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2000);	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2000);	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2000);
  HAL_Delay(3000);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
}

//飞机上电后必须要保持最低油门一段时间
void Motor_init(void){
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
}


// 设定四个电机的转速，0-1000
void Motor_set(int motor1,int motor2,int motor3,int motor4 ){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000+motor1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000+motor2);	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000+motor3);	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000+motor4);
}           
















