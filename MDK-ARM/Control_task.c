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
//��ռ�ձ���20000���˴���5%��10%����1000��2000


//У׼�����ʱ����Ҫ��һ�ξͿ�����
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

//�ɻ��ϵ�����Ҫ�����������һ��ʱ��
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


// �趨�ĸ������ת�٣�0-1000
void Motor_set(int motor1,int motor2,int motor3,int motor4 ){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000+motor1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000+motor2);	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000+motor3);	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000+motor4);
}           
















