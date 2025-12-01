#include "motor.h"
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

 void motor_init(){
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
 }
void forward(){
set_speed[0]= map(g_sbus_channels[0],10);//左前轮
set_speed[1]=map(g_sbus_channels[0],10);//右前轮
set_speed[2]=map(g_sbus_channels[0],10);//左后轮
set_speed[3]=map(g_sbus_channels[0],10);//右后轮
int ch4 = g_sbus_channels[1];
		int B= ch4 - 992;//正的
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,B*25); // 左轮正方向占空比
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,B*25); // 同步另一路 PWM
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);//右轮
}//这里如果id没有严格配对需要改代码
void back(void){
set_speed[0]=-map(g_sbus_channels[0],10);
set_speed[1]=-map(g_sbus_channels[0],10);
set_speed[2]=-map(g_sbus_channels[0],10);
set_speed[3]=-map(g_sbus_channels[0],10);
int ch4 = g_sbus_channels[1];
			int B= 992-ch4;//负的
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0); // 正方向占空比
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,B*25);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0); // 同步另一路 PWM
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,B*25);
}
void left(void){
set_speed[0]= -map(g_sbus_channels[0],10);//左前轮
set_speed[1]=map(g_sbus_channels[0],10);//右前轮
set_speed[2]=-map(g_sbus_channels[0],10);//左后轮
set_speed[3]=map(g_sbus_channels[0],10);//右后轮
int ch4 = g_sbus_channels[1];
			int B= ch4 - 992;//正的
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,-B*25); // 左轮正方向占空比
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,B*25); // 同步另一路 PWM
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);//右轮
	
}
void right(void){
int ch4 = g_sbus_channels[1];
			int B= ch4 - 992;//正的
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,B*25); // 左轮正方向占空比
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,-B*25); // 同步另一路 PWM
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);//右轮
}
void motor_init1(){
set_speed[0]= map(g_sbus_channels[0],0);//左前轮
set_speed[1]=map(g_sbus_channels[0],0);//右前轮
set_speed[2]=map(g_sbus_channels[0],0);//左后轮
set_speed[3]=map(g_sbus_channels[0],0);//右后轮
int ch4 = g_sbus_channels[1];
		int B= ch4 - 992;//正的
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0); // 左轮正方向占空比
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0); // 同步另一路 PWM
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);//右轮


}





























