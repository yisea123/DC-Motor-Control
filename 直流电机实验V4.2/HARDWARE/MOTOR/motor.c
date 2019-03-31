#include "timer.h"
#include "v_pid.h"
#include "motor.h"
#include "led.h"
#include "usart.h"
#include "lcd.h"


//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8| GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);		

	
//定时器初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
 //初始化输出比较参数
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式1  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能  
  //TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值  
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高  
	
	TIM_OC1Init(TIM1,&TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1预装载使能	 
//	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
//	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH2预装载使能	 
//	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
//	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH3预装载使能	
	TIM_OC4Init(TIM1,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH4预装载使能
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //使能TIMx在ARR上的预装载寄存器  
	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE 主输出使能	
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1


}

void TIM8_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); //时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6| GPIO_Pin_7|GPIO_Pin_8| GPIO_Pin_9;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOC, &GPIO_InitStructure);		

	
//定时器初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
 //初始化输出比较参数
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式1  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能  
  //TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值  
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高  
	
	TIM_OC1Init(TIM8,&TIM_OCInitStructure); 
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	 
	TIM_OC2Init(TIM8,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能	 
	TIM_OC3Init(TIM8,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH3预装载使能	
	TIM_OC4Init(TIM8,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH4预装载使能
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIMx在ARR上的预装载寄存器  
	TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE 主输出使能	
	TIM_Cmd(TIM8, ENABLE);  //使能TIM1


}


void Motor_pwm1_set(int pwm)
{
	if(pwm>3599)
		pwm=3599;
	if(pwm<0)
		pwm=0;
		TIM_SetCompare1(TIM8,pwm);	
}
void Motor_pwm2_set(int pwm)
{
	if(pwm>3599)
		pwm=3599;
	if(pwm<0)
		pwm=0;
		TIM_SetCompare2(TIM8,pwm);	
}

void Motor_pwm3_set(int pwm)
{
	if(pwm>3599)
		pwm=3599;
	if(pwm<0)
		pwm=0;
		TIM_SetCompare3(TIM8,pwm);	
}
void Motor_pwm4_set(int pwm)
{
	if(pwm>3599)
		pwm=3599;
	if(pwm<0)
		pwm=0;
		TIM_SetCompare4(TIM8,3599-pwm);	 //原因不详，可能是初始化时设置的问题
}

void Motor_pwm5_set(int pwm)
{
	if(pwm>3599)
		pwm=3599;
	if(pwm<0)
		pwm=0;
		TIM_SetCompare1(TIM1,pwm);	
}
void Motor_pwm6_set(int pwm)
{
	if(pwm>3599)
		pwm=3599;
	if(pwm<0)
		pwm=0;
		TIM_SetCompare4(TIM1,3599-pwm);	
}



