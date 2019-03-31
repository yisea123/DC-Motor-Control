#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "lcd.h"
#include "motor.h"
#include "v_pid.h"
#include "p_pid.h"
#include "stm32f10x_fsmc.h"
/************************************************
 ALIENTEK战舰STM32开发板实验8
 定时器中断实验
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

/***************************************************

        速度PID，修改了智能车PID代码的一个bug
				
				然后，多了一个bug 
				
				或许得试试模糊PID，或者专家PID

         硬件连接： 开发板  A6-PWM4 A7-PWM3 GND-GND
			               电机	 红-OUT1  黑-OUT2
										 编码器 :B6-B   B7-A
                      串口1接PC
																				方
																				2018/10/5

******************************************************/
#define VMAX  33 
int Count=10000,Old_Could=10000;
long Real_Position=0;
int now_velocity=0;
int set_velocity=0;
int pwm1=0,pwm2=0;  //仅用于显示
extern short int PID_m_add;
extern int PID_P_add; 
int new_flag=0;

volatile long Set_Position=0;
extern int Max_V;
 int main(void)
 {		
 	u16 t,i;
	 long num,sendnum;  
	u16 len;	
	
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(9600);	 //串口初始化为115200
 	LED_Init();			     //LED端口初始化
 
  LCD_Init();	
	POINT_COLOR=BLUE;																		//字体颜色
	TIM3_PWM_Init(3559,0);
	TIM4_Int_Init();  
	TIM2_Int_Init(9,7199);  
	 pwm1=0;
	 pwm2=0;
   	while(1)
	{
	
		if(now_velocity>=0)															
			LCD_ShowString(30,150,200,16,16,"velocity:   ");
		 else
			LCD_ShowString(30,150,200,16,16,"velocity: -");	
		 LCD_ShowNum(150,150,abs(now_velocity),4,16);
		 if(Real_Position>=0)													
					LCD_ShowString(30,200,200,16,16,"Position:   ");
		 else
			LCD_ShowString(30,200,200,16,16,"Position:-");	
		 LCD_ShowNum(150,200,abs(Real_Position),6,16);
		 
		 
		 if(set_velocity>=0)	
		 LCD_ShowString(30,80,200,16,16,"set_velocity:  ");
		  else
		  LCD_ShowString(30,80,200,16,16,"set_velocity: -");
				pwm2=abs(set_velocity);
		 LCD_ShowNum(150,80,pwm2,4,16);
		 
		 
		 if(Set_Position>=0)	
		 LCD_ShowString(30,100,200,16,16,"Set_Position:  ");
		  else
		 LCD_ShowString(30,100,200,16,16,"Set_Position: -");
			pwm1=abs(Set_Position);
		 LCD_ShowNum(150,100,pwm1,6,16);
			
		//		printf("请输入设定位置,以回车键结束\n");  
			if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
		//	printf("\r\n您发送的消息为:\r\n\r\n");
			for(t=0;t<len;t++)
			{
				USART_SendData(USART1, USART_RX_BUF[t]);//向串口1发送数据
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
			}
			
		//	printf("\nlen is %d\n",len);
		
			if(USART_RX_BUF[0]=='-')
			{
				sendnum=0;
				for(t=1;t<len;t++)
				{	
				
					num=USART_RX_BUF[t]-48;
					for(i=0;i<len-1-t;i++)
					{
						num*=10;
					}
					sendnum+=num;
				}
				sendnum=-sendnum;
				Set_Position=sendnum;
				new_flag=1;
			}
			else
			{  
				sendnum=0;
				for(t=0;t<len;t++)
				{	
					
					num=USART_RX_BUF[t]-48;
					for(i=0;i<len-1-t;i++)
					{
						num*=10;
					}
					sendnum+=num;
				}
					Set_Position=sendnum;
				new_flag=1;
			}
		//	printf("\r\n\r\n");//插入换行
				
			printf("\nSet_Position is %ld\n",Set_Position);
		//	printf("请输入数据,以回车键结束\n"); 
			USART_RX_STA=0;
		}
	}	 

 
}	 
 



































