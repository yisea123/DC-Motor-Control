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
#include "i_pid.h"
#include "adc.h"
#include "can.h"
#include "DataScope_DP.h"

/************************************************
 ALIENTEK战舰STM32开发板实验8
 定时器中断实验
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/




/*******************************
更换成自制的点电机驱动板
硬件连接：  

电机轴向顺时针方向转动为正
 M1  C7      Encoder1 A1  A6     
 M2  C6               B1  A7
 M3  C9      Encoder2 A2  B6
 M4  C8               B2  B7
 M5  A11      Encoder3 A3  A0
 M6  A8              B3  A1

ADC1   C0
ADC2   C1
ADC3   C2

M1   编码器排线上Z 旁边的电源线
M2   排线红色一端的电源线


***************************/

/***************************
 本实验已基本实现单个直流电机位置速度双环控制，位置速度电流三环的控制尚不稳定
 ，需要调整PID参数与算法	

                  王鼎方
									——2019/2/27
****************************/
/********************************

 本实验基本实现了三个电机的位置速度控制 ,及单独的电流控制，效果不佳
 由于PWM输出占用了CAN引脚，需对CAN通讯引脚重定义
 应使用三套PID参数
 控制周期可重置，
 优化~~~~
 
 
                      王鼎方
											——2019/3/2




*******************************/
/*****************************
  调了一些小小的BUG，
	电机二电机三暂不确定是否可以稳定工作。
	电流环老样子~，尚未改进硬件电路。
	位置环可能是死 区的BUG，更换了容量 更大的自举电容（10UF ）后暂未出现问题。
//http://m.eepw.com.cn/article/201710/368839.html
   累死了，，，，，，，，，，，，，，，，，，，，，
           王鼎方
					 ——2019/3/8
*******************************/

/******************************
 加入了 TIM6 计时功能
 测得AD检测时间约为3*5=15次 需0.5ms ，但在实际使用中断时遇到问题，需设为18左右
 
 电流检测暂时还是放在while
 
 传输数据极其耗时
 
        王鼎方
				——2019/3/10


****************************/
#define VMAX  33 
#define MYLCD 1
#define MYUSART 0
#define MYUSB 0  
//CAN和USB在硬件上冲突，若要使能CAN，需CAN_RX0_INT_ENABLE置1，同时移除或修改USB的相关设置

int new_flag=0;//在串口通讯中作标置位

extern int Count1,Old_Could1,Count2,Old_Could2,Count3,Old_Could3;



volatile long Real_Position1=0;
volatile int now_velocity1=0;
volatile int now_current1;
int Initcurrent1;//初始电流  零点
volatile int set_velocity1=0;//用于参数传递
volatile int Set_Current1=0;//用于参数传递

volatile long Real_Position2=0;
volatile int now_velocity2=0;
volatile int now_current2;
int Initcurrent2;//初始电流  零点
volatile int set_velocity2=0;//用于参数传递
volatile int Set_Current2=0;//用于参数传递

volatile long Real_Position3=0;
volatile int now_velocity3=0;
volatile int now_current3;
int Initcurrent3;//初始电流  零点
volatile int set_velocity3=0;//用于参数传递
volatile int Set_Current3=0;//用于参数传递

extern int PID_v_add1;
extern int PID_i_add1;
extern int PID_i_add_set1;
extern int PID_P_add1; 
extern int PID_P_add_set1; 
extern int PID_v_add_set1; 

extern int PID_i_add2;
extern int PID_i_add_set2;
extern int PID_P_add2; 
extern int PID_P_add_set2; 
extern int PID_v_add_set2;

extern int PID_i_add3;
extern int PID_i_add_set3;
extern int PID_P_add3; 
extern int PID_P_add_set3; 
extern int PID_v_add_set3;


volatile int user_set_velocity1=8;//手动设定速度
volatile long user_Set_Position1=5000;//手动设定位置
volatile int user_Set_Current1=100;//设定电流大小，正负代表方向
                              

volatile int user_set_velocity2=0;//手动设定速度
volatile long user_Set_Position2=0;//手动设定位置
volatile int user_Set_Current2=100;


volatile int user_set_velocity3=0;//手动设定速度
volatile long user_Set_Position3=0;//手动设定位置
volatile int user_Set_Current3=100;
															 
int MODE=0;  //0 纯位置环  1 纯速度环 2 纯电流环  
             //3 位置速度  4 速度电流 5 位置速度电流 
						 //6 正交解码

int main(void)
 {	
	 u8 mo=0;//mo用于检测模式变化
	unsigned char i;          //计数变量
	unsigned char Send_Count; 
   int time=0;
	  delay_init();	    	 //延时函数初始化	  
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	  uart_init(115200);	 //串口初始化为115200
	  LED_Init();			     //LED端口初始化
	
//	 
	if(MYLCD)
	{
		LCD_Init();
		POINT_COLOR=BLUE;																		//字体颜色
		LCD_ShowString(30,10,100,16,16,"MODE: ");
		LCD_ShowString(30,120,200,16,16,"ADC_CH0_VAL:");	      
//  LCD_ShowString(60,190,200,16,16,"Send Data:");		//提示发送的数据	
//	LCD_ShowString(60,250,200,16,16,"Receive Data:");	//提示接收到的数据	
		LCD_ShowNum(150,10,MODE,4,16);
	
	}
	 													

	 Adc_Init();		
	 Initcurrent1=3100; //ADC的初始化与赋初值须在TIM3初始化之前
//	 Initcurrent2=Get_Adc_Average(ADC_Channel_2,100); 
//	 Initcurrent3=Get_Adc_Average(ADC_Channel_3,100); 
	 Initcurrent2=3100; 
	 Initcurrent3=3100; 
	 TIM1_PWM_Init(3559,0); //3599为占空比最大值
	 TIM8_PWM_Init(3559,0);
	
	 TIM2_Int_Init(14,7199);  
	 TIM6_Int_Init(999,7199);  
	// TIM7_Int_Init(10,7199);  
	 TIM3_Int_Init();    
	 TIM4_Int_Init();    
   TIM5_Int_Init();    
 	
//    Motor_pwm1_set(0);
//	  Motor_pwm2_set(1000);

	
	  mo=MODE;
				
  while(1)
	{
		
		
//    now_current1=Get_Adc_Average(ADC_Channel_10,5);  //AD采样,3*5=15次约需要0.5ms  	
//		now_current2=Get_Adc_Average(ADC_Channel_11,5);  
//		now_current3=Get_Adc_Average(ADC_Channel_12,5);
		                                                     
	  TIM_Cmd(TIM6, ENABLE);
    TIM6->CNT=0;
		time=0;
    DataScope_Get_Channel_Data( Real_Position1 , 1 );
		Send_Count = DataScope_Data_Generate(1);
		 for( i = 0 ; i < Send_Count; i++)  //循环发送,直到发送完毕   
	 	  {
		    while((USART1->SR&0X40)==0);  
  	    USART1->DR = DataScope_OutPut_Buffer[i]; //从串口丢一个字节数据出去      
		  }
			
      time=TIM6->CNT;
      TIM_Cmd(TIM6, DISABLE);
    
    	LCD_ShowString(30,60,200,16,16,"time:   ");
			LCD_ShowNum(150,60,time,6,16);
     //  delay_ms(1);
	}
 
}	 
 


//	  if(now_velocity==0)  //堵转保护,,,,,这种检测方式确实干涉了位置环，，，，导致了大量的超调
//		{                    //除非能做到一次到位不再调整，否则不可取
//		  delay_ms(5);
//			if(now_velocity==0)
//			{
//				MODE=6;
//				Motor_pwm1_set(0);
//	    	Motor_pwm2_set(0);
//			}
//		}
	
//		if(MYLCD)
//			{
//				if(mo!=MODE)
//						{
//							mo=MODE;
//							LCD_ShowNum(150,10,MODE,4,16);
//						}
//				LCD_ShowNum(150,10,MODE,4,16);
//						
//						
//						if(Real_Position1>=0)	
//				{					
//						LCD_ShowString(30,60,200,16,16,"Position1:   ");
//					  LCD_ShowNum(150,60,Real_Position1,6,16);
//				}
//					else
//					{
//						LCD_ShowString(30,60,200,16,16,"Position1: -");	
//						LCD_ShowNum(150,60,-Real_Position1,6,16);
//					}
//					
//				if(PID_i_add1>=0)	
//				{					
//						LCD_ShowString(30,80,200,16,16,"PID_i_add1:   ");
//					LCD_ShowNum(150,80,PID_i_add1,6,16);
//				}
//					else
//					{
//						LCD_ShowString(30,80,200,16,16,"PID_i_add1: -");	
//						LCD_ShowNum(150,80,-PID_i_add1,6,16);
//					}
//				
//				if(user_Set_Current1>=0)	
//				{					
//						LCD_ShowString(30,100,200,16,16,"user_Set_Current1:   ");
//					LCD_ShowNum(150,100,user_Set_Current1,6,16);
//				}
//					else
//					{
//						LCD_ShowString(30,100,200,16,16,"user_Set_Current1: -");	
//						LCD_ShowNum(150,100,-user_Set_Current1,6,16);
//					}

//    
//          if(Initcurrent1>=0)	
//				{					
//						LCD_ShowString(30,120,200,16,16,"Initcurrent1:   ");
//					  LCD_ShowNum(150,120,Initcurrent1,6,16);
//				}
//					else
//					{
//						LCD_ShowString(30,120,200,16,16,"Initcurrent1: -");	
//						LCD_ShowNum(150,120,-Initcurrent1,6,16);
//					}
//          if(now_velocity1>=0)	
//				{					
//						LCD_ShowString(30,140,200,16,16,"now_velocity1:   ");
//					  LCD_ShowNum(150,140,now_velocity1,6,16);
//				}
//					else
//					{
//						LCD_ShowString(30,140,200,16,16,"now_velocity1: -");	
//						LCD_ShowNum(150,140,-now_velocity1,6,16);
//					}

//					 if(now_current1>=0)	
//				{					
//						LCD_ShowString(30,160,200,16,16,"current:   ");
//					  LCD_ShowNum(150,160,now_current1,6,16);
//				}
//					else
//					{
//						LCD_ShowString(30,160,200,16,16,"current: -");	
//						LCD_ShowNum(150,160,-now_current1,6,16);
//					}
//          
//					 if(now_velocity1>=0)	
//				{					
//						LCD_ShowString(30,180,200,16,16,"now_velocity1:   ");
//					  LCD_ShowNum(150,180,now_velocity1,6,16);
//				}
//					else
//					{
//						LCD_ShowString(30,180,200,16,16,"now_velocity1: -");	
//						LCD_ShowNum(150,180,-now_velocity1,6,16);
//					}
//          if(PID_v_add_set1>=0)	
//				{					
//						LCD_ShowString(30,200,200,16,16,"PID_v_add_set1:   ");
//					  LCD_ShowNum(150,200,PID_v_add_set1,6,16);
//				}
//					else
//					{
//						LCD_ShowString(30,200,200,16,16,"PID_v_add_set1: -");	
//						LCD_ShowNum(150,200,-PID_v_add_set1,6,16);
//					}

//					 if(PID_v_add1>=0)	
//				{					
//						LCD_ShowString(30,220,200,16,16,"PID_v_add1:   ");
//					  LCD_ShowNum(150,220,PID_v_add1,6,16);
//				}
//					else
//					{
//						LCD_ShowString(30,220,200,16,16,"PID_v_add1: -");	
//						LCD_ShowNum(150,220,-PID_v_add1,6,16);
//					}
//			    LCD_ShowxNum(156,240,now_current1,4,16,0);//显示ADC的值
//					LCD_ShowxNum(156,260,now_current2,4,16,0);
//					
//					LCD_ShowxNum(156,280,now_current3,4,16,0);
//			    LCD_ShowxNum(20,240,Set_Current1,4,16,0);//显示ADC的值
//					LCD_ShowxNum(20,260,Set_Current2,4,16,0);
//					LCD_ShowxNum(20,280,Set_Current3,4,16,0);
//				}
//	



    // u8 res,ve,key=0;
	 // int cur;
   //u8 canbuf[8];
  //long  posi; 
//丢到while循环里
	//		key=Can_Receive_Msg(canbuf);
//		if(key)//CAN接收到有数据
//		{			
//			LCD_Fill(60,270,130,310,WHITE);//清除之前的显示
//			
// 			for(i=0;i<key;i++)
//			{									    
//				if(i<4)LCD_ShowxNum(60+i*32,270,canbuf[i],2,16,0X80);	//显示数据
//				else LCD_ShowxNum(60+(i-4)*32,290,canbuf[i],2,16,0X80);	//显示数据
// 			}
//			
//			res=Can_Send_Msg(canbuf,8);//发送8个字节 
//			if(res)LCD_ShowString(60+80,240,200,16,16,"Failed");		//提示发送失败
//			else LCD_ShowString(60+80,240,200,16,16,"OK    ");	 		//提示发送成功	
//			
//			
//			if(canbuf[0]==1)//模式控制
//				MODE=canbuf[1];
			

			
			
	//	}
	
	//返还数据  共8字节   
//0 	02    1 返回确认接收命令  2  返回相关数据
//1  	01    1 返回位置值·   2 速度值  电流值
//2	  01    1 正   2 负
//3	  00    低二位          这里发出去的是10进制 ，在上位机上显示的是16进制
//4	  00    从·低到高
//5	  00
//6	  00
//7	  00
//	
//	
//	
//			if(MODE==0||MODE==3||MODE==5)
//			{
//			  canbuf[0]=2;
//				canbuf[1]=1;
//				if(Real_Position>=0)
//				  canbuf[2]=1;
//				else
//					canbuf[2]=2;
//				posi=labs(Real_Position);
//			  canbuf[3]=(u8)(posi%100);
//				posi/=100;
//				canbuf[4]=(u8)(posi%100);
//				posi/=100;
//				canbuf[5]=(u8)(posi%100);
//				posi/=100;
//				canbuf[6]=(u8)(posi%100);
//				canbuf[7]=0;
//				Can_Send_Msg(canbuf,8);
//			}
//			
//				if(MODE==1||MODE==4)
//			{
//			  canbuf[0]=2;
//				canbuf[1]=2;
//				if(now_velocity>=0)
//				  canbuf[2]=1;
//				else
//					canbuf[2]=2;
//				vel=labs(now_velocity);
//			  canbuf[3]=vel;
//			  cur=now_current;
//				canbuf[4]=cur%100;
//				cur/=100;
//				canbuf[5]=cur%100;
//				canbuf[6]=0;
//				canbuf[7]=0;
//				Can_Send_Msg(canbuf,8);
//			}
//			
//			
//			
//			


































