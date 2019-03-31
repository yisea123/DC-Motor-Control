


#include "timer.h"
#include "v_pid.h"
#include "motor.h"
#include "p_pid.h"
#include <math.h>
#define VMAX  33   //满占空比的速度
extern int MODE; 

#define kp1  4
#define ki1  0  
#define kd1  0  
#define kp2  4
#define ki2  0  
#define kd2  0
#define kp3  4
#define ki3  0  
#define kd3  0
struct PID
{
     float current_error;                  
     float last_error;                     
     float prev_error;                     
}PID_P1,PID_P2,PID_P3; 

int PID_P_add1=0;
int Max_V=VMAX;
int PID_P_add_set1;
extern int new_flag;
extern volatile int set_velocity1;
extern volatile int user_set_velocity1;
extern long Real_Position1;
extern volatile long user_Set_Position1;


int PID_P_add2=0;
int PID_P_add_set2;
extern volatile int set_velocity2;
extern volatile int user_set_velocity2;
extern long Real_Position2;
extern volatile long user_Set_Position2;


int PID_P_add3=0;
int PID_P_add_set3;
extern volatile int set_velocity3;
extern volatile int user_set_velocity3;
extern long Real_Position3;
extern volatile long user_Set_Position3;



void  Motor_ctl_Position()
{
  
    float P1,I1,D1,P2,I2,D2,P3,I3,D3;
	
    PID_P1.prev_error=PID_P1.last_error;         
    PID_P1.last_error=PID_P1.current_error;  
	  PID_P1.current_error=user_Set_Position1- Real_Position1 ;  
    P1=kp1*(PID_P1.current_error-PID_P1.last_error);
    I1=ki1*PID_P1.current_error;     
    D1=kd1*(PID_P1.current_error-(2*PID_P1.last_error)+PID_P1.prev_error);
    PID_P_add1=PID_P_add1+(int)(P1+I1+D1); 
    PID_P_add_set1=PID_P_add1;
	
    PID_P2.prev_error=PID_P2.last_error;         
    PID_P2.last_error=PID_P2.current_error;  
	  PID_P2.current_error=user_Set_Position2- Real_Position2 ;  
    P2=kp2*(PID_P2.current_error-PID_P2.last_error);
    I2=ki2*PID_P2.current_error;     
    D2=kd2*(PID_P2.current_error-(2*PID_P2.last_error)+PID_P2.prev_error);
    PID_P_add2=PID_P_add2+(int)(P2+I2+D2);
    PID_P_add_set2=PID_P_add2;
	
    PID_P3.prev_error=PID_P3.last_error;         
    PID_P3.last_error=PID_P3.current_error;  
	  PID_P3.current_error=user_Set_Position3- Real_Position3 ;  
    P3=kp3*(PID_P3.current_error-PID_P3.last_error);
    I3=ki3*PID_P3.current_error;     
    D3=kd3*(PID_P3.current_error-(2*PID_P3.last_error)+PID_P3.prev_error);
    PID_P_add3=PID_P_add3+(int)(P3+I3+D3);			
    PID_P_add_set3=PID_P_add3;//为了不干扰PID的运行

	if(MODE==3||MODE==5)
	{
		 if(PID_P_add_set1>user_set_velocity1&&user_set_velocity1>0)
        PID_P_add_set1=user_set_velocity1;                              
		 else if(PID_P_add_set1<-user_set_velocity1&&user_set_velocity1>0)
        PID_P_add_set1=-user_set_velocity1;  
		 else if(PID_P_add_set1<user_set_velocity1&&user_set_velocity1<0)
        PID_P_add_set1=user_set_velocity1;                              
		 else if(PID_P_add_set1>-user_set_velocity1&&user_set_velocity1<0)
        PID_P_add_set1=-user_set_velocity1;  
		 else ;                            //用于在位置环中限制速度环的速度		 
	 
		 set_velocity1=PID_P_add_set1;
	
		 
		 if(PID_P_add_set2>user_set_velocity2&&user_set_velocity2>0)
        PID_P_add_set2=user_set_velocity2;                              
		 else if(PID_P_add_set2<-user_set_velocity2&&user_set_velocity2>0)
        PID_P_add_set2=-user_set_velocity2;  
		 else if(PID_P_add_set2<user_set_velocity2&&user_set_velocity2<0)
        PID_P_add_set2=user_set_velocity2;                              
		 else if(PID_P_add_set2>-user_set_velocity2&&user_set_velocity2<0)
        PID_P_add_set2=-user_set_velocity2;  
		 else ;                            //用于在位置环中限制速度环的速度		 
 
		 set_velocity2=PID_P_add_set2;
		 
		 if(PID_P_add_set3>user_set_velocity3&&user_set_velocity3>0)
        PID_P_add_set3=user_set_velocity3;                              
		 else if(PID_P_add_set3<-user_set_velocity3&&user_set_velocity3>0)
        PID_P_add_set3=-user_set_velocity3;  
		 else if(PID_P_add_set3<user_set_velocity3&&user_set_velocity3<0)
        PID_P_add_set3=user_set_velocity3;                              
		 else if(PID_P_add_set3>-user_set_velocity3&&user_set_velocity3<0)
        PID_P_add_set3=-user_set_velocity3;  
		 else ;                            //用于在位置环中限制速度环的速度		 
	
		 set_velocity3=PID_P_add_set3;
		 
		 
		 Motor_ctl_Velocity();
	}
	 if(MODE==0)
	{
	 if(PID_P_add_set1>0)
    {
      if(PID_P_add_set1>3500)
        PID_P_add_set1=3500;                              
			Motor_pwm1_set(0);
			Motor_pwm2_set(PID_P_add_set1);
    }     
   if(PID_P_add_set1<0)                                 
    {
     
      PID_P_add_set1=-PID_P_add_set1;                       
      if(PID_P_add_set1>3500)
        PID_P_add_set1=3500; 
			Motor_pwm1_set(PID_P_add_set1);
			Motor_pwm2_set(0);
    }
		
		if(PID_P_add_set2>0)
    {
      if(PID_P_add_set2>3500)
        PID_P_add_set2=3500;                              
			Motor_pwm3_set(0);
			Motor_pwm4_set(PID_P_add_set2);
    }     
   if(PID_P_add_set2<0)                                 
    {
     
      PID_P_add_set2=-PID_P_add_set2;                       
      if(PID_P_add_set2>3500)
        PID_P_add_set2=3500; 
			Motor_pwm3_set(PID_P_add_set2);
			Motor_pwm4_set(0);
    }
		
		if(PID_P_add_set3>0)
    {
      if(PID_P_add_set3>3500)
        PID_P_add_set3=3500;                              
			Motor_pwm5_set(0);
			Motor_pwm6_set(PID_P_add_set3);
    }     
   if(PID_P_add_set3<0)                                 
    {
     
      PID_P_add_set3=-PID_P_add_set3;                       
      if(PID_P_add_set3>3500)
        PID_P_add_set3=3500; 
			Motor_pwm5_set(PID_P_add_set3);
			Motor_pwm6_set(0);
    }
	}
			 
			 
}





