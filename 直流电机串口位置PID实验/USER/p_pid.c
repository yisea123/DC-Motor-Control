
/****************************************

bug:   15000一以下





***************************************/








#include "timer.h"
#include "v_pid.h"
#include "motor.h"
#include "p_pid.h"

#define VMAX  33   //满占空比的速度

#define kp  3.089  //3.08
#define ki  0.006
#define kd  0   
struct
{
     float current_error;                  
     float last_error;                     
     float prev_error;                     
}PID_P;                                 
 int PID_P_add=0;
 int Max_V=VMAX;
extern int new_flag;
extern int set_velocity;
extern long Real_Position;
extern volatile long Set_Position;
void  Motor_ctl_Position()
{
  
    float P,I,D;
   
    PID_P.prev_error=PID_P.last_error;         
    PID_P.last_error=PID_P.current_error;  
	
			if(new_flag)	
				{
				PID_P.prev_error=0;
				PID_P.last_error=0;
				Max_V=VMAX;
				new_flag=0;
					
				}
				
	
	if(Set_Position- Real_Position>1000)
	{
    PID_P.current_error=0; 
		PID_P.prev_error=0;         
    PID_P.last_error=0; 
 Motor_ctl_Velocity(VMAX);		
	}
	else if(Set_Position- Real_Position<-1000)
	{
    PID_P.current_error=0; 
		PID_P.prev_error=0;         
    PID_P.last_error=0; 
  Motor_ctl_Velocity(-VMAX);		
	}
	
	else{
		
	 if(Set_Position- Real_Position>100||Set_Position- Real_Position<-100)
    PID_P.current_error=(Set_Position- Real_Position)/100 ; 
	  else	if(Set_Position- Real_Position>5||Set_Position- Real_Position<-5)
			{
			PID_P.current_error=(Set_Position- Real_Position)*3;   //*2一以下可以停下来，但误差大，*3很精确但电机抖动大，还是用多套PID参数好些
			Max_V=VMAX;
			}
			else
				{ 
					Max_V=0;
				//	PID_P.current_error=0 ;
				}		 
    P=kp*(PID_P.current_error-PID_P.last_error);
    I=ki*PID_P.current_error;     
    D=kd*(PID_P.current_error-(2*PID_P.last_error)+PID_P.prev_error);
    PID_P_add=PID_P_add+(int)(P+I+D);                

   
      if(PID_P_add>Max_V)
        PID_P_add=Max_V;                              
			 if(PID_P_add<-Max_V)
        PID_P_add=-Max_V;  
      Motor_ctl_Velocity(PID_P_add);
  
		 }
}





