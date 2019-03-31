#include "timer.h"
#include "i_pid.h"
#include "motor.h"
extern int MODE; 
#define kp1  1.2
#define ki1  0.2
#define kd1  0   
#define kp2  1.2
#define ki2  0.2
#define kd2  0  
#define kp3  1.2
#define ki3  0.2
#define kd3  0  
struct PID
{
   float current_error;                  
   float last_error;                     
   float prev_error;                     
}PID_I1,PID_I2,PID_I3; 


int PID_i_add1=0;                         
int  PID_i_add_set1=0; 	
extern volatile int user_set_velocity1;
extern volatile int now_current1;
extern volatile int Set_Current1;
extern int Initcurrent1;


int PID_i_add2=0;                         
int  PID_i_add_set2=0; 	
extern volatile int user_set_velocity2;
extern volatile int now_current2;
extern volatile int Set_Current2;
extern int Initcurrent2;


int PID_i_add3=0;                         
int  PID_i_add_set3=0; 	
extern volatile int user_set_velocity3;
extern volatile int now_current3;
extern volatile int Set_Current3;
extern int Initcurrent3;



void Motor_ctl_Current()
{
  
    int P1,I1,D1,P2,I2,D2,P3,I3,D3;
    PID_I1.prev_error=PID_I1.last_error;         
    PID_I1.last_error=PID_I1.current_error;     
    PID_I1.current_error=Set_Current1-now_current1+Initcurrent1;               
    P1=(int)(kp1*(PID_I1.current_error-PID_I1.last_error));
    I1=(int)(ki1*PID_I1.current_error);     
    D1=(int)(kd1*(PID_I1.current_error-(2*PID_I1.last_error)+PID_I1.prev_error));
    PID_i_add1=PID_i_add1+(P1+I1+D1);   
	 if(PID_i_add1<0&&Set_Current1>=0)  //在电流环 给定为正值时不出现负的输出
		 PID_i_add1=0;
	  if(PID_i_add1>0&&Set_Current1<0)	//给定负值不出现正输出
		 PID_i_add1=0;
		
   if(PID_i_add1>3500)         //限幅
        PID_i_add1=3500;
			else if(PID_i_add1<-3500)	
				PID_i_add1=-3500;
			
			PID_i_add_set1=PID_i_add1;  //传参
			
   if(PID_i_add_set1>0&&Set_Current1>0)
    {                               
			  Motor_pwm1_set(0);
			  Motor_pwm2_set(PID_i_add_set1);
    }     
    else if(PID_i_add1<0&&Set_Current1<0)
    {           
			  PID_i_add_set1=-PID_i_add_set1;
			  Motor_pwm1_set(PID_i_add_set1);
			  Motor_pwm2_set(0);
    }     
		else
			{
    	Motor_pwm1_set(0);
			Motor_pwm2_set(0);
      }
		
		
		
		PID_I2.prev_error=PID_I2.last_error;         
    PID_I2.last_error=PID_I2.current_error;     
    PID_I2.current_error=Set_Current2-now_current2+Initcurrent2;                    
    P2=(int)(kp2*(PID_I2.current_error-PID_I2.last_error));
    I2=(int)(ki2*PID_I2.current_error);     
    D2=(int)(kd2*(PID_I2.current_error-(2*PID_I2.last_error)+PID_I2.prev_error));
    PID_i_add2=PID_i_add2+(P2+I2+D2);   
  if(PID_i_add2<0&&Set_Current2>=0)  //在电流环 给定为正值时不出现负的输出
		 PID_i_add2=0;
	  if(PID_i_add2>0&&Set_Current2<0)	//给定负值不出现正输出
		 PID_i_add2=0;
		
   if(PID_i_add2>3500)         //限幅
        PID_i_add2=3500;
			else if(PID_i_add2<-3500)	
				PID_i_add2=-3500;
			
			PID_i_add_set2=PID_i_add2;  //传参
			
   if(PID_i_add_set2>0&&Set_Current2>0)
    {                               
			  Motor_pwm3_set(0);
			  Motor_pwm4_set(PID_i_add_set2);
    }     
    else if(PID_i_add2<0&&Set_Current2<0)
    {           
			  PID_i_add_set2=-PID_i_add_set2;
			  Motor_pwm3_set(PID_i_add_set2);
			  Motor_pwm4_set(0);
    }     
		else
			{
    	Motor_pwm3_set(0);
			Motor_pwm4_set(0);
      }
			
		PID_I3.prev_error=PID_I3.last_error;         
    PID_I3.last_error=PID_I3.current_error;     
    PID_I3.current_error=Set_Current3-now_current3+Initcurrent3;                  
    P3=(int)(kp3*(PID_I3.current_error-PID_I3.last_error));
    I3=(int)(ki3*PID_I3.current_error);     
    D3=(int)(kd3*(PID_I3.current_error-(2*PID_I3.last_error)+PID_I3.prev_error));
    PID_i_add3=PID_i_add3+(P3+I3+D3);          
     if(PID_i_add3<0&&Set_Current3>=0)  //在电流环 给定为正值时不出现负的输出
		 PID_i_add3=0;
	  if(PID_i_add3>0&&Set_Current3<0)	//给定负值不出现正输出
		 PID_i_add3=0;
		
   if(PID_i_add3>3500)         //限幅
        PID_i_add3=3500;
			else if(PID_i_add3<-3500)	
				PID_i_add3=-3500;
			
			PID_i_add_set3=PID_i_add3;  //传参
			
   if(PID_i_add_set3>0&&Set_Current3>0)
    {                               
			  Motor_pwm5_set(0);
			  Motor_pwm6_set(PID_i_add_set3);
    }     
    else if(PID_i_add3<0&&Set_Current3<0)
    {           
			  PID_i_add_set3=-PID_i_add_set3;
			  Motor_pwm5_set(PID_i_add_set3);
			  Motor_pwm6_set(0);
    }     
		else
			{
    	Motor_pwm5_set(0);
			Motor_pwm6_set(0);
      }
   }
