#include "timer.h"
#include "v_pid.h"
#include "i_pid.h"
#include "motor.h"
extern int MODE; 
#define kp1  4      //3
#define ki1  0.2    //1
#define kd1 0   
#define kp2  4 
#define ki2  0.2
#define kd2 0   
#define kp3  4 
#define ki3  0.2
#define kd3 0   
struct PID
{
    int current_error;                  
    int last_error;                     
    int prev_error;                     
}PID_M1,PID_M2,PID_M3;                                 

extern volatile int Set_Current1;
extern volatile int user_Set_Current1;
extern int now_velocity1;
extern volatile int set_velocity1;
extern int Initcurrent1;
int PID_v_add1=0;
int PID_v_add_set1;


extern volatile int Set_Current2;
extern volatile int user_Set_Current2;
extern int now_velocity2;
extern volatile int set_velocity2;
extern int Initcurrent2;
int PID_v_add2=0;
int PID_v_add_set2;


extern volatile int Set_Current3;
extern volatile int user_Set_Current3;
extern int now_velocity3;
extern volatile int set_velocity3;
extern int Initcurrent3;
int PID_v_add3=0;
int PID_v_add_set3;

void Motor_ctl_Velocity()
{
  
     int P1=0,I1=0,D1=0,P2=0,I2=0,D2=0,P3=0,I3=0,D3=0;
    PID_M1.prev_error=PID_M1.last_error;         
    PID_M1.last_error=PID_M1.current_error;     
    PID_M1.current_error=set_velocity1- now_velocity1 ;               
    P1=(int)(kp1*(PID_M1.current_error-PID_M1.last_error));
    I1=(int)(ki1*PID_M1.current_error);     
    D1=(int)(kd1*(PID_M1.current_error-(2*PID_M1.last_error)+PID_M1.prev_error));
    PID_v_add1=PID_v_add1+(P1+I1+D1); 
	if(PID_v_add1>3500)
		PID_v_add1=3500;
	  else if(PID_v_add1<-3500)
		  PID_v_add1=-3500;
    PID_v_add_set1=PID_v_add1;
	  
	  PID_M2.prev_error=PID_M2.last_error;         
    PID_M2.last_error=PID_M2.current_error;     
    PID_M2.current_error=set_velocity2- now_velocity2 ;               
    P2=(int)(kp2*(PID_M2.current_error-PID_M2.last_error));
    I2=(int)(ki2*PID_M2.current_error);     
    D2=(int)(kd2*(PID_M2.current_error-(2*PID_M2.last_error)+PID_M2.prev_error));
    PID_v_add2=PID_v_add2+(P2+I2+D2);   
   if(PID_v_add2>3500)
		PID_v_add2=3500;
	  else if(PID_v_add2<-3500)
		  PID_v_add2=-3500;		
    PID_v_add_set2=PID_v_add2;
	
	  PID_M3.prev_error=PID_M3.last_error;         
    PID_M3.last_error=PID_M3.current_error;     
    PID_M3.current_error=set_velocity3- now_velocity3 ;               
    P3=(int)(kp3*(PID_M3.current_error-PID_M3.last_error));
    I3=(int)(ki3*PID_M3.current_error);     
    D3=(int)(kd3*(PID_M3.current_error-(2*PID_M3.last_error)+PID_M3.prev_error));
    PID_v_add3=PID_v_add3+(P3+I3+D3);
    if(PID_v_add3>3500)
		 PID_v_add3=3500;
	   else if(PID_v_add3<-3500)
		  PID_v_add3=-3500;		
    PID_v_add_set3=PID_v_add3;
	
	
	if(MODE==4||MODE==5)       
	{
    Set_Current1=PID_v_add_set1;
		if(Set_Current1>user_Set_Current1&&user_Set_Current1>0)   //根据设定电流值限定电流
			Set_Current1=user_Set_Current1;
		if(Set_Current1<-user_Set_Current1&&user_Set_Current1>0)
			Set_Current1=-user_Set_Current1;
		else if(Set_Current1<-user_Set_Current1&&user_Set_Current1<0)
        Set_Current1=-user_Set_Current1;                       
		 else if(Set_Current1>user_Set_Current1&&user_Set_Current1<0)
        Set_Current1=user_Set_Current1; 
		 else ;   
		 
		 Set_Current2=PID_v_add_set2;
		if(Set_Current2>user_Set_Current2&&user_Set_Current2>0)
			Set_Current2=user_Set_Current2;
		if(Set_Current2<-user_Set_Current2&&user_Set_Current2>0)
			Set_Current2=-user_Set_Current2;
		else if(Set_Current2<-user_Set_Current2&&user_Set_Current2<0)
        Set_Current2=-user_Set_Current2;                           
		 else if(Set_Current2>user_Set_Current2&&user_Set_Current2<0)
        Set_Current2=+user_Set_Current2; 
		 else ;   
		 

		 
		 Set_Current3=PID_v_add_set3;
		if(Set_Current3>user_Set_Current3&&user_Set_Current3>0)
			Set_Current3=user_Set_Current3;
		if(Set_Current3<-user_Set_Current3&&user_Set_Current3>0)
			Set_Current3=-user_Set_Current3;
		else if(Set_Current3<-user_Set_Current3&&user_Set_Current3<0)
        Set_Current3=-user_Set_Current3;                    
		 else if(Set_Current3>user_Set_Current3&&user_Set_Current3<0)
        Set_Current3=user_Set_Current3;  
		 else ;   
		 
		 
    Motor_ctl_Current();
	}
  else if(MODE==3||MODE==1)
   {
	 if(PID_v_add_set1>0)
    {                           
			  Motor_pwm1_set(0);
			  Motor_pwm2_set(PID_v_add_set1);
    }
      
   if(PID_v_add_set1<0)                                 
    {
			PID_v_add_set1=-PID_v_add_set1;         
			Motor_pwm1_set(PID_v_add_set1);
			Motor_pwm2_set(0);
    }
		
		 if(PID_v_add_set2>0)
    {
                                 
			  Motor_pwm3_set(0);
			  Motor_pwm4_set(PID_v_add_set2);
    }
      
   if(PID_v_add_set2<0)                                 
    {
     
      PID_v_add_set2=-PID_v_add_set2;                       
    
			Motor_pwm3_set(PID_v_add_set2);
			Motor_pwm4_set(0);
    }
		
		 if(PID_v_add_set3>0)
    {
                                
			  Motor_pwm5_set(0);
			  Motor_pwm6_set(PID_v_add_set3);
    }
      
   if(PID_v_add_set3<0)                                 
    {
     
      PID_v_add_set3=-PID_v_add_set3;                       
    
			Motor_pwm5_set(PID_v_add_set3);
			Motor_pwm6_set(0);
    }
	}
}
