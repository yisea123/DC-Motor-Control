#include "timer.h"
#include "v_pid.h"
#include "motor.h"

#define kp  30 
#define ki  1
#define kd  0   
struct
{
    short int current_error;                  
    short int last_error;                     
    short int prev_error;                     
}PID_M;                                 
short int PID_m_add=0;                         
extern int now_velocity;


void Motor_ctl_Velocity(int Setvelocity)
{
  
    short int P,I,D;
   int  pwm3; 	
    PID_M.prev_error=PID_M.last_error;         
    PID_M.last_error=PID_M.current_error;     
    PID_M.current_error=Setvelocity- now_velocity ;               
    P=(short int)(kp*(PID_M.current_error-PID_M.last_error));
    I=(short int)(ki*PID_M.current_error);     
    D=(short int)(kd*(PID_M.current_error-(2*PID_M.last_error)+PID_M.prev_error));
    PID_m_add=PID_m_add+(P+I+D);                

   if(PID_m_add>0)
    {
      if(PID_m_add>3500)
        PID_m_add=3500;                              
			  Motor_pwm1_set(PID_m_add);
			  Motor_pwm2_set(0);
    }
      
   if(PID_m_add<0)                                 
    {
     
      pwm3=-PID_m_add;                       
      if(pwm3>3500)
      pwm3=3500; 
			Motor_pwm1_set(0);
			Motor_pwm2_set(pwm3);
    }
   
   }
