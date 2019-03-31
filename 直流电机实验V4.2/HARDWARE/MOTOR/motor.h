#ifndef __MOTOR_H
#define __MOTOR_H



void TIM1_PWM_Init(u16 arr,u16 psc);
void TIM8_PWM_Init(u16 arr,u16 psc);
void Motor_pwm1_set(int pwm1);
void Motor_pwm2_set(int pwm2);
void Motor_pwm3_set(int pwm3);
void Motor_pwm4_set(int pwm4);
void Motor_pwm5_set(int pwm5);
void Motor_pwm6_set(int pwm6);
#endif
