#ifndef __MOTOR_H
#define __MOTOR_H


void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void Motor_pwm1_set(int pwm1);
void Motor_pwm2_set(int pwm2);
#endif
