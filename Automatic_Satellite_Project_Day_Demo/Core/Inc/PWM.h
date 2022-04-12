#ifndef PWM_H_
#define PWM_H_
#include "stm32f103xb.h"
void compass_F(void);//forward
void compass_R(void);//reverse
void vertical_up(void);
void vertical_down(void);
void skew_R(void);//right 
void skew_L(void);//left
void Compass_Calibration(void);

#endif 
