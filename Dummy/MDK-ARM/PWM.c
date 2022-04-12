#include "PWM.h"
#include "stm32f1xx_hal.h"
//TIM3 channel 1, logic control PA0
void compass_F(void){   
	//GPIOA->ODR&=~GPIO_ODR_ODR0;  
	    
	TIM3->CCR1 = 35; 
	HAL_Delay(700);
	TIM3->CCR1 = 0;	
	HAL_Delay(500);
}

void compass_R(void){
for (int i=0;i<500;i++){
	TIM3->CCR1 = 0;	
	GPIOA->ODR |=GPIO_ODR_ODR0; 
	}   
	TIM3->CCR1 = 35;  
	HAL_Delay(700);
	TIM3->CCR1 = 0;	
	HAL_Delay(500);
  GPIOA->ODR&=~GPIO_ODR_ODR0; 
}
//TIM3 channel 2, logic control PA1
void vertical_up(void){    	
	TIM3->CCR2 = 30;
  HAL_Delay(1000);
	TIM3->CCR1 = 0;	
  HAL_Delay(500);
 
}
void vertical_down(void){   
	for (int i=0;i<500;i++){
	TIM3->CCR2 = 0;	
	GPIOA->ODR |=GPIO_ODR_ODR1; 
	}   
	
	TIM3->CCR2 = 30;
  HAL_Delay(1000);
	TIM3->CCR1 = 0;	
  HAL_Delay(500); 
	GPIOA->ODR&=~GPIO_ODR_ODR1;  
}
//TIM3 channel 3, logic control PA4
void skew_R(void){     
	 
  TIM3->CCR3 = 35; 
	HAL_Delay(500);
	TIM3->CCR3 = 0;	
	HAL_Delay(500); 
}
void skew_L(void){
	for (int i=0;i<500;i++){
	TIM3->CCR3 = 0;	
	GPIOA->ODR |=GPIO_ODR_ODR4; 
	}   
	TIM3->CCR3 = 35; 
	HAL_Delay(500);
	TIM3->CCR3 = 0;	
	HAL_Delay(500);  
	GPIOA->ODR&=~GPIO_ODR_ODR4;  
}
 
	
