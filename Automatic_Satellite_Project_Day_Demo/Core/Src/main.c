/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	* @Programmer     : Mingxiao Li
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"   
#include "GPS.h"  
#include "bno055.h"
#include "bno055_stm32.h"
#include "PWM.h"
#include "function.h" 
#include "signal_strength.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	Coo_Cal Coordinate;//GPS type 
 	bno055_vector_t vmg;// mag type
  bno055_vector_t vag;//Acc type
	bno055_vector_t vg,vgy;
	bno055_calibration_state_t cs;
	bno055_calibration_data_t cd;
	Radians thetaphi;
	tilt_YX tilted;
	DM la, lo; 
	loc dish;
	loc sat_82,
				sat_91, 
				sat_91_82; 
	GPSang angs;//GPS_Angles 
	double ini_set = 30;//the largest angle thet the skew moter can go
  double Compass=0;
	double Xg=0,Yg=0,Zg=0;  
	 
	 
	bool is_cmps = false;
	bool is_vert = false;
	bool is_skew = false;
	
	/*********Signal Strength********/
	strs test_s;// the signal strength after move
	strs last_s;// the signal strength before move
	strs best_s;// the best signal strength the dish have
	GPSang test_ang;//the 3 angles for current dish position

	bool is_str_test = false;//we need to enter the signal strength test
	int cal_ang_read = 0;
	
	bool cmps_finish = false;
	bool cmps_reverse = false;
	bool cmps_increase = false;
	bool cmps_inc_delay = false;
	bool cmps_delay_off = false;
	bool is_cmps_in_range = false;
	int cmps_inc_counter = 0;
	int cmps_counter = 0;

	bool vert_finish = false;
	bool vert_reverse = false;
	bool vert_increase = false;
	bool vert_inc_delay = false;
	bool vert_delay_off = false;
	bool is_vert_in_range = false;

	int vert_inc_counter = 0;
	int vert_counter = 0; 

	
	uint16_t Nimq82,Nimq91;//adc variables
  
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
 {
  /* USER CODE BEGIN 1 */
	//Shuo
	/*********************************GPS Angle Variables*********************************/
	//Nimiq 82 
		double pie = 3.1415926;
	  sat_82.lati = 0;
		sat_82.longi = -82*pie/180;;  
	//Nimiq 91
	  sat_91.lati = 0;
	  sat_91.longi = -91*pie/180;
		sat_91_82.lati = 0;
	sat_91_82.longi = (sat_91.longi+sat_82.longi)/2;
	/*********calculation_&_moves********/

	//strength
	/****************************************************************************/
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	/*******************I/O Initializations**************/ 
	/**************GPS Initialization************/
	GPS_Init();		 //Compass Initialization   
	/*********************Adafruit**********************/ 
	bno055_assignI2C(&hi2c1);
	bno055_setup(); 
  bno055_setOperationModeCOMPASS();
   
	/*****************Motors PWM Control Init***** ***/ 
 	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
 	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
 	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	 
	/********Parse and Import GPS Coordinate */
		 
  for(int i=0;i<100;i++) 
	{
		GPS_ext();
		Coordinate = Ext_Dec(); 	
	}
			 
	la.deg = Coordinate.La_De;
	la.min = Coordinate.La_Min_De;
	la.a = 'n';
	lo.deg = Coordinate.Lo_De;
	lo.min = Coordinate.Lo_Min_De;
	lo.a = 'w';
	dish = MtoR(la, lo);//minute degree to decimal degress
// 			 dish.lati= 50.40758696958*3.141592654/180;
//   dish.longi=-104.6602968666*3.141592654/180;
	angs = cal_ang(dish, sat_91_82 );
	printf("Calibrating.......");
	Compass_Calibration();
	TIM3->CCR1=0;
	HAL_Delay(5000); 
  cd=bno055_getCalibrationData();
	HAL_Delay(1000);
  bno055_setCalibrationData(cd);
	HAL_Delay(5000);
	cs=bno055_getCalibrationState();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	 	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/*****ADC 82 91 signal strength********/
		ADC_Select_CH10();
	  HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,1000);
		Nimq82=HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1); 
		ADC_Select_CH11();
	  HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,1000);
		Nimq91=HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		//3.07V  98% strength for nimq 82
		//3.3V/4096 * value= real value 
		/***************Compass_3Axis**************Saved to CMPS**********/
 
		vmg= bno055_getVectorMagnetometer();
    vag= bno055_getVectorAccelerometer();  	
    vg= bno055_getVectorEuler();  
 	  Xg=vg.xg;//yaw
 	  Yg=vg.yg;//roll
 	  Zg=90-(vg.zg);//pitch
		thetaphi=theta_phi_rad(vag.xg, vag.yg, vag.zg);
		tilted=normalized_azimuth(vmg.xg, vmg.yg, vmg.zg, thetaphi.theta, thetaphi.phi);
		Compass=(tilted.Azimuth)+18; 
//  		printf("True North Azimuth = %d \r\n",Compass);  
	   //printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", vg.xg, vg.yg, vg.zg);
		  
 		/*****************calculation_&_moves*****************************************/
		//50.40758696958, -104.6602968666
		
		//50 24.4524307N, 104 39.6133385W
		//if not achieve the calculating angles, moters move  
	if((Compass>((angs.compass)-3)&&Compass<((angs.compass)+3))&&
		(Yg>((angs.vertical)-2)&&Yg<((angs.vertical)+2))&&(Zg>((angs.skew)-1)&&Zg<((angs.skew)+1))){
			if(cmps_finish && vert_finish)
			{
					   is_str_test = true; 
			}
			else { is_str_test = true; }
		  
		 ADC_Select_CH10();
	   HAL_ADC_Start(&hadc1);
	   HAL_ADC_PollForConversion(&hadc1,1000);
	   Nimq82=HAL_ADC_GetValue(&hadc1);
		 HAL_ADC_Stop(&hadc1); 
		 ADC_Select_CH11();
		 HAL_ADC_Start(&hadc1);
		 HAL_ADC_PollForConversion(&hadc1,1000);
		 Nimq91=HAL_ADC_GetValue(&hadc1);
		 HAL_ADC_Stop(&hadc1); 
		 test_s = load_str(Nimq82, Nimq91, angs);
		 best_s = load_str(Nimq82, Nimq91, angs);
		
					/*****************************signal strength test********************************************/
		if(is_str_test)
		{
			//*********compass*********//
			if (!cmps_finish)
			{ 
				last_s = copy(test_s);//copy the angles, signals for compare
				//compass motor move for signal chack
				if (!cmps_reverse)	{ TIM3->CCR1 = 25; 
															HAL_Delay(300);
															TIM3->CCR1 = 0;	
															HAL_Delay(1000);
															Compass=(tilted.Azimuth)+18; }
				else 								{   TIM3->CCR1 = 0;	
													      GPIOA->ODR |=GPIO_ODR_ODR0;    
															  TIM3->CCR1 = 25;  
															  HAL_Delay(300);
															  TIM3->CCR1 = 0;	
															  HAL_Delay(1000);
															  GPIOA->ODR&=~GPIO_ODR_ODR0;  
																Compass=(tilted.Azimuth)+18; }
				
				if (cal_ang_read == 0)
				{					
				//Load ADC data from CH10 and CH11
				ADC_Select_CH10();
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1,1000);
				Nimq82=HAL_ADC_GetValue(&hadc1);
				HAL_ADC_Stop(&hadc1); 
				ADC_Select_CH11();
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1,1000);
				Nimq91=HAL_ADC_GetValue(&hadc1);
				HAL_ADC_Stop(&hadc1);
				//Read compass angle from Eular 
				vg = bno055_getVectorEuler();
				test_ang = load_ang(vg);
				test_s = load_str(Nimq82, Nimq91, test_ang);
				cal_ang_read++;
				}
				cmps_increase = sig_increase(test_s, last_s);//bool
				cmps_inc_delay = delay_off(cmps_inc_counter);//bool
				HAL_Delay(500);
				//if signal for compass moving didn't increase
				if (!cmps_increase)
				{
					//2 chance for not increase chacking move or reverse motor
					if (!cmps_inc_delay) { cmps_inc_counter++; }
					else 
					{ 
						cmps_reverse = reverse_moter(cmps_reverse); 
						cmps_inc_counter = 0;
					
						cmps_delay_off = delay_off(cmps_counter);//bool
						if (cmps_delay_off) 		 { cmps_finish = true; printf("cmps finish /r/n");}
						else if (cmps_inc_delay) { cmps_counter++; }
						else										 {}
					}//cmps_inc_delay
				}//!cmps_increase
				
				// signal for compass moving increase
				else
				{
					//3 chance head shacking or finish compass 
					if(!cmps_delay_off)
					{
						if (better_sig(test_s, best_s)){best_s = copy(test_s);printf("The best C reload /r/n");}
						else  												 {printf("not load /r/n");}
					}//!cmps_delay_off
					else
					{
						//compass motor move for achieve the best signal
						if (Compass<((best_s.ang.compass)-1)||Compass>((best_s.ang.compass)+1))
						{
							HAL_Delay(500); 
							if(Compass<(best_s.ang.compass-1)) { 
							TIM3->CCR1 = 25; 
							HAL_Delay(300);
							TIM3->CCR1 = 0;	
							HAL_Delay(1000); Compass=(tilted.Azimuth)+18;}
							else    										 			 { 
							TIM3->CCR1 = 0;	
							GPIOA->ODR |=GPIO_ODR_ODR0;    
							TIM3->CCR1 = 25;  
							HAL_Delay(300);
							TIM3->CCR1 = 0;	
							HAL_Delay(1000);
							GPIOA->ODR&=~GPIO_ODR_ODR0;Compass=(tilted.Azimuth)+18;}
						}
						else { TIM3->CCR1 = 0; is_cmps_in_range = true;}
					}//cmps_delay_off
				}//cmps_increase
			}//! compass_finish
			else 
			{
				TIM3->CCR1 = 0;
				cmps_finish = true;
			//*********vertical*********//
			if (!vert_finish)
			{
				last_s = copy(test_s);//copy the angles, signals
				if (!vert_reverse) { vertical_up();  }
				else 							 { vertical_down();  }
				if (cal_ang_read == 1)
				{				
				//Load ADC data from CH10 and CH11
				ADC_Select_CH10();
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1,1000);
				Nimq82=HAL_ADC_GetValue(&hadc1);
				HAL_ADC_Stop(&hadc1); 
				ADC_Select_CH11();
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1,1000);
				Nimq91=HAL_ADC_GetValue(&hadc1);
				HAL_ADC_Stop(&hadc1);
				//Read Vertical angle from Eular 
				vg = bno055_getVectorEuler();
				test_ang = load_ang(vg);
				test_s = load_str(Nimq82, Nimq91, test_ang);
				cal_ang_read++;
				}
				vert_increase = sig_increase(test_s, last_s);//bool
				vert_inc_delay = delay_off(vert_inc_counter);//bool
				HAL_Delay(500);
				if (!vert_increase)
				{
					if (!vert_inc_delay) { vert_inc_counter++; }
					else 
					{ 
						vert_reverse = reverse_moter(vert_reverse); 
						vert_inc_counter = 0;
					
						vert_delay_off = delay_off(vert_counter);//bool
						if (vert_delay_off) 		 { vert_finish = true; printf("vert finish /r/n");}
						else if (vert_inc_delay) { vert_counter++; }
						else										 {}
					}//vert_inc_delay
				}//!vert_finish
				else
				{
					if (!vert_delay_off)
					{
						if (better_sig(test_s, best_s)){best_s = copy(test_s); printf("The best v reload /r/n");}
						else  												 {printf("not load /r/n");}
					}
					else
					{
						//vertical motor move
						if (Yg<((best_s.ang.vertical)-1)||Yg>((best_s.ang.vertical)+1))
						{
							HAL_Delay(500); 
							if(Yg<(angs.vertical-1)) { vertical_up(); }
							else    								 { vertical_down(); }
						}
						else { TIM3->CCR2=0; is_vert_in_range = true;}
					}//vert_delay_off
				}//vert_increase
			}
			else 
			{
				TIM3->CCR1=0;
				TIM3->CCR2=0;
				break;
			}//vert_finish
			}//cmps_finish
		}//is_str_test
		}
		else
		{
			if (Zg<((angs.skew)-1)||Zg>((angs.skew)+1)){ 
			HAL_Delay(500);
			//skew motor move
			if (Zg<((angs.skew)-1)){
			skew_R(); 
			}else 
			skew_L();
			} 
				else{		
				TIM3->CCR3=0;
				//vertical motor move
				if (Yg<((angs.vertical)-2)||Yg>((angs.vertical)+2)){
			  HAL_Delay(500);
				if (Yg<((angs.vertical)-2)){
				vertical_up();
				}else
				vertical_down(); 
			}
				else{
					TIM3->CCR2=0; 
					//compass motor move 
					if (Compass<((angs.compass)-3)||Compass>((angs.compass)+3)){
					HAL_Delay(500); 
					if(Compass<(angs.compass)-3){
					compass_F();
					Compass=(tilted.Azimuth)+18;
					}
					else compass_R();
					Compass=(tilted.Azimuth)+18; 
					}else
					TIM3->CCR1 = 0;  	} }
    }
 
		
	}//while loop bracket
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	GPS_CallBack();
	/* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
  the HAL_UART_RxCpltCallback could be implemented in the user file
  */ 
		
	 //HAL_UART_Transmit(&huart2, &GPS.rxTmp, 1, 1000); 
	   
}
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit (&huart2 ,(uint8_t *)&ch,1,HAL_MAX_DELAY );
	return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

