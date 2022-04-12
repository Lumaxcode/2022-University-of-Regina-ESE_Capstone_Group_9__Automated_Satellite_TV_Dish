#include "GPSConfig.h"
#include "GPS.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

GPS_t GPS;
Coo_t CODNT;
Coo_Cal CODNT_Cal;
//unsigned char GPS_Ex[26];
//##################################################################################################################
double convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}
//##################################################################################################################
void	GPS_Init(void)
{
	GPS.rxIndex=0;
	HAL_UART_Receive_IT(&huart1,&GPS.rxTmp,1);	
}
//##################################################################################################################
void	GPS_CallBack(void)
{
	GPS.LastTime=HAL_GetTick();
	if(GPS.rxIndex < sizeof(GPS.rxBuffer)-2)
	{
		GPS.rxBuffer[GPS.rxIndex] = GPS.rxTmp;
		GPS.rxIndex++;
	}	
	HAL_UART_Receive_IT(&huart1,&GPS.rxTmp,1);
}
//##################################################################################################################
/*void	GPS_Process(void)
{
//	if( (HAL_GetTick()-GPS.LastTime>50) && (GPS.rxIndex>0))
//	{
//		char	*str;
//		#if (_GPS_DEBUG==1)
//		printf("%s",GPS.rxBuffer);
//		#endif
//		str=strstr((char*)GPS.rxBuffer,"$GPGGA,");
//		if(str!=NULL)
//		{
//			memset(&GPS.GPGGA,0,sizeof(GPS.GPGGA));
//			sscanf(str,"$GPGGA,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%hhd,%hhd,%f,%f,%c,%hd,%s,*%2s\r\n",&GPS.GPGGA.UTC_Hour,&GPS.GPGGA.UTC_Min,&GPS.GPGGA.UTC_Sec,
//			&GPS.GPGGA.UTC_MicroSec,&GPS.GPGGA.Latitude,&GPS.GPGGA.NS_Indicator,&GPS.GPGGA.Longitude,&GPS.GPGGA.EW_Indicator,&GPS.GPGGA.PositionFixIndicator,
//			&GPS.GPGGA.SatellitesUsed,&GPS.GPGGA.HDOP,&GPS.GPGGA.MSL_Altitude,&GPS.GPGGA.MSL_Units,&GPS.GPGGA.AgeofDiffCorr,GPS.GPGGA.DiffRefStationID,GPS.GPGGA.CheckSum);
//			if(GPS.GPGGA.NS_Indicator==0)
//				GPS.GPGGA.NS_Indicator='-';
//			if(GPS.GPGGA.EW_Indicator==0)
//				GPS.GPGGA.EW_Indicator='-';
//			if(GPS.GPGGA.Geoid_Units==0)
//				GPS.GPGGA.Geoid_Units='-';
//			if(GPS.GPGGA.MSL_Units==0)
//				GPS.GPGGA.MSL_Units='-';
//			GPS.GPGGA.LatitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Latitude);
//			GPS.GPGGA.LongitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Longitude);			
//		}		
//		memset(GPS.rxBuffer,0,sizeof(GPS.rxBuffer));
//		GPS.rxIndex=0;
//	}
//	HAL_UART_Receive_IT(&huart1,&GPS.rxTmp,1);
}
*/
//##################################################################################################################
//*******************************************************************************************************************
/*********User Defined Parameters**********/
  char GPS_Ex[26];// Extraction Buffer
char deg_lati_a[3], deg_long_a[4], min_lati_a[3], min_long_a[3],min_lati_a_II[6],min_long_a_II[6];// Attention: the length should be the data lenght + 1 for the Null in the last of the position
float La_De, La_Min_De,La_Min_II_De,
				Lo_De,Lo_Min_De,Lo_Min_II_De;
   
//*******************************************************************************************************************/
//GPS.rxBuffer size 512 [0~511]
//GPS.rxBuffer[19, 20, 21, 22, 23, 24, 25, 26, 27,........44]--GPRMC->26 members-->extract buffer GPS_Ex[26]
//GPS.rxBuffer[120, 133, 134,............................145]--GPGGA->26 members 
unsigned char GPS_ext(void){
	if( (HAL_GetTick()-GPS.LastTime>50) && (GPS.rxIndex>0))
	{
		char	*str;
		#if (_GPS_DEBUG==1)
		printf("%s",GPS.rxBuffer);
		#endif
		str=strstr((char*)GPS.rxBuffer,"$GPGGA,");
		if(str!=NULL)
		{
			memset(&GPS.GPGGA,0,sizeof(GPS.GPGGA));
			sscanf(str,"$GPGGA,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%hhd,%hhd,%f,%f,%c,%hd,%s,*%2s\r\n",&GPS.GPGGA.UTC_Hour,&GPS.GPGGA.UTC_Min,&GPS.GPGGA.UTC_Sec,
			&GPS.GPGGA.UTC_MicroSec,&GPS.GPGGA.Latitude,&GPS.GPGGA.NS_Indicator,&GPS.GPGGA.Longitude,&GPS.GPGGA.EW_Indicator,&GPS.GPGGA.PositionFixIndicator,
			&GPS.GPGGA.SatellitesUsed,&GPS.GPGGA.HDOP,&GPS.GPGGA.MSL_Altitude,&GPS.GPGGA.MSL_Units,&GPS.GPGGA.AgeofDiffCorr,GPS.GPGGA.DiffRefStationID,GPS.GPGGA.CheckSum);
			if(GPS.GPGGA.NS_Indicator==0)
				GPS.GPGGA.NS_Indicator='-';
			if(GPS.GPGGA.EW_Indicator==0)
				GPS.GPGGA.EW_Indicator='-';
			if(GPS.GPGGA.Geoid_Units==0)
				GPS.GPGGA.Geoid_Units='-';
			if(GPS.GPGGA.MSL_Units==0)
				GPS.GPGGA.MSL_Units='-';
			GPS.GPGGA.LatitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Latitude);
			GPS.GPGGA.LongitudeDecimal=convertDegMinToDecDeg(GPS.GPGGA.Longitude);			
		}		
		memset(GPS.rxBuffer,0,sizeof(GPS.rxBuffer));
		GPS.rxIndex=0;
	}
	HAL_UART_Receive_IT(&huart1,&GPS.rxTmp,1);
	if(GPS.rxBuffer[0]=='$'&&GPS.rxBuffer[0]!=0){ 	 
	for(int m=0;m<5;m++){
		GPS_Ex[0] = GPS.rxBuffer[19]; 
		GPS_Ex[1] = GPS.rxBuffer[20];
		GPS_Ex[2] = GPS.rxBuffer[21];
		GPS_Ex[3] = GPS.rxBuffer[22];
		GPS_Ex[4] = GPS.rxBuffer[23];
		GPS_Ex[5] = GPS.rxBuffer[24];
		GPS_Ex[6] = GPS.rxBuffer[25];
		GPS_Ex[7] = GPS.rxBuffer[26];
		GPS_Ex[8] = GPS.rxBuffer[27];
		GPS_Ex[9] = GPS.rxBuffer[28];
		GPS_Ex[10] = GPS.rxBuffer[29];
		GPS_Ex[11] = GPS.rxBuffer[30];//N Latitude
		GPS_Ex[12] = GPS.rxBuffer[31];//,
		GPS_Ex[13] = GPS.rxBuffer[32];
		GPS_Ex[14] = GPS.rxBuffer[33];
		GPS_Ex[15] = GPS.rxBuffer[34];
		GPS_Ex[16] = GPS.rxBuffer[35];
		GPS_Ex[17] = GPS.rxBuffer[36];
		GPS_Ex[18] = GPS.rxBuffer[37];
		GPS_Ex[19] = GPS.rxBuffer[38];
		GPS_Ex[20] = GPS.rxBuffer[39];
		GPS_Ex[21] = GPS.rxBuffer[40];
		GPS_Ex[22] = GPS.rxBuffer[41];
		GPS_Ex[23] = GPS.rxBuffer[42];
		GPS_Ex[24] = GPS.rxBuffer[43];
		GPS_Ex[25] = GPS.rxBuffer[44]; //W Longitude 
	}	  
	}	return *GPS_Ex;
}
  
//array to float 
//Coo_Cal stands for calculated coordinate
Coo_Cal Ext_Dec(void){
			deg_lati_a[0] = GPS_Ex[0];
			deg_lati_a[1] = GPS_Ex[1];
			min_lati_a[0] = GPS_Ex[2];
			min_lati_a[1] = GPS_Ex[3];
			min_lati_a_II[0] = GPS_Ex[5];
			min_lati_a_II[1] = GPS_Ex[6];
			min_lati_a_II[2] = GPS_Ex[7];
			min_lati_a_II[3] = GPS_Ex[8];
			min_lati_a_II[4] = GPS_Ex[9];
	
			deg_long_a[0] = GPS_Ex[13];
			deg_long_a[1] = GPS_Ex[14];
			deg_long_a[2] = GPS_Ex[15];
			min_long_a[0] = GPS_Ex[16];
			min_long_a[1] = GPS_Ex[17];
			min_long_a_II[0] = GPS_Ex[19];
			min_long_a_II[1] = GPS_Ex[20];
			min_long_a_II[2] = GPS_Ex[21];
			min_long_a_II[3] = GPS_Ex[22];
			min_long_a_II[4] = GPS_Ex[23];
	
		  CODNT.deg_lati = (double) atof(deg_lati_a);//Degreee
 	   	CODNT.min_lati = (double)atof(min_lati_a);//2 digit MIN 
 			CODNT.min_lati_II = (double)atof(min_lati_a_II);//After decimal point 5 digit
  		CODNT.deg_long =  (double)atof(deg_long_a); 
 		  CODNT.min_long = (double)atof(min_long_a); 
 			CODNT.min_long_II = (double)atof(min_long_a_II); 
			CODNT_Cal.La_De =CODNT.deg_lati ;
			CODNT_Cal.La_Min_De = CODNT.min_lati+((CODNT.min_lati_II)/100000);
			CODNT_Cal.Lo_De =CODNT.deg_long ;
			CODNT_Cal.Lo_Min_De = CODNT.min_long+((CODNT.min_long_II)/100000);
			printf("Your coordinate is:  %f Degree %f.%f Minutes North %f Degree %f.%f Minutes West \r\n", CODNT.deg_lati,CODNT.min_lati,CODNT.min_lati_II,CODNT.deg_long,CODNT.min_long,CODNT.min_long_II );	
			return CODNT_Cal; 
			
 }	
	
	
 
 
	 //int q=0; 
//    for(q = 0;q<2;++q)
//		{
//			deg_lati_a[q] = GPS_Ex[q];
//		}   
//		for(q = 0;q<3;++q)
//		{
//			deg_long_a[q] = GPS_Ex[q+13];
//		}  
//		for(q = 0;q<2;++q)//first two digit
//		{
//			min_lati_a[q] = GPS_Ex[q+2];
//			min_long_a[q] = GPS_Ex[q+16];
//	  }  
//		for(q = 0;q<5;++q)//last five digit
//		{ 
//			min_lati_a_II[q] = GPS_Ex[q+5];
//			min_long_a_II[q] = GPS_Ex[q+19];
//	  }  
		 
//			deg_lati =(float) atof(deg_lati_a);//Degreee
//			min_lati = atof(min_lati_a);//2 digit MIN 
//			min_lati_II = atof(min_lati_a_II);//After decimal point 5 digit
//			deg_long =(float) atof(deg_long_a); 
//			min_long = atof(min_long_a); 
//			min_long_II = atof(min_long_a_II); 
 	
