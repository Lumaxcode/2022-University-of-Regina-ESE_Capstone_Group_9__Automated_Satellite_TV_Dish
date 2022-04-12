#ifndef _GPS_H_
#define _GPS_H_

#include <stdint.h>

//##################################################################################################################

typedef struct
{
	uint8_t			UTC_Hour;
	uint8_t			UTC_Min;
	uint8_t			UTC_Sec;
	uint16_t		UTC_MicroSec;
	
	float				Latitude;
	double			LatitudeDecimal;
	char				NS_Indicator;
	float				Longitude;
	double			LongitudeDecimal;
	char				EW_Indicator;
	
	uint8_t			PositionFixIndicator;
	uint8_t			SatellitesUsed;
	float				HDOP;
	float				MSL_Altitude;
	char				MSL_Units;
	float				Geoid_Separation;
	char				Geoid_Units;
	
	uint16_t		AgeofDiffCorr;
	char				DiffRefStationID[4];
	char				CheckSum[2];	
	
}GPGGA_t;

typedef struct 
{
	uint8_t		rxBuffer[512];
	uint16_t	rxIndex;
	uint8_t		rxTmp;	
	uint32_t	LastTime;	
	
	GPGGA_t		GPGGA;
	
}GPS_t;
/*********Coordinate TypeDef*******/
typedef struct 
{
	double deg_lati , 
	      min_lati ,
			  min_lati_II, 
			  deg_long , 
			  min_long,
		  	min_long_II;
			
}Coo_t;//coordinate type
typedef struct //coordinate in a float form
{
	float La_De,
			  La_Min_De,
				Lo_De,
				Lo_Min_De;
				
}Coo_Cal;
extern GPS_t GPS;
//##################################################################################################################
void	GPS_Init(void);
void	GPS_CallBack(void);
//void	GPS_Process(void);
unsigned char GPS_ext(void); //Extracted coordinate in degree from Rx_buffer
Coo_Cal Ext_Dec(void);//Extracted data conversion to decimal
//##################################################################################################################

#endif
