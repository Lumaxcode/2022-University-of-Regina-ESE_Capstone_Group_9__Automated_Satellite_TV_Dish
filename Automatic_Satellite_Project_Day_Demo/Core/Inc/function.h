#ifndef _FUNCTION_H_
#define _FUNCTION_H_

#include "math.h"
#include "complex.h"
#include "bno055.h"

typedef struct GPS_angles
{
	double compass;
	double vertical;
	double skew;
} GPSang;
typedef struct location 
{	
	double lati;
	double longi;
	
} loc;
typedef struct degree_min
{
	double deg;
	double min;
	char a;
}DM;



loc MtoR(DM lati, DM longi);
GPSang cal_ang(loc dish1, loc satl);
GPSang load_ang(bno055_vector_t vg);
bool is_range(double test, double cal_ang, int ssd);
#endif
