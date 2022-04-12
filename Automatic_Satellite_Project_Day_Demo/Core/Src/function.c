#include "function.h"

double la1, la2, lo1, lo2;
double pi = 3.1415926;
loc MtoR(DM lati, DM longi)
{
	loc loc1;
	loc1.lati = lati.deg + lati.min * 1.6667 * 0.01;// minute to degree
	loc1.longi = (longi.deg + longi.min * 1.6667 * 0.01);
	
	if (lati.a =='s' || lati.a == 'S')
	{
		loc1.lati = -loc1.lati;
	}
	if (longi.a =='w' ||longi.a == 'W')
	{
		loc1.longi = -loc1.longi;
	}
	loc1.lati = loc1.lati*pi/180;
	loc1.longi = loc1.longi*pi/180;
	return loc1;
}

GPSang cal_ang(loc dish1, loc satl)
{
	GPSang ang;
	
	// satellite coordinate
	la1 = satl.lati;
	lo1 = satl.longi;
	//dish coordinate 
	la2 = dish1.lati;
	lo2 = dish1.longi;
	//the angles we  need to set on the dish 
	ang.compass = 180+(atan((tan(lo2 - lo1))/ (sin(la2)))*(180/pi));
	
	ang.vertical = atan((cos(lo2 - lo1) * cos(la2) - 0.1513) 
	/ sqrt(1 - pow(cos(lo2 - lo1) * cos(la2), 2)))*(180/pi);
	
	ang.skew =90+(atan(sin(lo2 - lo1) / tan(la2))*(180/pi));
	return ang;
}
GPSang load_ang(bno055_vector_t vg)
{
	GPSang ang;
	ang.compass = vg.xg;
	ang.vertical = vg.yg;
	ang.skew = vg.zg;
	return ang;
}


