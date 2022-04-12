#ifndef _SIGNAL_STRENGTH_H_
#define _SIGNAL_STRENGTH_H_
#include <stdbool.h>
#include "function.h"
#include  "stdint.h"
typedef struct strength 
{
	double sig82;
	double sig91;
	GPSang ang;
}strs;

bool sig_increase(strs test, strs last);
bool better_sig(strs test, strs ideal);
bool reverse_moter(bool rev);
bool delay_off(int counter);
strs load_str(uint16_t sti82,uint16_t sti91,GPSang ang);
strs copy(strs test);

#endif
