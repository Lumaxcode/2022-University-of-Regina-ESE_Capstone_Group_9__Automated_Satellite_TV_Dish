#include "signal_strength.h"

bool sig_increase(strs test, strs last)
{
	if (test.sig82 + test.sig91 > last.sig82 + last.sig91) { return true; }
	else { return false; }
}

bool better_sig(strs test, strs ideal)
{
	if ((test.sig82 + test.sig91) > (ideal.sig82 + ideal.sig91)) { return true; }
	else { return false; }
}

bool reverse_moter(bool rev)
{
	if (rev) { return false; }
	else { return true; }
}

bool delay_off(int counter)
{
	if (counter < 1) { return false; }
	else { return true; }
}

strs load_str(uint16_t Nimq82,uint16_t Nimq91,GPSang ang)
{
	strs test;
		test.sig82=Nimq82;
		test.sig91=Nimq91;
		test.ang.compass = ang.compass;
		test.ang.vertical = ang.vertical;
		test.ang.skew = ang.skew;
	return test;
}
strs copy(strs test)
{
	strs last;
	last.sig82 = test.sig82;
	last.sig91 = test.sig91;
	last.ang.compass = test.ang.compass;
	last.ang.vertical = test.ang.vertical;
	last.ang.skew = test.ang.skew;
	return last;
}
