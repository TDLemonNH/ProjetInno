#include <wiringPi.h>
#include <iostream>
#include "utils.hpp"

using namespace std;

PI_THREAD (valve_cycle) {
	uint64_t timer = getTimeStamp();
	while(true)
	{
		if((getTimeStamp() - timer) > 100000)
		{
			cout << "Hello world" << endl;
			timer = getTimeStamp();
		}
	}
}	
