#ifndef FLOWSENSOR_H
#define FLOWSENSOR_H

#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <inttypes.h>
#include <math.h>
#include "../utils.hpp"

class FlowSensor
{
	
public:
	FlowSensor();
	~FlowSensor();
	
	double flow();
	
private:
	int fd;



};

#endif // FLOWSENSOR_H
