#ifndef FLOWSENSOR_H
#define FLOWSENSOR_H

#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <inttypes.h>
#include <math.h>
#include "../utils.hpp"


// sudo chmod o+rw /dev/i2c*
// i2cdetect -y 0/1

// Honeywell
// 2: SCL (blanc bleu) 3: Vdd (bleu) 4: GND (brun)     5: SDA (vert)
// Raspberry
// Vcc (P1)   ---
// SDA        ---
// SCL        GND

class FlowSensor
{
	
public:
	FlowSensor();
	~FlowSensor();
	
	double flow();
	short reverseByteOrder(short v);
	
private:
	int fd;



};

#endif // FLOWSENSOR_H
