#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H

#include <iostream>
#include <wiringPi.h>
#include <inttypes.h>
#include <math.h>
#include "../utils.hpp"

#define SCLK 7
#define DIN 12
#define DOUT 13
#define CSB 14
#define SCLK_PERIODE 10 // us

class PressureSensor
{
	
public:
	PressureSensor();
	~PressureSensor();
	
	void reset();
	double pressure();
	double depth();
	
private:
	// I/O
	void setDIN(bool state);
	char getDOUT();
	void setSCLK(bool state);
	void waitOnePulse();
	void wakeUp();
	
	void writeWord(char pattern);
	uint32_t readWord(int size);
	
	double computeTP(uint32_t d1, uint32_t d2, uint32_t *constants);
	
	unsigned char checkSum(uint32_t sensorCoefficients[]);
	
	char reverseBitOrder(char data);

private:
	uint32_t mConstants[8];
	double mP0;

};

#endif // PRESSURESENSOR_H
