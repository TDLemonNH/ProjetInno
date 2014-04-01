#include <iostream>
#include <wiringPi.h>

#include "PressureSensor/pressureSensor.hpp"

#define VALVE_IN 2
#define VALVE_OUT 3

using namespace std;

void initValves();

int main()
{
	PressureSensor sensor;
	
	const double profondeur = 30e-2; // Profondeur souhaitee en metres
	const double p0 = sensor.pressure();
	const double peq = p0 + profondeur*0.1; 
	
	do {
		
		
		double p = sensor.pressure();
		cout << p << endl; 
		
		if(p < peq)
		{
			// Relacher de l'air
			digitalWrite(VALVE_OUT, 1);
			delay(200);
			digitalWrite(VALVE_OUT, 0);
		}
		else
		{
			// Rajouter de l'air
			digitalWrite(VALVE_IN, 1);
			delay(10);
			digitalWrite(VALVE_IN, 0);
		}
		
	} while(true);
	
	return 0;
}

void initValves()
{
	pinMode (VALVE_IN, OUTPUT);
	pinMode (VALVE_OUT, OUTPUT);
	
	digitalWrite(VALVE_IN, 0);
	digitalWrite(VALVE_OUT, 0);
}
