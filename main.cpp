// g++ -o controller main.cpp PressureSensor/pressureSensor.cpp utils.cpp Observer/observer.cpp Controller/controller.cpp -lwiringPi -I /home/pi/Desktop/eigen
// g++ -o controller main.cpp Observer/observer.cpp Controller/controller.cpp utils.cpp -I "E:/Dropbox/Projet Inno veste plongeur/Eigen/"

#define RASPI
#define VALVE_IN 2
#define VALVE_OUT 3

#ifdef RASPI
	#include <wiringPi.h>
	#include "PressureSensor/pressureSensor.hpp"
#endif
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "Observer/observer.hpp"
#include "Controller/controller.hpp"
#include "utils.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

double valve_command_wished = 0;
double valve_command_actual = 0;
double dmaxin = 0.1;
double dmaxout = 0.05;

void logData(double t, double hm, MatrixXd est, double dest, double u, ofstream &log);

PI_THREAD (valve_cycle) {
	uint64_t current = getTimeStamp();
	uint64_t start = getTimeStamp();
	int cycle_time = 100000;
	double u = 0;
	
	while(true)
	{
		current = getTimeStamp();
		if ((current - start) > cycle_time)
		{
			digitalWrite(VALVE_OUT, 0);
			digitalWrite(VALVE_IN, 0);
			delay(10);
			
			piLock(0);
			u = valve_command_wished;
			valve_command_actual = u;
			piUnlock(0);
			
			if ( u > 0 )
			{
				digitalWrite(VALVE_IN, 1);
			}
			else if ( u < 0 )
			{
				digitalWrite(VALVE_OUT, 1);
			}
			start = current;
		}
	}
}	


int main(int argc, char* argv[])
{
	// Parse arguments
	int stepIncrement = 10000;
	for (int i = 1; i < argc; i++) {
		if (i + 1 != argc) // Check that we haven't finished parsing already
		{
			if (std::string(argv[i]) == "-timestep") 
			{
				stepIncrement = atoi(argv[i + 1]);
			} 
			else 
			{
				std::cout << "Not enough or invalid arguments, please try again.\n";
				exit(0);
			}
		}
		cout << argv[i] << endl;
	}
	cout << "Timestep: " << stepIncrement << endl;
	
	// Clear log
	ofstream log;
	log.open("log.txt", ios::trunc);
	if(!log.is_open())
		cout << "Error while opening log.txt";
	log << "t ," << "hm, " << "hest, " << "vest, " << "Nest, " << "dest, " << "actual" << "wished" << endl;
	
	// Define objects
	#ifdef RASPI
		PressureSensor sensor;
	#endif
    
    Observer obs;
	obs.start();
	
	Controller cont;
	
	int process = piThreadCreate(valve_cycle);
	if (process !=0)
	{
		cout << "erreur" << endl;
	}
	
	// Initialize valves
	pinMode (VALVE_IN, OUTPUT);
	pinMode (VALVE_OUT, OUTPUT);
	
	digitalWrite(VALVE_IN, 0);
	digitalWrite(VALVE_OUT, 0);
	
	// Start controller loop
	double hm; double dest; double u;
	uint64_t startT = getTimeStamp();
	uint64_t currentT = getTimeStamp();
	uint64_t previousT = getTimeStamp();
	uint64_t timer;
	while(true) 
	{
		currentT = getTimeStamp();
		if((currentT - previousT) > stepIncrement)
		{
			previousT = currentT;
			
			// Depth mesure
			timer = getTimeStamp();
				#ifdef RASPI
					hm = sensor.depth();
					cout << "hm: " << hm << endl;
				#else
					hm = 1;
				#endif
			timer = (getTimeStamp() - timer)*1e-3;
			cout << "Mesure time: " << timer << endl;
			
			timer = getTimeStamp();
				piLock(0);
					u = valve_command_actual;
				piUnlock(0);
				MatrixXd est = obs.step(hm, u);
			timer = (getTimeStamp() - timer)*1e-3;
			cout << "Obs time: " << timer << endl;
			
			MatrixXd stateest(3, 1);
			stateest << est(1,0), est(2,0), est(3,0);
			dest = est(4,0);
			
			// cout << "Stateest: " << endl << stateest << endl;
			
			timer = getTimeStamp();
			
				// u = cont.step(stateest, 0.15, dest);
				u = cont.step(stateest, 0.15, 0);
				
				piLock(0);
					valve_command_wished = u;
				piUnlock(0);
			
			timer = (getTimeStamp() - timer)*1e-3;
			cout << "Controller time: " << timer << endl;
			
			log << (currentT - startT)*1e-6 << ", " << hm << ", " << est(1, 0) << ", " << est(2, 0) << ", " << est(3, 0) << ", " << dest << ", " << valve_command_actual << ", " << valve_command_wished << endl;
		}
	}
	
	log.close();
}

void logData(double t, double hm, MatrixXd est, double dest, double u, ofstream &log)
{
	log << t << ", " << hm << ", " << est(0, 0) << ", " << est(1, 0) << ", " << est(2, 0) << ", " << dest << ", " << u << endl;
}
