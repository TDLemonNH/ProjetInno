// g++ -o controller main.cpp PressureSensor/pressureSensor.cpp utils.cpp Observer/observer.cpp Controller/controller.cpp -lwiringPi -I /home/pi/Desktop/eigen
// g++ -o controller main.cpp Observer/observer.cpp Controller/controller.cpp utils.cpp -I "E:/Dropbox/Projet Inno veste plongeur/Eigen/"

// #define RASPI

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

void logData(double t, double hm, MatrixXd est, double dest, double u, ofstream &log);

int main(int argc, char* argv[])
{
	// Parse arguments
	int stepIncrement = 100000;
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
	log << "t ," << "hm, " << "hest, " << "vest, " << "Nest, " << "dest, " << "u" << endl;
	
	// Define objects
	#ifdef RASPI
		PressureSensor sensor;
	#endif
    
    Observer obs;
	obs.start();
	
	Controller cont;
	
	// Start controller loop
	double hm; double dest; double u;
	uint64_t startT = getTimeStamp();
	uint64_t currentT = getTimeStamp();
	uint64_t previousT = getTimeStamp();
	while(true) 
	{
		currentT = getTimeStamp();
		if((currentT - previousT) > stepIncrement)
		{
			previousT = currentT;
			#ifdef RASPI
				hm = sensor.depth();
			#else
			    hm = 0.5;
			#endif
			
			MatrixXd est = obs.step(hm, cont.getU());
			
			MatrixXd stateest(3, 1);
			stateest << est(1,0), est(2,0), est(3,0);
			dest = est(4,0);
			
			cout << "Stateest: " << endl << stateest << endl;
			
			u = cont.step(stateest, 0.15, dest);
			
			logData((currentT - startT)*1e-6, hm, stateest, dest, u, log);
		}
	}
	
	log.close();
}

void logData(double t, double hm, MatrixXd est, double dest, double u, ofstream &log)
{
	log << t << ", " << hm << ", " << est(0, 0) << ", " << est(1, 0) << ", " << est(2, 0) << ", " << dest << ", " << u << endl;
}
