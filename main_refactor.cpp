// g++ -o controller main_refactor.cpp PressureSensor/pressureSensor.cpp FlowSensor/flowSensor.cpp utils.cpp Observer/observer.cpp Controller/controller.cpp -lwiringPi -I /home/pi/Desktop/eigen

#define VALVE_IN 2
#define VALVE_OUT 3

#include <wiringPi.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <Eigen/Dense>
#include <math.h>

#include "PressureSensor/pressureSensor.hpp"
#include "FlowSensor/flowSensor.hpp"
#include "Observer/observer.hpp"
#include "Controller/controller.hpp"
#include "ObsConLoop/obsconloop.hpp"
#include "utils.hpp"

using namespace std;
using namespace Eigen;

double HR = 1; // Desired depth (command)

// Global variables for thread communication
double valve_command_wished = 0;
double valve_command_actual = 0;
double hmforvalve;
double dmaxin = 0.026;
double dmaxout = 0;

// Valve thread definition
#include "valvecycle.hpp"

// Function declarations (definitions after the main function)
void initialize();
void loadConfigData(string fileName, Observer& observer, Controller& controller);


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
	
	// Clear log and write header (csv format)
	ofstream log;
	log.open("log.txt", ios::trunc);
	if(!log.is_open())
		cout << "Error while opening log.txt";
	log << "t ," << "hm, " << "hest, " << "vest, " << "Nest, " << "flow, " << "dest, " << "actual," << "wished" << endl;
	
	// Init GPIO pins
	initialize();
	
	// Define objects
	PressureSensor sensor;
	if(!sensor.reset())
		return 0;
		
	FlowSensor flowSensor;
    
    Observer obs;
	obs.start();
	
	Controller cont;
	
	loadConfigData("config.csv", obs, cont);
	
	// Start valve command thread
	int process = piThreadCreate(valve_cycle);
	if (process != 0)
	{
		cout << "Error while creating piThread." << endl;
	}
	
	// Start controller loop
	double hm; double dest; double u; double flow;
	uint64_t startT = getTimeStamp();
	uint64_t currentT = getTimeStamp();
	uint64_t previousT = getTimeStamp();
	uint64_t timer; // For timing, see use below
	
	while(true) 
	{
		currentT = getTimeStamp();
		
		if((currentT - previousT) > stepIncrement) // Step increment can be provided as an argument with -timestep
		{
			previousT = currentT;
			
			// Depth measure
			timer = getTimeStamp();
				hm = sensor.depth();
				cout << "hm: " << hm << endl;
			timer = (getTimeStamp() - timer)*1e-3;
			// cout << "Measure time: " << timer << endl;
			
			// Flow measure
			flow = flowSensor.flow();
			cout << "flow: " << flow << endl;
			
			// Step observer, using the measured depth and the valve command
			timer = getTimeStamp();
				piLock(0);
					u = valve_command_actual;
				piUnlock(0);
				MatrixXd est = obs.step(hm, u);
			timer = (getTimeStamp() - timer)*1e-3;
			//cout << "Obs time: " << timer << endl;
			
			MatrixXd stateest(3, 1);
			stateest << est(1,0), est(2,0), est(3,0);
			dest = est(4, 0);
			
			// cout << "Stateest: " << endl << stateest << endl;
			
			// Compute new command and change global variables to pass the command to the valve thread
			timer = getTimeStamp();
				u = cont.step(stateest, HR, dest);
				piLock(0);
					valve_command_wished = u;
					hmforvalve = hm;
				piUnlock(0);
			timer = (getTimeStamp() - timer)*1e-3;
			//cout << "Controller time: " << timer << endl;
			
			// Log data for processing later (e.g. matlab)
			log << (currentT - startT)*1e-6 << ", " << hm << ", " << est(1, 0) << ", " << est(2, 0) << ", " << est(3, 0) << ", " << flow << ", " << dest << ", " << valve_command_actual << ", " << valve_command_wished << endl;
		}
	}
	
	log.close();
}

void initialize()
{
    wiringPiSetup();
		
	// Initialize valves
	pinMode (VALVE_IN, OUTPUT);
	pinMode (VALVE_OUT, OUTPUT);
	
	digitalWrite(VALVE_IN, 0);
	digitalWrite(VALVE_OUT, 0);
}



void loadConfigData(string fileName, Observer& observer, Controller& controller)
{
	ifstream file;
	file.open(fileName.c_str());
	if(!file.is_open())
		cout << "Error while opening  " << file;
		
	string line;
	vector<string> data;
		
	getline(file, line); getline(file, line); // skip the first two lines
	
	// Read K
	getline(file, line);
	split(data, line, ',' );
	
	Eigen::RowVector3d K;
	K << atof(data[1].c_str()), atof(data[2].c_str()), atof(data[3].c_str());
	controller.setK(K);
	
	cout << "K = " << endl << K << endl;
	
	// Read k
	getline(file, line);
	split(data, line, ',' );
	
	double k;
	k = atof(data[1].c_str());
	controller.setk(k);
	
	cout << "k = " << endl << k << endl;
	
	// Read kd
	getline(file, line);
	split(data, line, ',' );
	
	double kd;
	kd = atof(data[1].c_str());
	controller.setkd(kd);
	
	cout << "kd = " << endl << kd << endl;
	
	getline(file, line); // skip blank line
	
	// Read A
	getline(file, line);
	split(data, line, ',' );
	
	Eigen::Matrix4d A;
	A << atof(data[1].c_str()), atof(data[2].c_str()), atof(data[3].c_str()), atof(data[4].c_str()),
	     atof(data[5].c_str()), atof(data[6].c_str()), atof(data[7].c_str()), atof(data[8].c_str()),
	     atof(data[9].c_str()), atof(data[10].c_str()), atof(data[11].c_str()), atof(data[12].c_str()),
	     atof(data[13].c_str()), atof(data[14].c_str()), atof(data[15].c_str()), atof(data[16].c_str());
	observer.setA(A);
	
	cout << "A = " << endl << A << endl;
	
	// Read B
	getline(file, line);
	split(data, line, ',' );
	
	Eigen::Vector4d B;
	B << atof(data[1].c_str()), atof(data[2].c_str()), atof(data[3].c_str()), atof(data[4].c_str());
	observer.setB(B);
	
	cout << "B = " << endl << B << endl;
	
	// Read C
	getline(file, line);
	split(data, line, ',' );
	
	Eigen::RowVector4d C;
	C << atof(data[1].c_str()), atof(data[2].c_str()), atof(data[3].c_str()), atof(data[4].c_str());
	observer.setC(C);
	
	cout << "C = " << endl << C << endl;
	
	// Read L
	getline(file, line);
	split(data, line, ',' );
	
	Eigen::Vector4d L;
	L << atof(data[1].c_str()), atof(data[2].c_str()), atof(data[3].c_str()), atof(data[4].c_str());
	observer.setL(L);
	
	cout << "L = " << endl << L << endl;


	file.close();
}

