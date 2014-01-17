#include <iostream>
#include <wiringPi.h>

// g++ -o controller main.cpp PressureSensor/pressureSensor.cpp utils.cpp PressureSensor/pressureSensor.hpp Observer/observer.hpp Observer/observer.cpp Controller/controller.cpp -lwiringPi -I /home/pi/Desktop/eigen
// g++ -o controller main.cpp Observer/observer.hpp Observer/observer.cpp Controller/controller.cpp utils.cpp -I "E:\Dropbox\Projet Inno veste plongeur\Code\Eigen"



#include "PressureSensor/pressureSensor.hpp"
#include "Observer/observer.hpp"
#include "Controller/controller.hpp"
#include "utils.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[])
{
	PressureSensor sensor;
    
    Observer obs;
	obs.start();
	
	Controller cont;
	
	uint64_t previousT = getTimeStamp();
	double hm;
	while(true) 
	{
		if((getTimeStamp() - previousT) > 100000)
		{
			previousT = getTimeStamp();
			hm = sensor.depth();
			//hm = 0.5;
			
			MatrixXd est = obs.step(hm, cont.getU());
			
			MatrixXd stateest(3, 1);
			stateest << est(1,0), est(2,0), est(3,0);
			double dest = est(4,0);
			
			cout << "Stateest: " << endl << stateest << endl;
			
			cont.step(stateest, 0.15, dest);
		}
	}
}
