#include "observer.hpp"

using namespace std;
using namespace Eigen;

Observer::Observer(){
	//A << 0, 1, 0, 0,
		 //0.2058, 0, -100.4812,0,
		 //0, 0, 0, 1,
		 //0, 0, 0, 0;
	
	//B << 0, 0, 1, 0;
	
	//C << 1, 0, 0, 0;
	
	//L << 18.0517, 34.9123, -0.2902, -0.1013;
	
	A << 0, 1, 0, 0,
		 0.2067, 0, -33.6491, 0,
		 0, 0, 0, 1,
		 0, 0, 0, 0;
	
	B << 0, 0, 1, 0;
	
	C << 1, 0, 0, 0;
	
	L << 18.85, 127.6622, -13.25, -10;
	
	
	
	cout << "A = " << endl << A << endl;
	cout << "B = " << endl << B << endl;
	cout << "C = " << endl << C << endl;
	cout << "L = " << endl << L << endl;
	
	Aobs = A - L*C;
	Bobs.resize(4,2);
	Bobs << B, L;
	Cobs.resize(5,4);
	Cobs << C, Matrix4d::Identity();
	
	cout << "Aobs = " << endl << Aobs << endl;
	cout << "Bobs = " << endl << Bobs << endl;
	cout << "Cobs = " << endl << Cobs << endl;
	
	state << 0, 0, 0, 0;
	
	cout << "StateInit = " << endl << state << endl;
}

Observer::~Observer()
{
}

void Observer::start()
{
	// Reset observer internal variables
	previousTime = getTimeStamp();
	state << 0, 0, 0, 0;
}

MatrixXd Observer::step(double hm, double u)
{
	Vector2d U; 
	hm -= HBAR; // -hbar / ubar = 0 
	U << u, hm;
	
	Vector4d statedot = Aobs*state + Bobs*U;
	MatrixXd y(5, 1);
	y = Cobs*state;
	
	MatrixXd offset(5, 1);
	offset << HBAR, HBAR, 0, NBAR, 0;
	y += offset;
	
	uint64_t currentTime = getTimeStamp();
	double deltaT = (currentTime - previousTime)*1e-6;
	state += statedot*deltaT;
	
	previousTime = currentTime;
	
	//cout << "Observer step: deltaT = " << deltaT << endl;
	//cout << "statedot = " << endl << statedot << endl;
	//cout << "state = " << endl << state << endl;
	//cout << "y = " << endl << y << endl << endl;
	
	return y;
}
