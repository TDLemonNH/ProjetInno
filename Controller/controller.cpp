#include "controller.hpp"

using namespace std;
using namespace Eigen;

Controller::Controller()
{
	K << -0.301877934929443, -0.271162633895695, 6;
	k = 0.265019573688946;
	kd = 1;
	
	u = 0;
}

Controller::~Controller()
{
}

double Controller::step(Eigen::Vector3d stateest, double hr, double d1est)
{
	MatrixXd in(5, 1);
	in << stateest, hr, d1est;
	
	MatrixXd bias(5, 1);
	bias << HBAR, 0, NBAR, HBAR, 0; // hbar, vbar, Nbar, hbar, dbar
	in = in - bias;
	
	MatrixXd gain(1, 5);
	gain << -K, k, -kd;
	
	MatrixXd out = gain*in;
	
	//cout << "Controller step: " << endl;
	//cout << "In: " << endl << in << endl;
	//cout << "Bias: "<< endl << bias << endl;
	//cout << "Gain: " << endl << gain << endl;
	//cout << "U: " << endl << out << endl << endl;
	
	u = out(0, 0);
	return u;
}

double Controller::getU()
{
	return u;
}
