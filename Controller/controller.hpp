#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <iostream>
#include "../utils.hpp"

class Controller
{
public:
	Controller();
	~Controller();
	
	double step(Eigen::Vector3d stateest, double hr, double d1est);
	double getU();
	
	void setK(Eigen::RowVector3d value) { K = value; }
	void setk(double value) { k = value; }
	void setkd(double value) { kd = value; }

private:
	Eigen::RowVector3d K;
	double k;
	double kd;
	
	double u;
};

#endif
