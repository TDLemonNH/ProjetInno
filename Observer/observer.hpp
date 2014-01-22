#ifndef OBSERVER_H
#define OBSERVER_H

#include <Eigen/Dense>
#include <iostream>
#include "../utils.hpp"

class Observer
{
public:
	Observer();
	~Observer();
	
	Eigen::MatrixXd step(double hm, double u);
	void start();
	
private:
	Eigen::Matrix4d A;
	Eigen::Vector4d B;
	Eigen::RowVector4d C;
	Eigen::Vector4d L;
	
	Eigen::Matrix4d Aobs;
	Eigen::MatrixXd Bobs;
	Eigen::MatrixXd Cobs;
	
	Eigen::Vector4d state;
	
	uint64_t previousTime;
	
};

#endif

