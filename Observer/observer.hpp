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
	
	void setA(Eigen::Matrix4d value) { A = value; }
	void setB(Eigen::Vector4d value) { B = value; }
	void setC(Eigen::RowVector4d value) { C = value; }
	void setL(Eigen::Vector4d value) { L = value; }
	
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

