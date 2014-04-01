#ifndef OBSCONLOOP_H
#define OBSCONLOOP_H

#include <Eigen/Dense>
#include <iostream>
#include "../utils.hpp"
#include "../Controller/controller.hpp"
#include "../Observer/observer.hpp"

class ObsConLoop
{
public:
	ObsConLoop(Controller& controller, Observer& observer);
	~ObsConLoop();
	
	void step();

private:
	Controller mController;
	Observer mObserver;

};

#endif

