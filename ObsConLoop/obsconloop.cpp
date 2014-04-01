#include "controller.hpp"

using namespace std;
using namespace Eigen;

ObsConLoop::ObsConLoop(Controller& controller, Observer& observer)
{
	mController = controller;
	mObserver = observer;
}

void ObsConLoop::step()
{
	
	
	
}

ObsConLoop::~ObsConLoop()
{
}
