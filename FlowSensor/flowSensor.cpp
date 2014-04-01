#include "flowSensor.hpp"

using namespace std;

FlowSensor::FlowSensor()
{
	cout << "Creating flow sensor object." << endl;
	
	fd = wiringPiI2CSetup(0x49);
	if(fd < 0)
		cout << "Error while stting up device: check address" << endl;
	cout << "Filehandle: " << fd << endl;
			cout << "Writing: " << wiringPiI2CWrite(fd, 0x49) << endl;

}

FlowSensor::~FlowSensor()
{

}

double FlowSensor::flow()
{
		double measures = 0;
	double measure;
	int nb = 4;
	
	for(int i = 0; i < nb; i++)
	{
		short measure = wiringPiI2CReadReg16(fd, 0);
		// cout << "Measure before reverse: " << measure << endl;
		measure = reverseByteOrder(measure);
		
		// cout << "Measure: " << measure << endl;
		
		measures += measure;
		delay(2);
	}
	
	measures = measures/nb; // Mean over nb measures
	
	measure = 200*((measures/16384) - 0.1)/0.8; // 200 = full scale flow (cf. datasheet)
	
	return measure;
}
