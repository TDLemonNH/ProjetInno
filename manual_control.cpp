#include <iostream>
#include <wiringPi.h>

#define VALVE_IN 2
#define VALVE_OUT 3

using namespace std;

int main()
{
	wiringPiSetup ();
	pinMode (VALVE_IN, OUTPUT);
	pinMode (VALVE_OUT, OUTPUT);
	
	digitalWrite(VALVE_IN, 0);
	digitalWrite(VALVE_OUT, 0);
	
	char ch;
	while(true)
	{
		cin.get(ch);
		if(ch == '+')
		{
			cout << "add air" << endl;
			digitalWrite(VALVE_IN, 1);
			delay(10);
			digitalWrite(VALVE_IN, 0);
		}
		else if(ch == '-')
		{
			cout << "remove air" << endl;
			digitalWrite(VALVE_OUT, 1);
			delay(200);
			digitalWrite(VALVE_OUT, 0);
			cout << "remove air end" << endl;
		}
		//else
		//{
			//digitalWrite(VALVE_OUT, 0);
			//digitalWrite(VALVE_IN, 0);
		//}
	}
}
