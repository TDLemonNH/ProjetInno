#ifndef VALVECYCLE_H
#define VALVECYCLE_H

double computeDmaxout(double hm)
{
	double C = 9.2;
	double b = 0.35;
	//double p = (101325 + hm*1e4)*1e-6; // p in MPa
	//double p0 = 101325*1e-6;
	//double qmaxout = 226.3*1.2*sqrt(abs(p - p0)*(p0 + p0)); //en l/min
	//return qmaxout*p0*1e6/8.31/293/60000; //conversion en mol/s
	
	double p = (103900 + hm*1e4)*1e-6; // p in MPa
	double p0 = 103900*1e-6;
	//double qmaxout = 240*47*sqrt(abs(p - p0)*(p0 + p0)) * 0.2; //en l/min
	//double qmaxout = 600*C*(p+0.1)*sqrt(1 - (((p0+0.1)/(p+0.1)-b)/(1-b))*(((p0+0.1)/(p+0.1)-b)/(1-b))); //en l/min
	double qmaxout = 12800 - 239000*p + 1110000*p*p + 67;
	return qmaxout*p0*1e6/8.31/293/60/1000; //conversion en mol/s
}	

PI_THREAD (valve_cycle) {
	uint64_t current = getTimeStamp();
	uint64_t start = getTimeStamp();
	int open_time;
	int cycle_time = 100000;
	double u = 0;
	double hm;
	
	while(true)
	{
		current = getTimeStamp();
		if (((current - start) > open_time) && (digitalRead(VALVE_OUT) || digitalRead(VALVE_IN)))
		{
			digitalWrite(VALVE_OUT, 0);
			digitalWrite(VALVE_IN, 0);
			delay(15);
		}
		if ((current - start) > cycle_time)
		{
			piLock(0);
				u = valve_command_wished;
				hm = hmforvalve;
			piUnlock(0);
			
			dmaxout = computeDmaxout(hm);
			
			cout << "Debit max out: " << dmaxout << endl;
			
			
			if(u > dmaxin)
				u = dmaxin;
			if(u < -dmaxout)
				u = -dmaxout;
				
			piLock(0);
				valve_command_actual = u;
			piUnlock(0);
			
			if (u > 0)
			{
				digitalWrite(VALVE_IN, 1);
				open_time = u/dmaxin*0.85*cycle_time;
			}
			else if (u < 0)
			{
				digitalWrite(VALVE_OUT, 1);
				open_time = abs(u/dmaxout*0.85*cycle_time);
			}
			start = current;
		}
	}
}

#endif
