#include "pressureSensor.hpp"

using namespace std;

PressureSensor::PressureSensor()
{
	cout << "Creating pressure sensor object." << endl;
	
	wiringPiSetup ();
	pinMode (SCLK, OUTPUT);
	pinMode (DOUT, INPUT);
	pinMode (DIN, OUTPUT);
	pinMode (CSB, OUTPUT);
	
	reset();
	
	// Read constants
	for(int i = 0; i < 8; i++)
	{
		writeWord(0xA0 + 2*i);
		mConstants[i] = readWord(16);
		cout << "Mot " << i << " : " << mConstants[i] << endl;
	}
	
	if( int(checkSum(mConstants)) != mConstants[7] )
		cout << "Warning: check sum different from C7, check data/communication!";
		
	// Save reference pressure (h = 0)
	mP0 = this->pressure();

}

PressureSensor::~PressureSensor()
{
}

void PressureSensor::reset()
{
	wakeUp();
	writeWord(0x1e);
	delayMicroseconds(70*SCLK_PERIODE); // Wait for reloading
}

double PressureSensor::pressure()
{
	uint32_t d1, d2;
	
	wakeUp();
	
	writeWord(0x44);
	delayMicroseconds(2500);
	wakeUp();
	readWord(8); 
	d1 = readWord(24);
	 
	writeWord(0x54);
	delayMicroseconds(2500);
	wakeUp();
	readWord(8);
	d2 = readWord(24);
	
	return computeTP(d1, d2, mConstants);
}

double PressureSensor::depth()
{
	return 10*(this->pressure() - mP0); // 1bar = 10m, pressure is in 0.1bar
}

double PressureSensor::computeTP(uint32_t d1, uint32_t d2, uint32_t *constants)
{
	//int32_t dt = d2 - (constants[5] << 8);
	//int32_t temp = 2000 + (dt * constants[6]) >> 23;
	//int64_t off = (constants[2] << 16) + (constants[4] * dt) >> 7;
	//int64_t sens = (constants[1] << 15) + (constants[3] * dt) >> 8;
	//int32_t p = (((d1 * sens) >> 21) - off) >> 15;
	
	int32_t dt = (int32_t)(d2 - constants[5] * pow(2, 8));
	int32_t temp = (int32_t)(2000 + double(dt  * constants[6]) / pow(2, 23));
	int64_t off = int64_t((constants[2] * pow(2, 16)) + (constants[4] * dt) / pow(2, 7));
	int64_t sens = int64_t(constants[1] * pow(2, 15) + (constants[3] * dt) / pow(2, 8));
	int32_t p = int32_t(((d1 * sens) / pow(2, 21) - off) / pow(2, 15));
	
	/*cout << "dt = " << dt << endl;
	cout << "C6 = " << dt * constants[6] << endl;
	cout << "off = " << off << endl;
	cout << "sens = " << sens << endl;*/
	
	// cout << "Pressure : " << p << " & temperature : " << temp << endl;
	
	return p*0.1*0.001; // p = xxxx.x mbar -> Bar
}

void PressureSensor::writeWord(char pattern)
{
	wakeUp();
	
    char c;
    pattern = reverseBitOrder(pattern);

    setSCLK(0);
 
    for(int i=0; i < 8; i++)
    {
        c = (char) (pattern & 1);
        
        if (c == 1)
            setDIN(1);
        else
			setDIN(0);
			
        waitOnePulse();
        setSCLK(1);
        waitOnePulse();
        setSCLK(0);
        
        pattern = (char) (pattern >> 1);
    }
    
    setDIN(0);
    setSCLK(0);
}

uint32_t PressureSensor::readWord(int size)
{
    uint32_t v = 0;

    setSCLK(0);
    waitOnePulse();

    for (int i = 0; i < size; i++)
    {
        setSCLK(1);
        delayMicroseconds(0.2*SCLK_PERIODE);
        v = v << 1;
        
        if (getDOUT())
        {
            v = v | 1;
		}
		else
		{
		}
		
        delayMicroseconds(0.8*SCLK_PERIODE);
        
        setSCLK(0);
        waitOnePulse();
    }
    
    setSCLK(0);

    return(v);
}

char PressureSensor::reverseBitOrder(char data)
{
	data = (data & 0xF0) >> 4 | (data & 0x0F) << 4;
	data = (data & 0xCC) >> 2 | (data & 0x33) << 2;
	data = (data & 0xAA) >> 1 | (data & 0x55) << 1;
	return data;
}

void PressureSensor::wakeUp()
{
	digitalWrite(CSB, 1);
	delayMicroseconds(SCLK_PERIODE);
	digitalWrite(CSB, 0);
}

void PressureSensor::setDIN(bool state)
{
	digitalWrite(DIN, state);
}

char PressureSensor::getDOUT()
{
   char dig = digitalRead(DOUT);
   return dig;
}

void PressureSensor::setSCLK(bool state)
{
	digitalWrite(SCLK, state);
}

void PressureSensor::waitOnePulse()
{
    delayMicroseconds(SCLK_PERIODE);
}

unsigned char PressureSensor::checkSum(uint32_t sensorCoefficients[])
{

    int cnt;
    unsigned int n_rem;
    unsigned int crc_read;
    unsigned char  n_bit;
    
    n_rem = 0x00;
    crc_read = sensorCoefficients[7];
    sensorCoefficients[7] = ( 0xFF00 & ( sensorCoefficients[7] ) );
    
    for (cnt = 0; cnt < 16; cnt++)
    { // choose LSB or MSB
        if ( cnt%2 == 1 ) n_rem ^= (unsigned short) ( ( sensorCoefficients[cnt>>1] ) & 0x00FF );
        else n_rem ^= (unsigned short) ( sensorCoefficients[cnt>>1] >> 8 );
        for ( n_bit = 8; n_bit > 0; n_bit-- )
        {
            if ( n_rem & ( 0x8000 ) )
            {
                n_rem = ( n_rem << 1 ) ^ 0x3000;
            }
            else {
                n_rem = ( n_rem << 1 );
            }
        }
    }
    
    n_rem = ( 0x000F & ( n_rem >> 12 ) );// // final 4-bit reminder is CRC code
    sensorCoefficients[7] = crc_read; // restore the crc_read to its original place
    
    return ( n_rem ^ 0x00 ); // The calculated CRC should match what the device initally returned.
}
