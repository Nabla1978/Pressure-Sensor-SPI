*******************   IntersemaBaro.h :  **********************

/*
* Interface to Intersema Barometric pressure sensor.
* For the moment, only the MS5534C is supported.
* Others might be added in the future, especially the one with I2C.
*
* Created 14 Jan 2010
* By Richard Ulrich
* Inspired by the work of Hari Nair
*/
#ifndef INTERSEMA_BARO_H
#define INTERSEMA_BARO_H

#include <WProgram.h>

class  BaroPressure_MS5534C
{
	public:
	BaroPressure_MS5534C(uint8_t pinMCLK, uint8_t pinSCLK, uint8_t pinDIN, uint8_t pinDOUT);

	void begin();

	long getHeightCentiMeters(void)
	{
		return AcquireAveragedSampleCm(NUM_SAMP_FOR_AVG);
	}

	long getHeightMeters(void)
	{
		long AltCm = getHeightCentiMeters();
		long AltAvgM  = (AltCm >= 0 ? (AltCm + 50L) / 100L : (AltCm - 50L) / 100L);
		return AltAvgM;
	}

	private:
	void ResetSensor();
	void ReadCoefficients(void);
	size_t ReadCoefficient(unsigned char addr);
	void SendCommand(unsigned long cmd, size_t nbits);
	unsigned int ReadWord(void);
	long AcquireAveragedSampleCm(const size_t nSamples);
	long ConvertPressureTemperature(unsigned int pressure, unsigned int temperature);
	void TriggerTemperatureSample(void);
	void TriggerPressureSample(void);
	long PascalToCentimeter(long pressurePa);
	
	enum SensorStates
	{
		SENSOR_READ_TEMPERATURE
	};

	static const size_t NUM_SAMP_FOR_AVG = 4;

	const uint8_t pinMCLK_; // master clock 32.5 kHz
	const uint8_t pinSCLK_; // serial clock
	const uint8_t pinDIN_;  // data in  (out on the arduino)
	const uint8_t pinDOUT_; // data out (in  on the arduino)
	
	unsigned int coefficients_[6];
	SensorStates SensorState_;
	size_t       SmpCnt_;
};
	
#endif
	
*******************   IntersemaBaro.cpp :  **********************
	
/*
* Interface to Intersema Barometric pressure sensor.
* For the moment, only the MS5534C is supported.
* Others might be added in the future, especially the one with I2C.
*
* Created 14 Jan 2010
* By Richard Ulrich
* Inspired by the work of Hari Nair
*/

#include “IntersemaBaro.h”
//#include “cppfix.h”

//using namespace Intersema;

/** @brief Constructur initializes the pins, reads the coefficients and reads a first altutude measurement. */
BaroPressure_MS5534C::BaroPressure_MS5534C(uint8_t pinMCLK, uint8_t pinSCLK, uint8_t pinDIN, uint8_t pinDOUT) : pinMCLK_(pinMCLK), pinSCLK_(pinSCLK), pinDIN_(pinDIN), pinDOUT_(pinDOUT)
{

}

/** @brief initialization function that has to be called in the setup() function. */
void BaroPressure_MS5534C::begin()
{
	// set the pin directions
	pinMode(pinMCLK_, OUTPUT);
	pinMode(pinSCLK_, OUTPUT);
	pinMode(pinDIN_,  OUTPUT);
	pinMode(pinDOUT_, INPUT);

	// generate approx 34kHz square for pinMCLK_
	tone(pinMCLK_, 31250);

	// configure the sensor
	ResetSensor();
	ReadCoefficients();

	// initialize
	long alt = AcquireAveragedSampleCm(NUM_SAMP_FOR_AVG);
	TriggerTemperatureSample();
	SensorState_ = SENSOR_READ_TEMPERATURE;
}

void BaroPressure_MS5534C::ResetSensor()
{
	SendCommand(0×155540, 21); // 1010101010101010 + 00000
}

void BaroPressure_MS5534C::ReadCoefficients(void)
{
	unsigned int wa = ReadCoefficient(0×15);
	unsigned int wb = ReadCoefficient(0×16);
	coefficients_[0] = (unsigned int)((wa >> 1) & (unsigned int)0×7FFF);
	coefficients_[4] = (unsigned int)(((wa & 0×1) << 10) | ((wb >> 6) & (unsigned int)0×3FF));
	coefficients_[5] = (unsigned int)(wb & 0×3F);

	wa = ReadCoefficient(0×19);
	wb = ReadCoefficient(0×1A);
	coefficients_[3] = (unsigned int)((wa >> 6) & 0×3FF);
	coefficients_[1] = (unsigned int)(((wa & 0×3F) << 6) | (wb & 0×3F));
	coefficients_[2] = (unsigned int)((wb >> 6) & 0×3FF);

	#ifdef DEBUG
	//    for(size_t i=0; i<6; ++i)
	//        {
	//            Serial.print(”Coefficient “);
	//            Serial.print(i + 1, DEC);
	//            Serial.print(” : “);
	//            Serial.println(coefficients_[i], DEC);
	//    }
	#endif
}

size_t BaroPressure_MS5534C::ReadCoefficient(unsigned char addr)
{
	// 111 + 6bit coeff addr + 000 + 1clk(send0)
	unsigned long cmd = (unsigned long)0×1C00 | (((unsigned long)addr) << 4);
	SendCommand(cmd,13);
	return ReadWord();
}

// send command MS bit first
void BaroPressure_MS5534C::SendCommand(unsigned long cmd, size_t nbits)
{
	while(nbits–)
	{
		if(cmd & (unsigned long)(1 << nbits))
			digitalWrite(pinDIN_, HIGH);
		else
			digitalWrite(pinDIN_, LOW);
	
		digitalWrite(pinSCLK_, HIGH);
		digitalWrite(pinSCLK_, LOW);
	}
}

unsigned int BaroPressure_MS5534C::ReadWord(void)
{
	unsigned int w;
	unsigned int clk = 16;
	w = 0;
	while(clk–)
	{
		digitalWrite(pinSCLK_, HIGH);
		digitalWrite(pinSCLK_, LOW);
		w |=  (digitalRead(pinDOUT_) << clk);
	}
	digitalWrite(pinSCLK_, HIGH);
	digitalWrite(pinSCLK_, LOW);

	return w;
}

long BaroPressure_MS5534C::AcquireAveragedSampleCm(const size_t nSamples)
{
	long pressAccum = 0;
	
	for(size_t n = nSamples; n; n–)
	{
		TriggerTemperatureSample();
		while(digitalRead(pinDOUT_))
		;
		const unsigned int temperature = ReadWord();
		TriggerPressureSample();
		while(digitalRead(pinDOUT_))
		;
		const unsigned int pressure = ReadWord(); // read pressure
		pressAccum += ConvertPressureTemperature(pressure, temperature);
	}
	long pressAvg = pressAccum / nSamples;
	long AltCm    = PascalToCentimeter(pressAvg * 10);

	return AltCm;
}

long BaroPressure_MS5534C::ConvertPressureTemperature(unsigned int pressure, unsigned int temperature)
{
	const long UT1  = (coefficients_[4] << 3) + 20224;
	const long dT   = (long)temperature  - UT1;
	const long TEMP = 200 + ((dT * (coefficients_[5] + 50)) >> 10);
	const long OFF  = (coefficients_[1] <<2) + (((coefficients_[3] -512) * dT) >> 12);
	const long SENS = coefficients_[0]  + ((coefficients_[2] * dT) >> 10)  + 24576;
	const long X    = ((SENS* ((long)pressure  - 7168)) >> 14) - OFF;
	pressure    = ((X * 10) >> 5) + 2500;
	temperature = TEMP;

	long T2 = 0, P2 = 0;
	if(TEMP < 200)
	{
		T2 = (11 * (coefficients_[5] + 24) * (200 - TEMP) * (200 - TEMP)) >> 20;
		P2 = (3 * T2 * (pressure - 3500)) >> 14;
		pressure    = pressure - P2;
		temperature = temperature - T2;
	}

	return pressure;
}

void BaroPressure_MS5534C::TriggerTemperatureSample(void)
{
	// 111 + 1001 + 000 + 2clks(send 0)
	ResetSensor();
	SendCommand(0xF20, 12);
}

void BaroPressure_MS5534C::TriggerPressureSample(void)
{
	// 111 + 1010 + 000 + 2clks(send 0)
	ResetSensor();
	SendCommand(0xF40, 12);
}

long BaroPressure_MS5534C::PascalToCentimeter(long pressurePa)
{
	// Lookup table converting pressure in Pa to altitude in cm.
	// Each LUT entry is the altitude in cm corresponding to an implicit
	// pressure value, calculated as [PA_INIT - 1024*index] in Pa.
	// The table is calculated for a nominal sea-level pressure  = 101325 Pa.
	static const size_t PZLUT_ENTRIES = 77;
	static const size_t PA_INIT       = 104908;
	static const size_t PA_DELTA      = 1024;

	static const long lookupTable[PZLUT_ENTRIES] = {
	-29408, -21087, -12700,  -4244,   4279,
	12874,  21541,  30281,  39095,  47986,
	56953,  66000,  75126,  84335,  93628,
	103006, 112472, 122026, 131672, 141410,
	151244, 161174, 171204, 181335, 191570,
	201911, 212361, 222922, 233597, 244388,
	255300, 266334, 277494, 288782, 300204,
	311761, 323457, 335297, 347285, 359424,
	371719, 384174, 396795, 409586, 422552,
	435700, 449033, 462560, 476285, 490216,
	504360, 518724, 533316, 548144, 563216,
	578543, 594134, 609999, 626149, 642595,
	659352, 676431, 693847, 711615, 729752,
	748275, 767202, 786555, 806356, 826627,
	847395, 868688, 890537, 912974, 936037,
	959766, 984206};

	if(pressurePa > PA_INIT)
		return lookupTable[0];
	else
	{
		const long inx = (PA_INIT - pressurePa) >> 10;
		if(inx >= PZLUT_ENTRIES - 1)
			return lookupTable[PZLUT_ENTRIES - 1];
		else
		{
			const long pa1 = PA_INIT - (inx << 10);
			const long z1 = lookupTable[inx];
			const long z2 = lookupTable[inx+1];
			return (z1 + (((pa1 - pressurePa) * (z2 - z1)) >> 10));
		}
	}
}