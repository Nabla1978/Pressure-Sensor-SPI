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
/* Interface to Intersema Barometric pressure sensor. * For the moment, only the MS5534C is supported.
* Others might be added in the future, especially the one with I2C.
* Created 14 Jan 2010 * By Richard Ulrich * Inspired by the work of Hari Nair
*/

#include “IntersemaBaro.h”
//#include “cppfix.h”

//using namespace Intersema;
/** @brief Constructur initializes the pins, reads the coefficients and reads a first altutude measurement. */
BaroPressure_MS5534C::BaroPressure_MS5534C(uint8_t pinMCLK, uint8_t pinSCLK, uint8_t pinDIN, uint8_t pinDOUT) : pinMCLK_(pinMCLK), pinSCLK_(pinSCLK), pinDIN_(pinDIN), pinDOUT_(pinDOUT)
{}

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
	//and then use wa and wb for creating coefficients 0, 4 and 5

	wa = ReadCoefficient(0×19);
	wb = ReadCoefficient(0×1A);
	//and then use wa and wb for creating coefficients 1, 2 and 3
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
	//gets in input the reading and make compensation. Returns only the pressure!
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
