#pragma once

// AS3935 Lightning Sensor with I2C communications
// Copyright (C) muman.ch, 2025.09.14
// info@muman.ch
// See github for details:
// https://github.com/mumanchu/AS3935LightningSensor

#include <Wire.h>

// Define this to enable debug output to Serial
// TODO see MumanchuDebug.h
#define DEBUG

#ifdef DEBUG
#define AS3935_LOGERROR(s) Serial.print("AS3935 ERROR: "); Serial.println(s); Serial.flush()
#define AS3935_ASSERT(b) if (!(b)) { Serial.println("AS3935 ASSERT"); Serial.flush(); return false; }
#else
#define AS3935_LOGERROR(s)
#define AS3935_ASSERT(b)
#endif

// These may not be defined
//typedef unsigned int uint;
//typedef unsigned long ulong;


class AS3935LightningSensor
{
protected:
	TwoWire* wire;
	int i2cAdds;

public:
	// Register bit masks taken from data sheet p19
	// the registers themselves do not have names, so we use the hex register numbers
	enum MASK : uint
	{
		// 0x00 R/W
		AFE_GB =            0b00111110,		// AFE gain boost
		PWD =               0b00000001,		// 1=power down, 0=active
		// 0x01 R/W
		NF_LEV =            0b01110000,		// Noise floor level
		WDTH =              0b00001111,		// Watchdog threshold
		// 0x02 R/W
		CL_STAT =           0b01000000,		// Clear statistics, toggle high-low-high
		MIN_NUM_LIGH =      0b00110000,		// Minimum strikes for INT
		SREJ =              0b00001111,		// Spike rejection
		//0x03 R/W
		LCO_FDIV =          0b11000000,		// Frequency divider for antenna tuning
		MASK_DIST =         0b00100000,		// Mask disturber INT
		INT =               0b00001111,		// Interrupt source
		// 0x04 R
		S_LIG_L =           0b11111111,		// Strike energy LS byte (21-bit value)
		// 0x05 R
		S_LIG_M =           0b11111111,		// Strike energy MS byte
		// 0x06 R
		S_LIG_MM =          0b00011111,		// Strike energy MMS byte
		// 0x07 R
		DISTANCE =          0b00111111,		// Distance estimation in km, p33
		// 0x08 R/W
		DISP_LCO =          0b10000000,		// IRQ has LCO signal, 500kHz / LCO_FDIV
		DISP_SRCO =         0b01000000,		// IRQ has SRCO signal, 1.1MHz
		DISP_TRCO =         0b00100000,		// IRQ has TRCO signal, 32.768kHz
		TUN_CAP =           0b00001111,		// Capacitor LCO tuning, 0..120pF in 8pF steps
		// 0x3A R
		TRCO_CALIB_DONE =   0b10000000,		// TRCO calibration done successfully
		TRCO_CALIB_NOK =    0b01000000,		// TRCO calibration failed
		// 0x3B R
		SRCO_CALIB_DONE =   0b10000000,		// SRCO calibration done successfully
		SRCO_CALIB_NOK =    0b01000000,		// SRCO calibration failed

		// Send direct command, write 0x96 to issue the command
		PRESET_DEFAULT =    0x3C,			// Set all registers to default values
		CALIB_RCO =         0x3D			// Calibrate internal RC oscillators, 2ms
	};

	// IRQ source, what set IRQ active, register 0x03:INT
	enum IRQ_SOURCE : uint
	{
		INT_NH =    0b0001,		// Noise level too high
		INT_D =     0b0100,		// Disturber detected, disabled by MASK_DIST
		INT_L =     0b1000,		// Lightning detected, no. of events set by MIN_NUM_LIGH
		INT_PURGE = 0b0000		// Distance estimation changed due to event purge, p34
	};

	// Optimal AFE gains for setAFEGain()
	enum AFEGAIN : uint
	{
		INDOOR = 0x12,			// default
		OUTDOOR = 0x0e
	};
	

	// Control
	bool begin(TwoWire* twoWire, uint i2cAddress);
	bool reset();
	bool powerUp();
	bool powerDown();
	bool calibrateOscillators();
	bool clearStatistics();

	// Data Values
	bool getStormDistance(uint* km);
	bool getStrikeEnergy(ulong* strikeEnergy);
	bool getIRQSource(uint* irqSource);

	// Configuration
	bool setAFEGain(AFEGAIN afeGain);
	bool getAFEGain(AFEGAIN* afeGain);
	bool setMinimumStrikes(uint minStrikes);
	bool getMinimumStrikes(uint* minStrikes);
	bool enableDisturberInterrupt();
	bool disableDisturberInterrupt();
	bool setNoiseFloorLevel(uint noiseFloorLevel);
	bool getNoiseFloorLevel(uint* noiseFloorLevel);
	bool setWatchdogThreshold(uint wdogThreshold);
	bool getWatchdogThreshold(uint* wdogThreshold);
	bool setSpikeRejection(uint spikeRejection);
	bool getSpikeRejection(uint* spikeRejection);

	// Calibration
	bool selectIRQOutput(uint n);
	bool setLCOFrequencyDivider(uint fdiv);
	bool getLCOFrequencyDivider(uint* fdiv);
	bool setTuningCapacitor(uint tuneCap);
	bool getTuningCapacitor(uint* tuneCap);

#ifdef DEBUG
	bool dumpRegisters();
#endif

protected:
	bool readRegister(uint reg, byte* data);
	bool writeRegister(uint reg, byte data);
	bool readRegister(uint reg, uint mask, uint* data);
	bool writeRegister(uint reg, uint mask, uint data);
	void restartWire();
};


///////////////////////////////////////////////////////////////////////////////
// Control

// Power up, reset the chip, set all registers to default values
bool AS3935LightningSensor::begin(TwoWire* twoWire, uint i2cAddress)
{
	wire = twoWire;
	i2cAdds = i2cAddress;

	// if powered down, power it up
	byte data;
	if (!readRegister(0x00, &data))
		return false;
	if (data & PWD) {
		if (!powerUp())
			return false;
	}

	// set all registers to default values
	return reset();
}

// Set all registers to their default values
// always call calibrateOscillators() after reset()
// takes 2ms
bool AS3935LightningSensor::reset()
{
	if (!writeRegister(PRESET_DEFAULT, 0x96))
		return false;
	delay(2);
	return true;
}

// Power up 
// always call calibrateOscillators() after powerUp()
// takes 2ms
// p36 
bool AS3935LightningSensor::powerUp()
{
	if (!writeRegister(0x00, PWD, 0))
		return false;
	// LCO startup time after power up is 2ms
	delay(2);
	return true;
}

bool AS3935LightningSensor::powerDown()
{
	return writeRegister(0x00, PWD, 1);
}

// Calibrate the internal SRCO and TRCO oscillators
// this must be done after every powerUp() or reset()
// takes 2ms
// p36
bool AS3935LightningSensor::calibrateOscillators()
{
	// calibrate RCO command
	if (!writeRegister(CALIB_RCO, 0x96))
		return false;
	//delay(2);

	//TODO only if it is in power down mode?
	// toggle DISP_SRCO, as described in data sheet p36
	if (!writeRegister(0x08, DISP_SRCO, 1))
		return false;
	delay(2);
	if (!writeRegister(0x08, DISP_SRCO, 0))
		return false;

	// did it work? test the CALIB_DONE and CALIB_NOK flags
	byte trco, srco;
	if (!readRegister(0x3a, &trco) || !readRegister(0x3b, &srco))
		return false;

	// calibration should have been done
	if (!(trco & TRCO_CALIB_DONE) || !(srco & SRCO_CALIB_DONE)) {
		AS3935_LOGERROR("calib not done");
		return false;
	}

	// calibration errors?
	if ((trco & TRCO_CALIB_NOK) || (srco & SRCO_CALIB_NOK)) {
		AS3935_LOGERROR("calib failed");
		return false;
	}
	return true;
}

// Clear statistics for lightning distance estimation over the last 15 minutes 
// this sets the getLightingDistance() value to 0x3f (out of range)
// it does NOT affect the getStrikeEnergy() value
// p35
bool AS3935LightningSensor::clearStatistics()
{
	// toggle CL_STAT bit high-low-high
	// (this code reads register 0x02 just once instead of three times)
	byte b;
	if (!readRegister(0x02, &b))
		return false;
	if (!writeRegister(0x02, b | CL_STAT))
		return false;
	if (!writeRegister(0x02, b & ~CL_STAT))
		return false;
	return writeRegister(0x02, b | CL_STAT);
}


///////////////////////////////////////////////////////////////////////////////
// Data Values

// Returns the estimated distance (the radius) to the leading edge of
// the storm in kilometers : 1 = overhead .. 40 = 40km 
// 63 (0x3f) = out of range or no storm detected
// The value remains unchanged until a new estimate is ready, or until
// clearStatistics() is called.
// p33 figure. 43
bool AS3935LightningSensor::getStormDistance(uint* km)
{
	return readRegister(0x07, DISTANCE, km);
}

// Returns the estimated energy of the latest lightning strike as
// a 21-bit value, 0..0x1fffff (0..2097151)
// It's a unitless magnitude, useful to compare with previous values.
// The value remains unchanged until a new strike occurs or it's set 
// to 0 when a Disturber Event is detected. It is NOT cleared by 
// clearStatistics().
// p32
bool AS3935LightningSensor::getStrikeEnergy(ulong* strikeEnergy)
{
	*strikeEnergy = 0;
	byte lsByte, msByte, mmsByte;
	if (!readRegister(0x04, &lsByte))
		return false;
	if (!readRegister(0x05, &msByte))
		return false;
	if (!readRegister(0x06, &mmsByte))
		return false;
	*strikeEnergy = ((ulong)(mmsByte & S_LIG_MM) << 16) + (msByte << 8) + lsByte;
	return true;
}

// If the IRQ output is activated, wait 2ms, then use this to find the 
// source of the interrupt.
// Returns and IRQ_SOURCE value,
//  1 = INT_NH Noise level too high
//  4 = INT_D  Disturber detected, disabled by MASK_DIST
//  8 = INT_L  Lightning detected, no. of events set by MIN_NUM_LIGH
//  0 = INT_PURGE  Distance estimation changed due to clearStatistics() 
//      call, see p34. 0 is only valid if IRQ goes high.
// The INT value is set to zero when it is read.
// p34
bool AS3935LightningSensor::getIRQSource(uint* irqSource)
{
	return readRegister(0x03, INT, (uint*)irqSource);
}


///////////////////////////////////////////////////////////////////////////////
// Configuration

// Adjusts the AFE gain boost for indoor or outdoor use
// AFEGAIN::INDOOR (default) or AFEGAIN::OUTDOOR are the recommended 
// settings, but any value between 0x00 and 0x1F can be used
// p28
bool AS3935LightningSensor::setAFEGain(AFEGAIN afeGain)
{
	return writeRegister(0x00, AFE_GB, afeGain);
}

bool AS3935LightningSensor::getAFEGain(AFEGAIN* afeGain)
{
	return readRegister(0x00, AFE_GB, (uint*)afeGain);
}

// Set the minimum number of lightning events in a 15 minute window
// to set IRQ and generate the INT_L interrupt, 0..3=1|5|9|16
// minStrikes : 0 = 1 strike (default);  1 = 5 strikes;
//              2 = 9 strikes; 3 = 16 strikes
bool AS3935LightningSensor::setMinimumStrikes(uint minStrikes)
{
	return writeRegister(0x02, MIN_NUM_LIGH, minStrikes);
}

bool AS3935LightningSensor::getMinimumStrikes(uint* minStrikes)
{
	return readRegister(0x02, MIN_NUM_LIGH, minStrikes);
}

// Enable/disable the "disturber" interrupt INT_D with the
// MASK_DIST bit
// p34
bool AS3935LightningSensor::enableDisturberInterrupt()
{
	return writeRegister(0x03, MASK_DIST, 0);
}

bool AS3935LightningSensor::disableDisturberInterrupt()
{
	return writeRegister(0x03, MASK_DIST, 1);
}

// Selects the threshold for the Noise Floor Limit
// when the noise crosses this threshold it will issue the INT_NH 
// interrupt and activate IRQ. This indicates that measurements
// cannot be taken because of excessive background noise.
// noiseFloorLevel : 0..7, level in microvolts RMS (default = 2)
// p30 figure 41
bool AS3935LightningSensor::setNoiseFloorLevel(uint noiseFloorLevel)
{
	return writeRegister(0x01, NF_LEV, noiseFloorLevel);
}

bool AS3935LightningSensor::getNoiseFloorLevel(uint* noiseFloorLevel)
{
	return readRegister(0x01, NF_LEV, noiseFloorLevel);
}

// The watchdog threshold WDTH defines the signal level that causes 
// entry to "Signal Verification Mode" where it determines if it's 
// a strike or a disturber event, see p17. 
// Increasing the threshold makes it less sensitive to disturbers,
// but also makes it less sensitive to weak signals from distant 
// lightning sources.
// Sensitivity is also affected by SREJ, see setSpikeRejection().
// wdogThreshold : 0..15 (default = 2)
// p29 figure 40
bool AS3935LightningSensor::setWatchdogThreshold(uint wdogThreshold)
{
	return writeRegister(0x01, WDTH, wdogThreshold);
}

bool AS3935LightningSensor::getWatchdogThreshold(uint* wdogThreshold)
{
	return readRegister(0x01, WDTH, wdogThreshold);
}

// Spike rejection works together with the watchdog threshold to
// determine if the signal is a lightning strike or a man-made 
// disturber.
// Increasing the value improves disturber rejection, but also 
// makes it less sensitive to weak signals from distant lightning 
// sources.
// spikeRejection : 0..15 (default = 2)
// p31, p32 figure 42 
bool AS3935LightningSensor::setSpikeRejection(uint spikeRejection)
{
	return writeRegister(0x02, SREJ, spikeRejection);
}

bool AS3935LightningSensor::getSpikeRejection(uint* spikeRejection)
{
	return readRegister(0x02, SREJ, spikeRejection);
}


///////////////////////////////////////////////////////////////////////////////
// Calibration

// When calibrating the LCO antenna oscillator's resonant frequency, 
// the oscillator frequency must be output on the IRQ pin so it can 
// be accurately tuned to 500kHz +-3.5% with setTuningCapacitor().
// A frequency divider LCO_FDIV (16|32|64|128) for the LCO 500kHz is 
// set with setLCOFrequencyDivider().
// n : 0 = return to normal IRQ operation
//     1 = output the LCO antenna oscillator frequency, 500kHz / LCO_FDIV
//     2 = output the SRCO system RC oscillator, 1.1MHz
//     3 = output the TRCO timer RC oscillator, 32.768kHz
// >>> DO NOT FORGET to call selectIRQOutput(0) to set the IRQ pin 
//     back to normal operation <<<
// You may also want to check the system RC oscillator or the timer 
// RC oscillator frequencies too.
// p36
bool AS3935LightningSensor::selectIRQOutput(uint n)
{
	AS3935_ASSERT(n < 4);
	uint data = 0;
	switch (n) {
	case 1:
		data = DISP_LCO;
		break;
	case 2:
		data = DISP_SRCO;
		break;
	case 3:
		data = DISP_TRCO;
		break;
	}
	return writeRegister(0x08, DISP_LCO|DISP_SRCO|DISP_TRCO, data >> 5);
}

// When tuning the antenna and outputing the LCO resonant frequency
// on the IRQ pin, see selectIRQOutput(), the LCO frequency is 
// divided by 16|32|64|128 to give a lower frequency output.
// fdiv : 0 = divide by 16 (default); 1 = 32; 2 = 64; 3 = 128
// the LCO oscillator '500kHz / 16' should be 31.250kHz
// p35
bool AS3935LightningSensor::setLCOFrequencyDivider(uint fdiv)
{
	return writeRegister(0x03, LCO_FDIV, fdiv);
}

bool AS3935LightningSensor::getLCOFrequencyDivider(uint* fdiv)
{
	return readRegister(0x03, LCO_FDIV, fdiv);
}

// The LCO antenna oscillator resonant frequency must be 500kHz +-3.5%.
// This tunes the resonant frequency by selecting the internal capacitors
// in steps of 8pf from 0..120pf. This varies the frequency by up to 25kHz.
// The resonant frequency can be verified by outputing the signal on the 
// IRQ pin, see selectIRQOutput(). The frequency see on the pin is divided 
// by LCO_FDIV, see setLOCFreuqncyDivider().
// tuneCap : 0 = 0pF; 1 = 8pF; 2 = 16pF; ..; 15 = 120pF
// p35
bool AS3935LightningSensor::setTuningCapacitor(uint tuneCap)
{
	return writeRegister(0x08, TUN_CAP, tuneCap);
}

bool AS3935LightningSensor::getTuningCapacitor(uint* tuneCap)
{
	return readRegister(0x08, TUN_CAP, tuneCap);
}


///////////////////////////////////////////////////////////////////////////////
// Internal methods

// Reads the register bits defined by 'mask' and shifts them right 
// (normalises) so the LS bit of *data is the LS bit of the mask.
bool AS3935LightningSensor::readRegister(uint reg, uint mask, uint* data)
{
	*data = 0;
	AS3935_ASSERT(mask != 0 && mask <= 0xff);
	byte b;
	if (!readRegister(reg, &b))
		return false;
	if (mask == 0xff) {
		*data = b;
		return true;
	}
	// shift data according to the bit mask
	b &= mask;
	for (int i = mask; (i & 1) == 0; i >>= 1) {
		b >>= 1;
	}
	*data = b;
	return true;
}

// Writes register bits in 'mask' with 'newData' without modifying any other
// bits. 'newData' is shifted left to fit the mask. Returns false if the
// data will not fit the mask.
bool AS3935LightningSensor::writeRegister(uint reg, uint mask, uint newData)
{
	AS3935_ASSERT(mask != 0 && mask <= 0xff);
	byte data;
	if (mask == 0xff) {
		// use all the bits
		data = newData;
	}
	else {
		// read existing value
		if (!readRegister(reg, &data))
			return false;
		// shift new data according to the bit mask
		for (uint i = mask; (i & 1) == 0; i >>= 1) {
			newData <<= 1;
		}
		// make sure new data fits the bit mask
		if (newData & ~mask) {
			AS3935_LOGERROR("data+mask error");
			return false;
		}
		// mask out existing data and replace with new data
		data = (data & ~mask) | newData;
	}

	// write the new data
	return writeRegister(reg, data);
}

// The I2C bus may hang if there's too much interference, which
// happened a lot when testing using a Tesla coil spark generator
// this hack usually recovers so I2C communications can continue
void AS3935LightningSensor::restartWire()
{
	wire->end();
	delay(20);
	wire->begin();
	wire->setClock(400000);
}


#ifdef DEBUG
bool AS3935LightningSensor::dumpRegisters()
{
	const char fmt[] = "%02X  0x%02X\n\r";
	char buf[16];

	byte b;
	for (int i = 0; i <= 8; ++i) {
		if (!readRegister(i, &b))
			return false;
		sprintf(buf, fmt, i, b);
		Serial.print(buf);
	}
	for (int i = 0x3a; i <= 0x3b; ++i) {
		if (!readRegister(i, &b))
			return false;
		sprintf(buf, fmt, i, b);
		Serial.print(buf);
	}
	Serial.println("");
	Serial.flush();
	return true;
}
#endif

// Change these if you want to use SPI

bool AS3935LightningSensor::readRegister(uint reg, byte* data)
{
	*data = 0;
	wire->beginTransmission(i2cAdds);
	wire->write((byte)reg);
	if (wire->endTransmission(false) != 0) {	// 'false' so the bus is not released
		AS3935_LOGERROR("endtx failed");
		restartWire();
		return false;
	}
	if (wire->requestFrom(i2cAdds, 1) != 1) {
		AS3935_LOGERROR("requestFrom failed");
		restartWire();
		return false;
	}
	*data = (byte)wire->read();
	return true;
}

bool AS3935LightningSensor::writeRegister(uint reg, byte data)
{
	wire->beginTransmission(i2cAdds);
	wire->write((byte)reg);
	wire->write(data);
	if (wire->endTransmission() != 0) {
		AS3935_LOGERROR("endtx failed");
		restartWire();
		return false;
	}
	return true;
}


