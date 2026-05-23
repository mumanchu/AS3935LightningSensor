# AS3935LightningSensor Library

## ** PRELIMINARY **
_Let me know if it doesn't work..._

Full details of this project can be found in the blog \
https://muman.ch/muman/index.htm?muman-as3935-lightning-sensor.htm

A Windows application has been developed (and is currently being enhanced), which works with the example sketch.
https://muman.ch/muman/index.htm?muman-as3935-lightning-sensor-app.htm

![gravity module](https://muman.ch/muman/as3935-gravity.png)
![sparkfun module](https://muman.ch/muman/as3935-sparkfun.png)


## Summary 
"_The AS3935 is a programmable fully integrated Lightning Sensor IC that detects the presence and approach of potentially hazardous lightning activity in the vicinity and provides an estimation of the distance to the head of the storm._"

Here's a summary of the 40-page data sheet, so you don't have to read it...

The chip uses an inductor/capacitor resonant circuit as an antenna, which is included on the module. This must be tuned to resonate at 500kHz +-3%, see below.

The chip's Analog Front End (AFE) detects incoming signals that are above the "watchdog threshold" and alerts the "lightning algorithm block". This uses a "disturber rejection algorithm" to determine if
it's a genuine lightning strike or a "disturber".

A "disturber" is a man-made (or machine-made) source of interference that could be misinterpreted as a lightning strike. If the algorithm determines the signal was a "disturber" it disables the sensor for 1.5 seconds. Background noise is also registered, as the sensor will not work if the background noise is too high.

The sensitivity of these checks can be configured with `setAFEGain()`, `setWatchdogThreshold()`, `setSpikeRejection()` and `setNoiseFloorLevel(`). But it's probaby best to leave these at their default values because it's difficult to test. For indoor or outdoor use, `setAFEGain()` has two optimal settings (`INDOOR` and `OUTDOOR`). The default is `INDOOR`.

If it's determined to be a lightning strike, the event is stored in a circular buffer from which it uses a statistical algorithm to estimate the distance of the storm in kilometers. Further strikes will not be detected for 1 second. The circular buffer can be cleared with `clearStatistics()`, which sets the storm distance to 0.

This is the link to the manufacturer's web page, they call it the 'Franklin Lightning Sensor' \
https://www.sciosense.com/franklin-lightning-sensor/


## Class Reference

Read the commented source code for details.

```cpp
class AS3935LightningSensor
{
  ...
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
};
```

## Example Sketch

TODO

An example sketch that works with the Windows application is being tested on various devices. This is being updated to do long term logging to flash memory using a device with a real time clock and an uniterruptable power supply. The log will hold many years worth of events. The Windows app can upload and display the log.
<br/> <br/>

## Readings

The distance to the leading edge of the storm is estimated in km, and returned by `getStormDistance()`: 1 = overhead, 63 = out of range. When a lighting strike is detected, it stores the energy level, 
which is read with `getStrikeEnergy()` as a unitless 21-bit value. 

If using the IRQ output as an event signal, call `getIRQSource()` to determine what happened.

## IRQ Output

See data sheet p34, see the link at end.

The active-high IRQ output is turned on when:
* A programmed number of lightning strikes have occurred, see `setMinimumStrikes()`
* Noise levels are exceeded and measurements can no longer be taken, see `setNoiseFloorLevel()`
* A disturber event was detected, this interrupt can be disabled with `disableDisturberInterrupt()`

You could use the IRQ output to generate a interrupt, but I think it's best just to poll it as a status pin, which is way faster than polling the chip with I2C messages. If IRQ goes high, you must wait 2ms before determining the interrupt reason using `getIRQSource()`. 

The signal from the internal oscillators can also be connected to the IRQ output pin for frequency measurement and calibration, see `selectIRQOutput()`.

## I2C Address

The I2C address is defined by how A1 and A0 are connected. My Gravity/DFRobot module uses switches to assign the address:
| A1 | A0 |   |
| ----- | ----- | ----- | 
|0 | 0  | INVALID! |
| 0 | 1 |  0x01 |
| 1 | 0 |  0x02 |
| 1 | 1 |  0x03 |

The chip supports SPI communications too, but the Gravity/DFRobot board allows only I2C. If you need SPI, just add a new `begin()` and write new `readRegister()` and `writeRegister()` methods for SPI.

## Tuning the Antenna's Resonant Frequency

See data sheet p35.

The antenna has an LCO oscillator (LCO = inductor/capacitor oscillator) which should be tuned to have a resonant frequency of 500kHz +-3.5%. The oscillator frequency can be output on the IRQ pin for measurement
with an accurate frequency counter, see `selectIRQOutput()`. 

Tuning is done using `setTuningCapacitor()` to select the TUN_CAP bits for 0..120pF in steps of 8pF. Once you have this value, put it in the code so it's configured in `setup()`.

## Clock Frequency Calibration

See data sheet p35.

The chip contains two RC (resistor-capacitor) oscillators: 

* SRCO system RC oscillator 1.1MHz
* TRCO timer RC oscillator 32.768kHz

Unlike the antenna, these do not need manual calibration. They are calibrated internally after power-up by `calibrateOscillators()`. You may not need to call this yourself because it's already called by `begin()` and `powerUp()`, but you must call it if you use `reset()`. 

These clocks are derived from the antenna LCO oscillator, so that should be manually calibrated first. If you want to check these frequencies, use `selectIRQOutput()` to output the signal on the IRQ pin. 


## Testing

You can either wait for the next thunderstorm (as did Benjamin Franklin), or use some kind of spark generator like a Tesla Coil. This will be wildy inaccurate, but it will indicate that something is happening.

I used a 1970's ZEROSTAT antistatic pistol, a piezo-electric device which generates a spark if you pull the trigger too fast. Piezo cigarette and gas lighters will probably do the same thing (but I didn't try it). 

> [!CAUTION]
> Electrostatic discharges (and lightning strikes) are LETHAL to modern electronics! See Disclaimer.

Alternatively, make a continuous spark generator with the ignition coil from that rusting Maserati in your garage. \
See https://muman.ch/muman/index.htm?muman-ignition-coil-driver.htm.

Tesla coils and piezo-electric devices caused I2C communications errors for me, and the I2C communications hung. This may be due to interference through the long off-board I2C wiring. I added the [temporary] `restartWire()` method which lets it recover.

## Data Sheet

All page references refer to THIS version of the data sheet \
https://www.sciosense.com/wp-content/uploads/2023/12/AS3935-Data-Sheet.pdf

## Gravity/DFRobot Module

https://www.dfrobot.com/product-1828.html

## Windows Application

![AS3935 Lightning Detector App](https://muman.ch/muman/lightning-detector-app-4.png)

## Revision History

| Date       | Version  | Description |
|:---------- |:---------|:----------- |
| 2026.05.21 | 0.0.0	| Preliminary |

<br/>


## Joke of the Week

I needed _a nap_, not _an App_, you fule!




