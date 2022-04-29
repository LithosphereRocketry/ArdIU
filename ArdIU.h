/* ArdIU Library header

A library designed for the ArdIU custom flight computer by Ben Graham. This library can be used in other
Arduino-based altimeters using the same major components, namely the MPU6050 gyroscope/accelerometer and the
BMP-280 barometric sensor. The library supports datalogging on an SD card, which is included as part of the
standard ArdIU.

This library builds on stock Arduino libraries as well as the Adafruit Sensor and BMP280 libraries as well as
Jeff Rowberg's I2CDevLib and MPU6050 libraries. These libraries must be installed in order for the program to
function.

The following copyright notice applies to the portions of this library derived from and/or directly including
Jeff Rowberg's I2CDev Library. The library itself is included as part of the ArdIU package, and its example
code was used in constructing the ArdIU core library.

I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/*
== Known issues ==
*/

#ifndef ARDIU_H
#define ARDIU_H

// core Arduino library
#include "Arduino.h"
//#include "math.h"

// datalogging things
#include "SPI.h"
#include "SdFat.h"

// barometer things
#include <BMP280_DEV.h>
#include <Device.h>


// IMU things
#include "EEPROM.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "VectorMath_I2CDev.h"

#include "HystCondition.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

// system & operation parameters
#define BASE_PRESSURE 1013.25
#define CHANNELS 3
#define BUF_SIZE 32
#define N_FLAGS 16

// Defaolt pinout definitions
#define SCK 13
#define MISO 12
#define MOSI 11
#define CS_BARO 9
#define CS_SD 10
#define BUZZER 7
#define LED 6
#define VIN A0
#define INTERRUPT 2


#define LIVE_FLAG (((unsigned long int)1) << 31)
#define TIME_MASK (LIVE_FLAG - 1)

#define IMU_BUFFER_SIZE 64

class ArdIU {
public:
	enum FlightState {
		READY = 'r',
		BOOST = 'b',
		COAST = 'c',
		DESCENT = 'd',
		LAND = 'l',
		NONE = 'n'
	};
	
	class Channel: public HystCondition {
	  public: // as is, this class is very memory-inefficient; to be improved later
		// the goal here is to get a functional interface and then trim it down as much as possible
		
		// hardware, might be able to optimize away
		const byte pin; 
		const byte contPin;
		// user settings, changed once
		bool canRefire;
		unsigned int fireTime; // time to fire for (1 ms - 65 sec)
		unsigned int hystTime; // how long it must maintain the given conditions (1 ms - 65 sec)
		
		// state variables
		bool fired;
		
		// functions
		Channel(byte p, byte c);
		void begin();
		void update();
		void reset();
		bool getCont();
	  private:
		// state variables
		unsigned long fireStartTime; // these are big clunky int32s that can probably be trimmed
		unsigned long hystStartTime;
	};
// USER FUNCTIONS
	// Setup & initialization
	static void begin();	
	static inline void setReady() {
		state = READY;
	}

	static void logData(); 
	// Collects data and logs to the SD card.
	
	static void update();
	
	static volatile bool imuInterrupt; // IMU interrupt status flag
	
	static float altitude;
	static float altApogee; // highest altitude recorded so far
	static float altLand;
	
	static void readIMU(); // pull data from IMU buffer
	static void readAlt(); // returns unsmoothed barometric altitude
	
	// get acceleration values, converted from unitless sensor values
	static float getAccelX() { return accel.x; } 
	static float getAccelY() { return accel.y; }
	static float getAccelZ() { return accel.z; }
	static float getAccel()  { return accel.mag(); }
	
	static Channel channels[CHANNELS];
	
	static void beepBoolean(bool input, int onTime, int offTime); // play a status tone, either true (high note) or false (low note)
	
	static MPU6050 imu; // IMU object
	static BMP280_DEV* pBaro; // pointer to barometer object because this library prefers that
	static SdFat SD; // SD card object
	
	static bool isIMU, isBaro, isSD; // flags for whether each core system is active
	
	static FlightState state;
	static float groundAlt; // ground altitude
	static VectorF vertical; // vector representing initial acceleration
	static VectorF worldVertical; // vector representing initial acceleration, rotated
	static QuatF rotation; // rotation from IMU
	static VectorF accel; // acceleration from IMU
	
	static inline float axialAccel() {
		return accel.dot(vertical);
	}
	
	static inline void setLiftoffCondition(float minAccel, unsigned int time) {
		liftoffAccel = minAccel;
		cLiftoff.hystTime = time;
	}
	static inline void setBurnoutCondition(unsigned int time) {
		cBurnout.hystTime = time;
	}
	static inline void setApogeeCondition(unsigned int time) {
		cApogee.hystTime = time;
	}
	static inline void setLandingCondition(float landTolerance, unsigned int time) {
		landThresh = landTolerance;
		cLand.hystTime = time;
	}
	static inline void setPyroCondition(byte channel, bool (*cond)(), unsigned int hystTime, unsigned int fireTime) {
		channels[channel].condition = cond;
		channels[channel].hystTime = hystTime;
		channels[channel].fireTime = fireTime;
	}
	static void setVinDiv(long int res1, long int res2); // set the standard voltage divider for unregulated pins
	static void setGroundAlt(); // calibrates the ground altitude; preferred against eventually
	
	static unsigned long int getMET(); // returns time since liftoff detect
	//static float getAltSmoothed(int e_life); // returns the current barometric altitude, with smoothing
	static float getVin(); // returns the voltage at the battery input, or about 0.5V less than VCC when powered over USB
	static float getVoltage(int analog_in); // scales an analog input to a divided voltage using the standard voltage divider
	static float getTilt(); // returns current off-axis tilt from IMU
	
	static void initSD(); // initialize the SD card
	static void initBaro(); // initialize the barometer
	static void initIMU(); // initialize the IMU
	
	static void storeBytes(const char* bytes, int size); // write bytes to the SD card, only opening the card when the buffer fills
	
	// template for writing any data type to the SD card
	template <class TYPE> 
	static void store(TYPE data) { // for some reason Arduino only allows templates in headers
		if(isSD) {
			const char* bytes = (const char*) &data; // tell C that, whatever this data was, treat it as if it were raw bytes/chars
		//	flightLog.write(bytes, sizeof(TYPE));
			storeBytes(bytes, sizeof(TYPE)); // write to the card
		}
	}
	
	struct DataFrame {
		unsigned long time;
		unsigned long state;
		float altitude;
		float accx, accy, accz;
		float tilt;
	};
	
	// Stock conditions that users will want
	template <int alt>
	static bool conditionAltDescent() {
		return state == DESCENT && altitude < alt;
	}
	
	static bool conditionApogee() {
		return state == DESCENT;
	}
	
	static bool conditionBurnout() {
		return state == COAST;
	}
	
	template<int angleDeg>
	static bool conditionBurnoutTilt() {
		return state == COAST && getTilt() < angleDeg*DEG_TO_RAD;
	}
	
//  flightXX.aiu 
#define MAX_FILE_LEN 16

private:
	static float liftoffAccel;
	static float landThresh;
	
	static void dmpDataReady(); // interrupt for when IMU data is ready
	static char filename[MAX_FILE_LEN]; // current filename to write to
	static byte buffer[BUF_SIZE]; // current data buffer to be written
	static long int SDPos; // current SD card write location
	static unsigned long lastBaro; // last time a barometer reading was recorded, in MET
	static HystCondition cLiftoff, cBurnout, cApogee, cLand; // these timers are an improvement but still clunky
	static byte imuFifoBuffer[IMU_BUFFER_SIZE]; // buffer for incoming IMU data
	
	//static float smooth1; // smoothing parameters for barometer data
	//static float smooth2; // may get rid of this
	//static float smooth3;
	
	static float vinScale; // scaling factor for battery readings
	
	static unsigned int imuPacketSize; // packet size for IMU buffer
	static byte imuDevStatus; // status reading from IMU
	static byte bytesBuffered; // number of bytes currently waiting to be written to card
	
	// conditions the user shouldn't need
	static bool stateConditionLiftoff() {
		return getAccel() > liftoffAccel;
	}
	static bool stateConditionBurnout() {
		return axialAccel() < 0;
	}
	static bool stateConditionApogee() {
		if(altitude <= altApogee) {
			return true;
		} else {
			altApogee = altitude;
			return false;
		}
	}
	static bool stateConditionLand() {
		if(abs(altitude - altLand) < landThresh) {
			return true;
		} else {
			altLand = altitude;
			return false;
		}
	}
};
#endif
