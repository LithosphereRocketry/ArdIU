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

#ifndef BASE_PRESSURE
#define BASE_PRESSURE 1013.25
#endif

#ifndef ARDIU_H
#define ARDIU_H

#ifndef CHANNELS
#define CHANNELS 3
#endif

#ifndef BUF_SIZE
#define BUF_SIZE 32
#endif

#ifndef N_FLAGS
#define N_FLAGS 16
#endif

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

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

// Defaolt pinout definitions
#ifndef SCK
#define SCK 13
#endif

#ifndef MISO
#define MISO 12
#endif

#ifndef MOSI
#define MOSI 11
#endif

#ifndef CS_BARO
#define CS_BARO 9
#endif

#ifndef CS_SD
#define CS_SD 10
#endif

#ifndef BUZZER
#define BUZZER 7
#endif

#ifndef LED
#define LED 6
#endif

#ifndef VIN
#define VIN A0
#endif

#ifndef INTERRUPT
#define INTERRUPT 2
#endif

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
	
	class Channel {
	  public: // as is, this class is very memory-inefficient; to be improved later
		// the goal here is to get a functional interface and then trim it down as much as possible
		
		// hardware, might be able to optimize away
		const byte pin; 
		const byte contPin;
		// user settings, changed once
		char state;
		bool canRefire;
		unsigned int fireTime; // time to fire for (1 ms - 65 sec)
		// Setting conditions for each possible parameter is inefficent and inflexible
		// Bettery way to manage conditions: function pointers
		// The user writes a function that returns whether or not the flight is within conditions and puts it here
		bool (*condition)();
		unsigned int hystTime; // how long it must maintain the given conditions (1 ms - 65 sec)
		
		// state variables
		bool fired;
		
		// functions
		Channel(byte p, byte c);
		void begin();
		void update();
		bool getCont();
		
		static bool condNever();
	  private:
		// state variables
		unsigned long fireStartTime; // these are big clunky int32s that can probably be trimmed
		unsigned long hystStartTime;
		
	};
// USER FUNCTIONS
	// Setup & initialization
	static void begin();	

	static void logData(int e_life); 
	// Collects data and logs to the SD card. Input- smoothing lifespan, typically a few hundred ms.
	
	static void update();
	
	static volatile bool imuInterrupt; // IMU interrupt status flag
	
	static float altitude;
	static float altApogee; // highest altitude recorded so far
	
	static void getIMU(); // pull data from IMU buffer
	
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

	static void setVinDiv(long int res1, long int res2); // set the standard voltage divider for unregulated pins
	static void setGroundAlt(); // calibrates the ground altitude; preferred against eventually
	
	static unsigned long int getMET(); // returns time since liftoff detect
	static float getAlt(); // returns unsmoothed barometric altitude
	static float getAltSmoothed(int e_life); // returns the current barometric altitude, with smoothing
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
	
//  flightXX.aiu 
#define MAX_FILE_LEN 16

private:
	static void dmpDataReady(); // interrupt for when IMU data is ready
	static char filename[MAX_FILE_LEN]; // current filename to write to
	static byte buffer[BUF_SIZE]; // current data buffer to be written
	static long int SDPos; // current SD card write location
	static unsigned long lastBaro; // last time a barometer reading was recorded, in MET
	static unsigned long lastStateReset; // last time the current hysteresis state was reset
	static byte imuFifoBuffer[IMU_BUFFER_SIZE]; // buffer for incoming IMU data
	
	static float smooth1; // smoothing parameters for barometer data
	static float smooth2; // may get rid of this
	static float smooth3;
	
	static float vinScale; // scaling factor for battery readings
	
	static unsigned int imuPacketSize; // packet size for IMU buffer
	static byte imuDevStatus; // status reading from IMU
	static byte bytesBuffered; // number of bytes currently waiting to be written to card
};
#endif
