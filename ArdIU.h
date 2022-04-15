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
// USER FUNCTIONS
	// Setup & initialization
	static void begin();	

	static void logData(int e_life); 
	// Collects data and logs to the SD card. Input- smoothing lifespan, typically a few hundred ms.

	static void getFlags();
	// Checks inflight events- almost always necessary.

	static void getApogee(int time, int altDrop);
	// Checks whether apogee has occured. Input- verification time, typically around 500-1000 ms,
	// verification drop, set to 0 for standard apogee detect or an altitude in meters to add a
	// descent distance threshhold
	
	static void getLiftoff(float threshhold, int time);
	// Checks whether liftoff has occured based on accel. Input- trigger acceleration, verification time.
	
	static void getBurnout(int time);
	// Checks whether burnout has occured based on direction of acceleration. Input- verification time.
	
	static bool channelFired[CHANNELS];
	// Array storing whether each channel has fired. Channels are numbered 0-n (0-2 on a standard ArdIU).
	
	static void fire(int channel, int time); // Fires the specified channel for the specified time (ms).
	// Avoid firing multiple channels in such a way that they "overlap"- ArdIU will stop all channels as
	// soon as one channel's time ends.
	
	static bool isApogee(); // Returns whether the altimeter has detected apogee.
	static bool isLiftoff(); // Returns whether the altimeter has detected liftoff.
	static bool isBurnout(); // Returns whether the altimeter has detected motor burnout.
	
	static volatile bool imuInterrupt; // IMU interrupt status flag
	
	static long int tBurnout; // time, relative to millis() system clock, of burnout detection
	static long int tApogee; // time, relative to millis() system clock, of apogee detection
	static long int tLiftoff; // time, relative to millis() system clock, of liftoff detection
	
	static float altApogee; // highest altitude recorded so far
	static float altitude; // current altitude as of last data frame
	
	static void getIMU(); // pull data from IMU buffer
	
	// get acceleration values, converted from unitless sensor values
	static float getAccelX() { return accel.x; } 
	static float getAccelY() { return accel.y; }
	static float getAccelZ() { return accel.z; }
	static float getAccel()  { return accel.mag(); }
	
	static void beepBoolean(bool input, int onTime, int offTime); // play a status tone, either true (high note) or false (low note)
	
	class Flag { // Flag class allows arbitrary events to be scheduled for a given time; it uses a bitmask to save memory 
		     // (since booleans are stored as larger types, it actually saves a significant amount)
		     // the compiler requires this to be in the header for some reason
	  private:
		unsigned long int time;  // high bit used for "live" boolean
		void (*event)(); // low level C hackery to pass a function as a parameter
	  public:
		long int getTime() { return time & TIME_MASK; } // mask away the live bit
		Flag() { // initialize a blank flag
			time = 0;
			event = NULL;
		}
		void set(long int timeIn, void (*eventIn)()) {
			time = timeIn & TIME_MASK; // overlay the flag time using the bitmask
			event = eventIn;
			setLive();
		}
		void getEvent() { // see if the event should happen
			if(isLive() && millis() > getTime()) { // if the time has passed and it's active...
				(*event)(); // run it
				setNotLive(); // set it inactive
			}
		}
		
		void setNotLive() { time = time & TIME_MASK; } // removes active flag from time:
		//   time      Fxxxxxxx xxxxxxxx xxxxxxxx xxxxxxxx
		// & TIME_MASK 01111111 11111111 11111111 11111111
		// = time      0xxxxxxx xxxxxxxx xxxxxxxx xxxxxxxx
		
		void setLive() { time = time | LIVE_FLAG; } // add active flag to time:
		//   time      Fxxxxxxx xxxxxxxx xxxxxxxx xxxxxxxx
		// | LIVE_FLAG 10000000 00000000 00000000 00000000
		// = time      1xxxxxxx xxxxxxxx xxxxxxxx xxxxxxxx
		
		bool isLive() { return ((time & LIVE_FLAG) != 0); } // reads active flag from time:
		//   time      Fxxxxxxx xxxxxxxx xxxxxxxx xxxxxxxx
		// & LIVE_FLAG 10000000 00000000 00000000 00000000
		// =           F0000000 00000000 00000000 00000000
	};
	static byte setFlag(long int met, void (*event)()); // add a flag to the buffer
	static Flag flagBuffer[N_FLAGS]; // buffer of flags that will be automatically handled
	
	static MPU6050 imu; // IMU object
	static BMP280_DEV* pBaro; // pointer to barometer object because this library prefers that
	static SdFat SD; // SD card object
	static bool isIMU, isBaro, isSD; // flags for whether each core system is active
	static byte pyroPins[CHANNELS], contPins[CHANNELS]; // list of pin assigments for pyrotechnic channels
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
	static bool getCont(int channel); // returns whether continuity is detected on a given pyro channel
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
	
//  flightXX.aiu 
#define MAX_FILE_LEN 16

private:
	static void dmpDataReady(); // returns whether data is waiting
//	static File flightLog;
	static char filename[MAX_FILE_LEN]; // current filename to write to
	static byte buffer[BUF_SIZE]; // current data buffer to be written
	static long int SDPos; // current SD card write location
	static long int lastBaro; // last time a barometer reading was recorded, in MET
	static byte imuFifoBuffer[IMU_BUFFER_SIZE]; // buffer for incoming IMU data
	static float smooth1; // smoothing parameters for barometer data
	static float smooth2;
	static float smooth3;
	static float vinScale; // scaling factor for battery readings
//	static VectorInt16Improved accelWorld; // Gravity doesn't necessarily work with +/-16G mode so these are ignored
//	static VectorFloat gravity;
	
	static unsigned int imuPacketSize; // packet size for IMU buffer
	static byte imuDevStatus; // status reading from IMU
	static byte bytesBuffered; // number of bytes currently waiting to be written to card
	static byte apogeeFlag, burnoutFlag, liftoffFlag; // addresses of various important flags; this might be reworked later
	static void restartFlag(byte &flag, void (*event)(), int time); // restart a flag's timer
};
#endif
