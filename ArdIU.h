/* ArdIU Library header

A library designed for the ArdIU custom flight computer by Ben Graham. This library can be used in other
Arduino-based altimeters using the same major components, namely the MPU6050 gyroscope/accelerometer and the
BMP-280 pBarometric sensor. The library supports datalogging on an SD card, which is included as part of the
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


// Note to self: <Stream>.flush() is evil.

#ifndef ARDIU_H
#define ARDIU_H

#ifndef CHANNELS
#define CHANNELS 3
#endif

#ifndef BUF_SIZE
#define BUF_SIZE 32
#endif

#ifndef N_FLAGS
#define N_FLAGS 64
#endif

// core Arduino library
#include "Arduino.h"
//#include "math.h"

// datalogging things
#include "SPI.h"
#include "SdFat.h"

// pBarometer things
#include <BMP280_DEV.h>
#include <Device.h>


// IMU things
#include "I2Cdev.h"
#include "helper_3dmath.h"
#include "EEPROM.h"

// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#include "MPU6050.h"

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

class VectorInt16Improved: public VectorInt16 {
  public:
	int getMagnitude() { // Original function from helper_3dmath.h breaks on dimensions higher than 255
		return sqrt(((long) x)*x + ((long) y)*y + ((long) z)*z); // Cast to long to avoid overflow
	}
};

class BetterVectorFloat: public VectorFloat {
 public:
  BetterVectorFloat(float x, float y, float z): VectorFloat(x, y, z) {}
  BetterVectorFloat(): VectorFloat() {}
  float dotProduct(VectorFloat v) {
	  return v.x*x + v.y*y + v.z*z;
  }
};
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
	
	static long int tBurnout;
	static long int tApogee;
	static long int tLiftoff;
	
	static float altApogee;
	static float altitude;
	
	static void getIMU();
	
	static float getAccelX() { return accel.x/1024.0; }
	static float getAccelY() { return accel.y/1024.0; }
	static float getAccelZ() { return accel.z/1024.0; }
	static float getAccel()  { return accel.getMagnitude()/1024.0; }
	static VectorFloat getVF(VectorInt16 vector) { return VectorFloat(vector.x, vector.y, vector.z); }
	
	static void beepBoolean(bool input, int onTime, int offTime);
	
	class Flag {
	  private:
		unsigned long int time;  // high bit used for "live" boolean
		void (*event)();
	  public:
		long int getTime() { return time & TIME_MASK; }
		Flag() {
			time = 0;
			event = NULL;
		}
		void set(long int timeIn, void (*eventIn)()) {
			time = timeIn & TIME_MASK;
			event = eventIn;
			setLive();
		}
		void getEvent() {
			if(isLive() && millis() > getTime()) {
				(*event)();
				setNotLive();
			}
		}
		void setNotLive() { time = time & TIME_MASK; }
		void setLive() { time = time | LIVE_FLAG; }
		bool isLive() { return ((time & LIVE_FLAG) != 0); }
	};
	static byte setFlag(long int met, void (*event)());	
	static Flag flagBuffer[N_FLAGS];
	
	static MPU6050 imu;
	static BMP280_DEV* pBaro;
	static SdFat SD;
	static bool isIMU, isBaro, isSD;
	static byte pyroPins[CHANNELS], contPins[CHANNELS];
	static float groundAlt;
	// static void setPyroPins(int pyro_pins[CHANNELS], int cont_pins[CHANNELS]);
	static BetterVectorFloat vertical;
	static void setVinDiv(long int res1, long int res2); // set the standard voltage divider for unregulated pins
	static void setGroundAlt(); // calibrates the ground altitude; preferred against eventually
	static unsigned long int getMET();
	static float getAltSmoothed(int e_life); // returns the current pBarometric altitude, with smoothing
	static float getAlt();
	static float getVin(); // returns the voltage at the battery input, or about 0.5V less than VCC when powered over USB
	static float getVoltage(int analog_in); // scales an analog input to a divided voltage using the standard voltage divider
	static bool getCont(int channel);
	static void initSD();
	static void initBaro();
	static void initIMU();
	static void storeBytes(const char* bytes, int size);
	static float getTilt();
	template <class TYPE> 
	static void store(TYPE data) { // for some reason Arduino only allows templates in headers
		if(isSD) {
			const char* bytes = (const char*) &data;
		//	flightLog.write(bytes, sizeof(TYPE));
			storeBytes(bytes, sizeof(TYPE));
		}
	}
	
//  flightXX.aiu 
#define MAX_FILE_LEN 16

private:
	static void dmpDataReady();
//	static File flightLog;
	static char filename[MAX_FILE_LEN];
	static byte buffer[BUF_SIZE];
	static long int SDPos;
	static long int lastBaro;
	static byte imuFifoBuffer[IMU_BUFFER_SIZE];
	static float smooth1;
	static float smooth2;
	static float smooth3;
	static float vinScale;
	static Quaternion imuQ;
	static VectorInt16Improved accel;
//	static VectorInt16Improved accelWorld; // Gravity doesn't necessarily work with +/-16G mode
//	static VectorFloat gravity;
	
	static unsigned int imuPacketSize;
	static byte imuDevStatus;
	static byte bytesBuffered;
	static byte apogeeFlag, burnoutFlag, liftoffFlag;
    static void restartFlag(byte &flag, void (*event)(), int time);
};
#endif