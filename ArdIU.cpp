// ArdIU Library source
#include "ArdIU.h"

#define NO_CHANNEL -1
#define NO_FLAG -1

//#define LOUD

// default ArdIU pinout
byte ArdIU::pyroPins[CHANNELS];
byte ArdIU::contPins[CHANNELS];
ArdIU::Flag ArdIU::flagBuffer[N_FLAGS];

// environmental and hardware stuff
float ArdIU::vinScale = 1.0/11;
BMP280_DEV* ArdIU::pBaro = NULL;
MPU6050 ArdIU::imu;
SdFat ArdIU::SD;
char ArdIU::filename[] = "";
long int ArdIU::SDPos = 0;
byte ArdIU::bytesBuffered = 0;
bool ArdIU::isIMU = false;
bool ArdIU::isBaro = false;
bool ArdIU::isSD = false;
bool ArdIU::channelFired[CHANNELS];
float ArdIU::groundAlt = 0.0;
long int ArdIU::tLiftoff = 0;
long int ArdIU::tApogee = 0;
long int ArdIU::tBurnout = 0;
long int ArdIU::lastBaro = 0;
float ArdIU::smooth1 = 0.0;
float ArdIU::smooth2 = 0.0;
float ArdIU::smooth3 = 0.0;
float ArdIU::altitude = 0.0;
float ArdIU::altApogee = 0.0;
byte ArdIU::apogeeFlag = NO_FLAG;
byte ArdIU::burnoutFlag = NO_FLAG;
byte ArdIU::liftoffFlag = NO_FLAG;
byte ArdIU::buffer[BUF_SIZE];
// File ArdIU::flightLog;

byte ArdIU::imuDevStatus;
unsigned int ArdIU::imuPacketSize;
byte ArdIU::imuFifoBuffer[IMU_BUFFER_SIZE];

QuatF ArdIU::rotation;
VectorF ArdIU::accel;
VectorF ArdIU::vertical;
// VectorInt16 ArdIU::accelWorld;
// VectorFloat ArdIU::gravity;
volatile bool ArdIU::imuInterrupt;

void ArdIU::dmpDataReady() { imuInterrupt = true; }
 
void ArdIU::beepBoolean(bool input, int onTime, int offTime) {
#ifdef LOUD
	tone(BUZZER, 1000+(input?1000:0), onTime); // tone is parallel because it runs in a system register or something...
	delay(onTime+offTime); // so include on time in delay
#endif
}

void ArdIU::begin() {
	// initialize pyro pinout
	for(int i = 0; i < CHANNELS; i++) { // first 3 channels default, all others NO_CHANNEL
		if(i < 3) {
			pyroPins[i] = i+3;  // 3, 4, 5
			contPins[i] = i+A1; // A1, A2, A3
		} else {
			pyroPins[i] = NO_CHANNEL; // set pyro channels to unknown - with more than 3 there's no good way to set up defaults
			contPins[i] = NO_CHANNEL;
		}
	}
	pinMode(BUZZER, OUTPUT); // setup all the pins
	pinMode(LED, OUTPUT);
	pinMode(CS_SD, OUTPUT);
	digitalWrite(CS_SD, HIGH);
	pinMode(VIN, INPUT);
	
	initIMU();
	beepBoolean(isIMU, 100, 50); // check IMU, status tone
	initSD();
	beepBoolean(isSD, 100, 50); // check SD, status tone
	initBaro();
	beepBoolean(isBaro, 100, 50); // check baro, status tone
	
	
	for(int i = 0; i < CHANNELS; i++) { // setup pyroPins, beep out continuity
		pinMode(pyroPins[i], OUTPUT);
		pinMode(contPins[i], INPUT);
		digitalWrite(pyroPins[i], LOW);
	}
	for(int i = 0; i < CHANNELS; i++) { // setup pyroPins, beep out continuity
		beepBoolean(getCont(i), 250, 250);
	}
	
	setGroundAlt(); // establish zero
}
void ArdIU::setVinDiv(long int resGnd, long int resVin) { // for other devices, set the VIN voltage divider
	vinScale = ((float) resGnd)/(resGnd+resVin);
}
void ArdIU::setGroundAlt() {
	if(isBaro) {
		getAlt();  // sometimes first reading is bogus so dump it
		float total = 0.0;
		const int num = 20;
		for(int i = 0; i < num; i++) { // take a bunch of readings, 100ms apart, to correct for wind, etc
                    delay(100);
                    float alt = getAlt();
                    // Serial.println(alt);
                    total += alt;
		}	
		groundAlt = total/num;
	}
}
unsigned long int ArdIU::getMET() {
	if(tLiftoff > 0) {
		return millis()-tLiftoff;
	} else { return 0; } // if we haven't lifted off, leave MET at 0
}

float ArdIU::getAlt() {
  float alt = 0.0; // give the altitude somewhere to go
  pBaro -> startForcedConversion(); // tell the barometer to start reading
  while(! pBaro -> getAltitude(alt)); // wait until we get a reading, then dump it in alt; this is by reference and therefore looks kinda weird
  return alt;
}

float ArdIU::getAltSmoothed(int e_life) {
	// Quadratic exponential smoothing algorithm, a less known algorithm that attempts to fit data to a parabola
	// https://en.wikipedia.org/wiki/Exponential_smoothing
	float beta = exp(-((float) (getMET()-lastBaro))/e_life); // beta is basically an exponential decay parameter
	float alpha = 1.0 - beta;
	smooth1 = (getAlt()-groundAlt)*alpha + beta*smooth1;
	smooth2 = smooth1*alpha + beta*smooth2;
	smooth3 = smooth2*alpha + beta*smooth3;
   	
	lastBaro = getMET();
	return 3*(smooth1 - smooth2) + smooth3; // gonna be honest, I'm not sure how this works
	// float b = (alpha / (2*sq(1-alpha)) * ((6 - 5*alpha)*smooth1 - (10 - 8*alpha)*smooth2 + (4 + 3*alpha)*smooth3);
	// float c = (sq(alpha) / sq(1-alpha)) * (smooth1 - 2*smooth2 + smooth3);
	
}
float ArdIU::getVoltage(int analog_in) {
	return analog_in*3.3/vinScale/1024.0; // reading -> voltage @ pin -> voltage @ input
}
float ArdIU::getVin() {
	return getVoltage(analogRead(VIN)); // packaging these two functions for convenience
}
boolean ArdIU::getCont(int channel) {
	float voltage = getVoltage(analogRead(contPins[channel]));
	return voltage > getVin()*0.5 && contPins[channel] >= 0 && pyroPins[channel] >= 0;
}

#define MAX_FILES 100
void ArdIU::initSD() {
	isSD = SD.begin(CS_SD); // initialize SD library
        filename[0] = 0;  // strcpy(filename, "") by exploiting properties of null-terminated strings
	if(isSD) { // if the card exists
		int i = 0;
		while(i < MAX_FILES) { // iterate until we find a file that still hasn't been written
			char tryfilename[MAX_FILE_LEN];
			char num[4];
			itoa(i, num, 10); // convert the file number to a C string/char array
			strcpy(tryfilename, "flight");
			strcat(tryfilename, num);
			strcat(tryfilename, ".aiu"); // punch in flightXX.aiu via C string operations
			if (!SD.exists(tryfilename)) { // if the file is available...
				strcpy(filename,tryfilename); // make it official
				break; // we're done here
			}
			i++;
		} 
		SDPos = 0; // start at beginning of file
		store((long int) 3); // version header
	}
}

void ArdIU::storeBytes(const char* bytes, int size) { // to prevent inlining
	for(int i = 0; i < size; i++) { // iterate over data given
		buffer[bytesBuffered] = bytes[i]; // add each byte to the buffer & increment count
		bytesBuffered++;
		if(bytesBuffered >= BUF_SIZE) { // if it's full...
			File flightLog = SD.open(filename, FILE_WRITE); // open the SD card
			flightLog.seek(SDPos); // go to the proper position
			int written = flightLog.write(buffer, BUF_SIZE); // dump data
			SDPos = flightLog.position(); // read our new position
			flightLog.close(); // save file
			bytesBuffered = 0; // reset buffer
		}
	}
}

void ArdIU::initBaro() {
	if(pBaro) { delete pBaro; } // if it already exists, reset it
	if(SCK == 13 && MISO == 12 && MOSI == 11) { // if we're on default pinout, set up barometer with hardware SPI
		pBaro = new BMP280_DEV(CS_BARO);
		isBaro = pBaro -> begin(SLEEP_MODE, OVERSAMPLING_X4, OVERSAMPLING_X1, IIR_FILTER_OFF, TIME_STANDBY_05MS);
	} else { // assume there's no barometer
		pBaro = NULL;
		isBaro = false;
	}
}

void ArdIU::initIMU() {
	// Based on code by Jeff Rowberg
	// initialize available I2C implementation
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		#ifndef WIRE_COMPAT_MODE
			Wire.setClock(400000);
		#endif
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif
	
	imu.initialize(); // start IMU & ready interrupt
	pinMode(INTERRUPT, INPUT);
	
	isIMU = imu.testConnection(); // test if it worked

	imuDevStatus = imu.dmpInitialize();

        struct calibrationData { int ax, ay, az, gx, gy, gz; } calData;
	EEPROM.get(0, calData); // read calibration values from EEPROM
	
	imu.setXGyroOffset(calData.gx); // set calibration data
	imu.setYGyroOffset(calData.gy);
	imu.setZGyroOffset(calData.gz);
	imu.setXAccelOffset(calData.ax);
	imu.setYAccelOffset(calData.ay);
	imu.setZAccelOffset(calData.az);
	
	if(imuDevStatus == 0 && isIMU) { // if the IMU exists...
		imu.setDMPEnabled(true); // enable offboard processing
		
		attachInterrupt(digitalPinToInterrupt(INTERRUPT), dmpDataReady, RISING); // set up interrupt
		imuInterrupt = false;
		
		imuPacketSize = imu.dmpGetFIFOPacketSize(); // set packet size
		
		imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); // set accel to +/-16 G mode
	} else { isIMU = false; } // otherwise ignore it
}
void ArdIU::getIMU() {
	// Based on code by Jeff Rowberg

	imu.dmpGetCurrentFIFOPacket(imuFifoBuffer);
	imuInterrupt = false; // we've received the interrupt
	
	Quaternion imuQ;
	VectorInt16 imuAccel;
	imu.dmpGetQuaternion(&imuQ, imuFifoBuffer); // unpack the data we just got
	imu.dmpGetAccel(&imuAccel, imuFifoBuffer);
	
	rotation = toQuatF(imuQ);
	accel = toVectorF(imuAccel);
}

void ArdIU::logData(int baro_e_life) {
	long int met = getMET(); // store current mission clock
	store(met);
	long int statemask = 0;
	
	// STATEMASK THINGS
	statemask += (byte) (getVin()*16); // bits 0-7: battery voltage (0-16V * 16)
	for(int i = 0; i < CHANNELS && i < 8; i++) {
		statemask |= getCont(i) << (i+8); // place a 1 at each position that has continuity from bits 8 to 15
	}
	statemask += ((long) isIMU << 16) | ((long) isSD << 17) | ((long) isBaro << 18) |
                     ((long) isLiftoff() << 24) | ((long) isBurnout() << 25) | ((long) isApogee() << 26); // place sensor states and flight states in corresponding bit positions in statemask
	store(statemask);
	
	if (isBaro) {
		altitude = getAltSmoothed(baro_e_life);
		store(altitude); // store altitude value
		if(altitude > altApogee) { altApogee = altitude; } // check for apogee while we're here (this should really be reorganized)
	} else { store(0.0); } // if we don't have a reading, store dummy values to keep spacing
	if(isIMU) {
		store(getAccelX()); // store accel/gyro values
		store(getAccelY());
		store(getAccelZ());
		store(getTilt());
	} else {
		store(0.0);
		store(0.0);
		store(0.0);
		store(0.0);
	} // if we don't have a reading, store dummy values to keep spacing
	
}
byte ArdIU::setFlag(long int met, void (*event)()) {
	int i;
	for(i = 0; i < N_FLAGS && flagBuffer[i].isLive(); i++) {} // search for next open event
	if(i < N_FLAGS) {
		flagBuffer[i].set(met, event); // set the event in the first available slot
		return i; // we found a slot
	} else { return N_FLAGS; } // there were no slots available
}

void ArdIU::getFlags() {
	for(int i = 0; i < N_FLAGS; i++) { flagBuffer[i].getEvent(); } // check each flag in the array
}

void endFiring() { // flag function to shut off all pyros
	for(int i = 0; i < CHANNELS; i++) {
		digitalWrite(ArdIU::pyroPins[i], LOW);
	}
}
void ArdIU::fire(int channel, int time) { // set a pyro channel high, then set a flag to turn it off after a set time
	if(channel >= 0 && channel < CHANNELS && getCont(channel) && !channelFired[channel]) {
		digitalWrite(pyroPins[channel], HIGH);
		channelFired[channel] = true; // record that we fired this channel and shouldn't fire it again
		setFlag(millis()+time, endFiring);
	} // note: this implementation prevents overlapping firings but that shouldn't really affect anything realistically
}

bool ArdIU::isLiftoff() { return tLiftoff > 0; } // if any time is > 0, we've recorded it
bool ArdIU::isBurnout() { return tBurnout > 0; }
bool ArdIU::isApogee() { return tApogee > 0; }

void _atBurnout() { ArdIU::tBurnout = millis(); } // function to be run when we detect burnout
void _atLiftoff() { // function to be run when we detect liftoff
	ArdIU::tLiftoff = millis();
	ArdIU::vertical = ArdIU::accel;
	ArdIU::vertical.normalize(); // when liftoff detected, record our vertical vector to be our current acceleration
}
void _atApogee() { ArdIU::tApogee = millis(); } // function to be run when we detect apogee

void ArdIU::restartFlag(byte &flag, void (*event)(), int time) {
	if(flag != NO_FLAG) { flagBuffer[flag].setNotLive(); } // if the flag exists, remove it
	flag = setFlag(millis()+time, event); // and put a new one there with a reset timer and store the new location in flag
}

// Flight event detection is done via watchdog flags that are constantly reset whenever conditions are outside event parameters;
// only if conditions stay favorable long enough for the timer to burn completely does the event fire
void ArdIU::getLiftoff(float threshhold, int time) {
	if(getAccel() < threshhold && !isLiftoff()) { // if we detect insufficient acceleration, reset the flag's timer
		restartFlag(liftoffFlag, _atLiftoff, time);
	}
}
void ArdIU::getBurnout(int time) {
	if(accel.dot(vertical) > 0 && !isBurnout()) { // if we're accelerating upward, reset the flag's timer
		restartFlag(burnoutFlag, _atBurnout, time);
	}
}

void ArdIU::getApogee(int time, int altDrop) {
	if(altitude > altApogee - altDrop && !isApogee()) { // if we aren't more than a certain margin below our apogee, reset the flag's timer
      		restartFlag(apogeeFlag, _atApogee, time);
	}
}

float ArdIU::getTilt() {
	VectorF v = vertical;
	v.rotate(rotation); // rotate the vertical vector by the measured rotation
	return acos(v.dot(vertical)); // find the angle between it and the original vertical
}
