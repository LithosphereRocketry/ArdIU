// ArdIU Library source
#include "ArdIU.h"

#define NO_CHANNEL -1
#define NO_FLAG -1

#define LOUD

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
ArdIU::FlightState ArdIU::state = ArdIU::NONE;
float ArdIU::groundAlt = 0.0;
unsigned long ArdIU::lastBaro = 0;
//float ArdIU::smooth1 = 0.0;
//float ArdIU::smooth2 = 0.0;
//float ArdIU::smooth3 = 0.0;
float ArdIU::altitude = 0.0;
float ArdIU::altApogee = 0.0;
float ArdIU::altLand = 0.0;
float ArdIU::landThresh = 10;
float ArdIU::liftoffAccel = 30;

HystCondition ArdIU::cLiftoff(200, stateConditionLiftoff);
HystCondition ArdIU::cBurnout(100, stateConditionBurnout);
HystCondition ArdIU::cApogee(1000, stateConditionApogee);
HystCondition ArdIU::cLand(10000, stateConditionLand);
byte ArdIU::buffer[BUF_SIZE];
ArdIU::Channel ArdIU::channels[] = {
	Channel(3, A1),
	Channel(4, A2),
	Channel(5, A3)
};

byte ArdIU::imuDevStatus;
unsigned int ArdIU::imuPacketSize;
byte ArdIU::imuFifoBuffer[IMU_BUFFER_SIZE];

QuatF ArdIU::rotation;
VectorF ArdIU::accel;
VectorF ArdIU::vertical;
VectorF ArdIU::worldVertical;
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
		channels[i].begin();
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
	
	for(int i = 0; i < CHANNELS; i++) { // beep out continuity
		beepBoolean(channels[i].getCont(), 250, 250);
	}
	
	pBaro -> startNormalConversion();
	setGroundAlt(); // establish zero
}
void ArdIU::setVinDiv(long int resGnd, long int resVin) { // for other devices, set the VIN voltage divider
	vinScale = ((float) resGnd)/(resGnd+resVin);
}
void ArdIU::setGroundAlt() {
	if(isBaro) {
		readAlt();  // sometimes first reading is bogus so dump it
		float total = 0.0;
		const int num = 20;
		for(int i = 0; i < num; i++) { // take a bunch of readings, 100ms apart, to correct for wind, etc
			delay(100);
			readAlt();
			// Serial.println(alt);
			total += altitude;
		}	
		groundAlt = total/num;
	}
}

unsigned long int ArdIU::getMET() {
	/*if(tLiftoff > 0) {
		return millis()-tLiftoff;
	} else { return 0; } // if we haven't lifted off, leave MET at 0*/
	return millis(); // debatable whether MET (after liftoff) is useful as a stat
}

void ArdIU::readAlt() {
  while(! pBaro -> getAltitude(altitude)); // wait until we get a reading, then dump it in alt; this is by reference and therefore looks kinda weird
}
/*
float ArdIU::getAltSmoothed(int e_life) { // might get rid of this, not sure it's needed or does much
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
*/
float ArdIU::getVoltage(int analog_in) {
	return analog_in*3.3/vinScale/1024.0; // reading -> voltage @ pin -> voltage @ input
}
float ArdIU::getVin() {
	return getVoltage(analogRead(VIN)); // packaging these two functions for convenience
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
void ArdIU::readIMU() {
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

void ArdIU::logData() {
	DataFrame currentFrame;
	
	currentFrame.time = getMET(); // store current mission clock
	
	// STATEMASK THINGS
	currentFrame.state += (byte) (getVin()*16); // bits 0-7: battery voltage (0-16V * 16)
	for(int i = 0; i < CHANNELS && i < 8; i++) {
		currentFrame.state |= channels[i].getCont() << (i+8); // place a 1 at each position that has continuity from bits 8 to 15
	}
	currentFrame.state += ((long) isIMU << 16) | ((long) isSD << 17) | ((long) isBaro << 18) |
                     ((long) state << 24); // place sensor states and flight states in corresponding bit positions in statemask
	
	if (isBaro) {
		currentFrame.altitude = altitude;
		if(currentFrame.altitude > altApogee) { altApogee = currentFrame.altitude; } // check for apogee while we're here (this should really be reorganized)
	} else { currentFrame.altitude = 0; } // if we don't have a reading, store dummy values to keep spacing
	if(isIMU) {
		currentFrame.accx = getAccelX(); // store accel/gyro values
		currentFrame.accy = getAccelY(); // store accel/gyro values
		currentFrame.accz = getAccelZ(); // store accel/gyro values
		currentFrame.tilt = getTilt();
	} else {
		currentFrame.accx = 0;
		currentFrame.accy = 0;
		currentFrame.accz = 0;
		currentFrame.tilt = 0;
	}
	store(currentFrame);
}

void ArdIU::update() {
	if(imuInterrupt) {
		readIMU();
	}
	readAlt();
	switch(state) {
		case READY:
			if(cLiftoff.update(millis())) {
				ArdIU::vertical = ArdIU::accel.normalize();
				ArdIU::worldVertical = ArdIU::vertical.rotate(ArdIU::rotation);
				state = BOOST;
				cBurnout.reset(millis());
			}
			break;
		case BOOST:
			if(cBurnout.update(millis())) {
				state = COAST;
				cApogee.reset(millis());
				cLiftoff.reset(millis());
			}
			break;
		case COAST:
			if(cLiftoff.update(millis())) {
				state = BOOST;
				cBurnout.reset(millis());
			} else if(cApogee.update(millis())) {
				state = DESCENT;
				cLand.reset(millis());
			}
			break;
		case DESCENT:
			if(cLand.update(millis())) {
				state = LAND;
			}
			break;
		case LAND:
			break;
		case NONE:
		default:
			break;
	}
	logData();
}

float ArdIU::getTilt() {
	VectorF v = worldVertical.rotate(~rotation);
	return acos(vertical.dot(v)); // find the angle between it and the original vertical
}


// Channels
ArdIU::Channel::Channel(byte p, byte c): pin(p), contPin(c), HystCondition() {
	
}
void ArdIU::Channel::begin() {
	pinMode(pin, OUTPUT);
	pinMode(contPin, INPUT);
	digitalWrite(pin, LOW);
	reset();
}
void ArdIU::Channel::update() {
	unsigned long t = millis(); // no idea if this saves anything
	
	if(HystCondition::update(t) && (canRefire || !fired)) {
		digitalWrite(pin, HIGH);
		fireStartTime = t;
		fired = true;
	}
	
	if(t - fireStartTime > fireTime) {
		digitalWrite(pin, LOW);
	}
}
void ArdIU::Channel::reset() {
	HystCondition::reset(millis());
}
bool ArdIU::Channel::getCont() {
	return getVoltage(analogRead(contPin)) > getVin()*0.5;
}