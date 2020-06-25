// ArdIU Library source
#include "ArdIU.h"

// This library is set up weirdly, so it has to go in the source:
#include "MPU6050_6Axis_MotionApps20.h"

#define NO_CHANNEL -1
#define NO_FLAG -1

// default ArdIU pinout
byte ArdIU::pyroPins[CHANNELS];
byte ArdIU::contPins[CHANNELS];
ArdIU::Flag ArdIU::flagBuffer[N_FLAGS];

// environmental and hardware stuff
float ArdIU::vinScale = 1.0/11;
Adafruit_BMP280* ArdIU::pBaro = NULL;
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

Quaternion ArdIU::imuQ;
VectorInt16Improved ArdIU::accel;
BetterVectorFloat ArdIU::vertical;
// VectorInt16 ArdIU::accelWorld;
// VectorFloat ArdIU::gravity;
volatile bool ArdIU::imuInterrupt;

void ArdIU::dmpDataReady() { imuInterrupt = true; }
 
void ArdIU::beepBoolean(bool input, int onTime, int offTime) {
	tone(BUZZER, 1000+(input?1000:0), onTime);
	delay(onTime+offTime);
}

void ArdIU::begin() {
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
	beepBoolean(isIMU, 100, 50);
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
/* void ArdIU::setPyroPins(int pyro_pins[CHANNELS], int cont_pins[CHANNELS]) { // set all the output pins
	for(int i = 0; i < CHANNELS; i++) {
		pyroPins[i] = pyro_pins[i];
		contPins[i] = cont_pins[i];
	}
} */
void ArdIU::setVinDiv(long int resGnd, long int resVin) { // for other devices, set the VIN voltage divider
	vinScale = ((float) resGnd)/(resGnd+resVin);
}
void ArdIU::setGroundAlt() {
	if(isBaro) {
		float total = 0.0;
		const int num = 20;
		for(int i = 0; i < num; i++) {
            if (isSD) { SD.begin(CS_SD); }
			total += getAlt();
			delay(100);
		}	
		groundAlt = total/num;
	}
}
unsigned long int ArdIU::getMET() {
	if(tLiftoff > 0) {
		return millis()-tLiftoff;
	} else { return 0; }
}

float ArdIU::getAlt()
{
	return pBaro -> readAltitude(BASE_PRESSURE);
}

float ArdIU::getAltSmoothed(int e_life) {
	// Quadratic exponential smoothing algorithm, a less known algorithm that attempts to fit data to a parabola
	// https://en.wikipedia.org/wiki/Exponential_smoothing
	float beta = exp(-((float) (getMET()-lastBaro))/e_life);
	float alpha = 1.0 - beta;
	smooth1 = (getAlt()-groundAlt)*alpha + beta*smooth1;
	smooth2 = smooth1*alpha + beta*smooth2;
	smooth3 = smooth2*alpha + beta*smooth3;
   
	lastBaro = getMET();
	return 3*(smooth1 - smooth2) + smooth3;
	// float b = (alpha / (2*sq(1-alpha)) * ((6 - 5*alpha)*smooth1 - (10 - 8*alpha)*smooth2 + (4 + 3*alpha)*smooth3);
	// float c = (sq(alpha) / sq(1-alpha)) * (smooth1 - 2*smooth2 + smooth3);
	
}
float ArdIU::getVoltage(int analog_in) {
	return analog_in*3.3/vinScale/1024.0; // reading -> voltage @ pin -> voltage @ input
}
float ArdIU::getVin() {
	return getVoltage(analogRead(VIN));
}
boolean ArdIU::getCont(int channel) {
	float voltage = getVoltage(analogRead(contPins[channel]));
	return voltage > getVin()*0.5 && contPins[channel] >= 0 && pyroPins[channel] >= 0;
}

#define MAX_FILES 100
void ArdIU::initSD() {
	isSD = SD.begin(CS_SD);
        filename[0] = 0;  // strcpy(filename, "")
	if(isSD) {
		int i = 0;
		while(i < MAX_FILES) {
			char tryfilename[MAX_FILE_LEN];
			char num[4];
			itoa(i, num, 10);
			strcpy(tryfilename, "flight");
			strcat(tryfilename, num);
			strcat(tryfilename, ".aiu");
			if (!SD.exists(tryfilename)) {
				strcpy(filename,tryfilename);
				break;
			}
			i++;
		} 
		SDPos = 0;
		
	//	File flightLog = SD.open(filename, FILE_WRITE);
		store((long int) 3); // version header
	}
}

void ArdIU::storeBytes(const char* bytes, int size) { // to prevent inlining
	for(int i = 0; i < size; i++) {
		buffer[bytesBuffered] = bytes[i];
		bytesBuffered++;
		if(bytesBuffered >= BUF_SIZE) {
		//	SD.begin(CS_SD);
			File flightLog = SD.open(filename, FILE_WRITE);
			flightLog.seek(SDPos);
			int written = flightLog.write(buffer, BUF_SIZE);
			SDPos = flightLog.position();
		//	flightLog.flush();
			flightLog.close();
			bytesBuffered = 0;
		}
	}
}

void ArdIU::initBaro() {
	if(pBaro) { delete pBaro; }
	if(SCK == 13 && MISO == 12 && MOSI == 11) {
		pBaro = new Adafruit_BMP280(CS_BARO);		
	} else {
		pBaro = new Adafruit_BMP280(SCK, MISO, MOSI, CS_BARO);
	}
	isBaro = pBaro -> begin();		
//	Serial.println(isBaro);
}

void ArdIU::initIMU() {
	// Based on code by Jeff Rowberg
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		#ifndef WIRE_COMPAT_MODE
			Wire.setClock(400000);
		#endif
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif
	
	imu.initialize();
	pinMode(INTERRUPT, INPUT);
	
	isIMU = imu.testConnection();

	imuDevStatus = imu.dmpInitialize();

        struct calibrationData { int ax, ay, az, gx, gy, gz; } calData;
	EEPROM.get(0, calData);
	
	imu.setXGyroOffset(calData.gx);
	imu.setYGyroOffset(calData.gy);
	imu.setZGyroOffset(calData.gz);
	imu.setXAccelOffset(calData.ax);
	imu.setYAccelOffset(calData.ay);
	imu.setZAccelOffset(calData.az);
	
	if(imuDevStatus == 0 && isIMU) {
		imu.setDMPEnabled(true);
		
		attachInterrupt(digitalPinToInterrupt(INTERRUPT), dmpDataReady, RISING);
		imuInterrupt = false;
		
		imuPacketSize = imu.dmpGetFIFOPacketSize();
		
		imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); // set accel to +/-16 G mode
	} else { isIMU = false; }
}
void ArdIU::getIMU() {
	// Based on code by Jeff Rowberg
	
	// get current FIFO count
	unsigned int fifoCount = imu.getFIFOCount();
//	Serial.println(" FIFO counted");
	
	do {
		// reset interrupt flag and get INT_STATUS byte
		byte mpuIntStatus = imu.getIntStatus();
	//	Serial.println(" Status retrieved");
		
		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			imu.resetFIFO();
		//	Serial.println(" FIFO full, clearing");
			fifoCount = imu.getFIFOCount();
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
		} else if (mpuIntStatus & 0x02) {
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < imuPacketSize) { fifoCount = imu.getFIFOCount(); }
		//	Serial.println(" Full packet recieved");
			// read a packet from FIFO
			imu.getFIFOBytes(imuFifoBuffer, imuPacketSize);
		//	Serial.println(" Packet read");
			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= imuPacketSize;
		//	Serial.println(" Read, looking for more packets");
		}
	} while(fifoCount >= imuPacketSize);
//	Serial.println(" FIFO emptied");
	imuInterrupt = false;
	
	imu.dmpGetQuaternion(&imuQ, imuFifoBuffer);
	imu.dmpGetAccel(&accel, imuFifoBuffer);
}

void ArdIU::logData(int baro_e_life) {
	long int met = getMET();
	store(met);
	long int statemask = 0;
	statemask += (byte) (getVin()*16); //0-7: battery voltage (0-16V * 16)
	for(int i = 0; i < CHANNELS && i < 8; i++) {
		statemask |= getCont(i) << (i+8);
	}
	statemask += ((long) isIMU << 16) | ((long) isSD << 17) | ((long) isBaro << 18) |
                     ((long) isLiftoff() << 24) | ((long) isBurnout() << 25) | ((long) isApogee() << 26);
	store(statemask);
	
	if (isBaro) {
		altitude = getAltSmoothed(baro_e_life);
		store(altitude);
	}
	if(isIMU) {
		store(getAccelX());
		store(getAccelY());
		store(getAccelZ());
		store(getTilt());
	}
	
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
	for(int i = 0; i < N_FLAGS; i++) { flagBuffer[i].getEvent(); }
}

void endFiring() {
	for(int i = 0; i < CHANNELS; i++) {
		digitalWrite(ArdIU::pyroPins[i], LOW);
	}
}
void ArdIU::fire(int channel, int time) {
	if(channel >= 0 && channel < CHANNELS && getCont(channel) && !channelFired[channel]) {
		digitalWrite(pyroPins[channel], HIGH);
		channelFired[channel] = true;
		setFlag(millis()+time, endFiring);
	}
}

void _atApogee() { ArdIU::tApogee = millis(); }
void ArdIU::getApogee(int checkTime, int altDrop) {
	if(altitude > altApogee - altDrop && !isApogee()) {
		if(apogeeFlag != NO_FLAG) { flagBuffer[apogeeFlag].setNotLive(); }
		apogeeFlag = setFlag(millis()+checkTime, _atApogee);
	}
}

bool ArdIU::isLiftoff() { return tLiftoff > 0; }
bool ArdIU::isBurnout() { return tBurnout > 0; }
bool ArdIU::isApogee() { return tApogee > 0; }

void _atBurnout() { ArdIU::tBurnout = millis(); }
void _atLiftoff() {
	ArdIU::tLiftoff = millis();
	ArdIU::vertical = BetterVectorFloat(ArdIU::getAccelX(), ArdIU::getAccelY(), ArdIU::getAccelZ());
	ArdIU::vertical.normalize();
}
void ArdIU::getLiftoff(float threshhold, int time) {
	if(getAccel() < threshhold && !isLiftoff()) {
		if(liftoffFlag != NO_FLAG) { flagBuffer[liftoffFlag].setNotLive(); }
		liftoffFlag = setFlag(millis()+time, _atLiftoff);
	}
}
void ArdIU::getBurnout(int time) {
	BetterVectorFloat accf = BetterVectorFloat(accel.x, accel.y, accel.z);
	if(accf.dotProduct(vertical) < 0 && !isBurnout()) {
		if(burnoutFlag != NO_FLAG) { flagBuffer[burnoutFlag].setNotLive(); }
		burnoutFlag = setFlag(millis()+time, _atBurnout);
	}
}

float ArdIU::getTilt() {
	BetterVectorFloat v = BetterVectorFloat(vertical.x, vertical.y, vertical.z);
	v.rotate(&imuQ);
	return acos(v.dotProduct(vertical));
}
