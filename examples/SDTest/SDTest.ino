// IMU calibration values, insert your own here
#define IMU_ACCEL_OFFSET_X -1688
#define IMU_ACCEL_OFFSET_Y -382
#define IMU_ACCEL_OFFSET_Z 1205
#define IMU_GYRO_OFFSET_X -15
#define IMU_GYRO_OFFSET_Y -48
#define IMU_GYRO_OFFSET_Z -20

// #define N_FLAGS 8 // Small flag buffer to save space for Serial

#include "ArdIU.h"

void setup() {
  ArdIU::begin();
}

void loop() {
  // A simple single-deploy script
  
  do { // While we wait for accel readings...
    ArdIU::getFlags(); // Check for events
    if(ArdIU::isLiftoff()) { // If we have liftoff...
      digitalWrite(LED, LOW); // turn off LED, liftoff detected
      ArdIU::logData(250); // Log data, 250ms life
      ArdIU::getApogee(500, 0); // Check for apogee, 500ms verification time, no drop distance
      
      if(ArdIU::isApogee()) { // If apogee...
        ArdIU::fire(0, 1000); // fire first channel for 1 sec
      }
    } else { // if we're still on the ground...
      ArdIU::getLiftoff(3, 200); // Check for acceleration - 3 G for 0.2 sec
      digitalWrite(LED, HIGH); // red LED, ready for launch
    }
  } while(!ArdIU::imuInterrupt);
  ArdIU::getIMU(); // Now that we have a reading, store it
}
