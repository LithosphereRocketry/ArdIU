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
      
      ArdIU::getBurnout(500);
      if(ArdIU::isApogee()) { // If apogee...
        ArdIU::fire(0, 1000); // fire first channel for 1 sec
      }
    } else { // if we're still on the ground...
      ArdIU::getLiftoff(30, 200); // Check for acceleration - 3 G for 0.2 sec
      digitalWrite(LED, HIGH); // red LED, ready for launch
    }
  } while(!ArdIU::imuInterrupt);
  ArdIU::getIMU(); // Now that we have a reading, store it
}
