////// TODO
/// Arming procedure
/// Custom libraries
/// Angle controller

//#define RATE 1000  // rate to define speed of entire controller, MPU takes 834us so max rate is 1190.47619 (timeStep = 840us)
//uint32_t timeStep = (uint32_t)(1000000 / (double)RATE);

bool radioLimitsSet = false;

#include "CONSTANTS_DRG.h"
#include "MPU6050_DRG.h"
#include "RADIO_DRG.h"
#include "CONTROLLER_DRG.h"

Mpu6050 mpu;
Radio radio;
Controller controller;

///// ===== EEPROM Addressbook
//// 0 - 12 = reserved for float MPU cals  // gP, gR, gY
//// 14 - 25 = reserved for float (4 bytes) pid values

/// Radio Channels
// pitch 0
// roll 1
// yaw 2
// throttle 3
// aux1 4
// aux2 5

///// =====

bool armed = false;

bool doMpuCal = false;
bool doThrusterCal = true;


int main() {

  uint64_t timePrev = 0;
  uint64_t radioLastUpdate = 0;  
  radioData_t controls;
  int runState = 0;
  float auxASwPrev = 0;
  float auxBSwPrev = 0;

  controller.setup(doThrusterCal);  // must be done first - servos start at power-on!
  mpu.setup(doMpuCal);
  // ISR as lambdas for correct casting
  radio.setup( []{radio.ch0Interrupt();}, []{radio.ch1Interrupt();}, []{radio.ch2Interrupt();}, []{radio.ch3Interrupt();}, []{radio.ch4Interrupt();}, []{radio.ch5Interrupt();} );

  while (1) { // run loop

    // === time keeping
    uint64_t timeNow = micros();
    uint32_t timeDelta = timeNow - timePrev;


    // === run radio script as fast as possible
    if (radio.getNew()) {
      radio.resetNew();  // reset flag
      radioLimitsSet = radio.run(&controls);
      radioLastUpdate = timeNow;
    }


    // === run once every timeStep
    if (timeDelta >= timeStep) {
      timePrev += timeDelta;

      // poll MPU for new data
      mpu.waitForNewAngles();
      angleData_t & angles = mpu.getAngles();
      angleData_t & angles_filtered = mpu.getFilteredAngles();

      switch(runState){
        case 0:  // off
          controller.zeroThrusters();
          controller.resetPidError();
          if(radioLimitsSet && abs(controls.pitch - 50.0) < 10.0 && abs(controls.roll - 50.0) < 10.0 && (timeNow - radioLastUpdate) < 70000){
            if(controls.auxA < 10.0 && controls.throttle < 5.0 && controls.auxB > 90.0 && auxBSwPrev < 10.0){
              runState = 2;
            }

            if(auxASwPrev < 10.0 && controls.auxA > 90.0 && controls.auxB < 10.0){
              runState = 1;
            }
            
          }
          auxASwPrev = controls.auxA;
          auxBSwPrev = controls.auxB;
          break;
        case 1:  // tuning

          if(controls.auxA < 10.0){
            runState = 0;
            break;
          }
          
          if (controls.yaw < 40.0){
            if(controls.throttle > 90.0 && controls.pitch > 90.0 && controls.roll < 10.0){  // calibrate MPU
               mpu.calibrate();
            }
          }
          else if(controls.yaw > 60.0) {
            if(controls.throttle < 10.0 && controls.pitch < 10.0 && controls.roll > 90.0){  // reset all stored PID values to 0
               controller.resetPidK(controls);
            }
          }
          else{
            
            // tune:
            //  pitch up - increase a little bit
            //  pitch down - decrease a little bit
            //  roll right (up) - increase more
            //  roll left (down) - decrease more
            controller.tunePidK(controls);
          }

          break;
        case 2:  // running
          if(controls.auxB < 90.0 || (timeNow - radioLastUpdate) > 70000){
            runState = 0;
            auxASwPrev = controls.auxA;
            auxBSwPrev = controls.auxB;
            break;
          }
          
          controller.run(controls, angles, angles_filtered);
          
          break;
        default:
          controller.zeroThrusters();
          controller.resetPidError();
          runState = 0;
          Serial.println("PANIC ;P");
          break;
      }
    }
  }
}
