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

  // 0========================================================================================================0
  // |    _____  ______ ____   ____   ______   _    __ ___     ____   ____ ___     ____   __     ______ _____ |
  // |   / ___/ / ____// __ \ / __ \ / ____/  | |  / //   |   / __ \ /  _//   |   / __ ) / /    / ____// ___/ |
  // |   \__ \ / /    / / / // /_/ // __/     | | / // /| |  / /_/ / / / / /| |  / __  |/ /    / __/   \__ \  |
  // |  ___/ // /___ / /_/ // ____// /___     | |/ // ___ | / _, _/_/ / / ___ | / /_/ // /___ / /___  ___/ /  |
  // | /____/ \____/ \____//_/    /_____/     |___//_/  |_|/_/ |_|/___//_/  |_|/_____//_____//_____/ /____/   |
  // 0========================================================================================================0

  uint64_t timePrev = 0;
  uint64_t radioLastUpdate = 0;
  
  // angleData_t & angles;
  // angleData_t & angles_filtered;
  angleData_t setpointAngles;
  radioData_t controls;

  // 0======================================0
  // |    _____  ______ ______ __  __ ____  |
  // |   / ___/ / ____//_  __// / / // __ \ |
  // |   \__ \ / __/    / /  / / / // /_/ / |
  // |  ___/ // /___   / /  / /_/ // ____/  |
  // | /____//_____/  /_/   \____//_/       |
  // 0======================================0

  controller.setup(doThrusterCal);  // must be done first - servos start at power-on!
  
  mpu.setup(doMpuCal);

  // ISR as lambdas for correct casting
  radio.setup( []{radio.ch0Interrupt();}, []{radio.ch1Interrupt();}, []{radio.ch2Interrupt();}, []{radio.ch3Interrupt();}, []{radio.ch4Interrupt();}, []{radio.ch5Interrupt();} );

  //  0===============================0
  //  |     __    ____   ____   ____  |
  //  |    / /   / __ \ / __ \ / __ \ |
  //  |   / /   / / / // / / // /_/ / |
  //  |  / /___/ /_/ // /_/ // ____/  |
  //  | /_____/\____/ \____//_/       |
  //  0===============================0

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

      // === if radio signal in the last 70ms - pulse every 20ms so missed 3 pulses
      if ((timeNow - radioLastUpdate) < 70000 && radioLimitsSet) {

        if (controls.auxB > 90.0 && controls.auxA < 10) {  // arm switch on and programming switch off
          controller.run(controls, angles, angles_filtered);
        }
        else { // if arm switch not on or limits not set

          if (controls.auxA > 90) {  // enter programming mode
            
            if (controls.yaw < 40){
              if(controls.throttle > 90 && controls.pitch > 90 && controls.roll < 10){  // calibrate MPU
                mpu.calibrate();
              }
            }
            else if(controls.yaw > 60) {
              if(controls.throttle < 10 && controls.pitch < 10 && controls.roll > 90){  // reset all stored PID values to 0
                controller.resetPidK();
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
          }
          controller.zeroThrusters();
          controller.resetPidError();
        }
      }
      else { // if radio disconnected
        controller.zeroThrusters();
        controller.resetPidError();
      }
    }
  }
}
