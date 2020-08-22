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
bool doThrusterCal = false;

uint64_t radioLastUpdate = 0;  
radioData_t controls;
int runState = 0;
float auxASwPrev = 0;
float auxBSwPrev = 0;

uint64_t timeNow;

int runRunState();

static void updateRunState(int newState) {
  runState = newState;
  auxASwPrev = controls.auxA;
  auxBSwPrev = controls.auxB;
}

int main() {

  uint64_t timePrev = 0;

  controller.setup(doThrusterCal);  // must be done first - servos start at power-on!
  mpu.setup(doMpuCal);
  // ISR as lambdas for correct casting
  radio.setup( []{radio.ch0Interrupt();}, []{radio.ch1Interrupt();}, []{radio.ch2Interrupt();}, []{radio.ch3Interrupt();}, []{radio.ch4Interrupt();}, []{radio.ch5Interrupt();} );

  while (1) { // run loop

    // === time keeping
    timeNow = micros();
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

//      Serial.printf("%d\t%f\t%f\t%f\t%f\t0\t100\n",runState*40,controls.auxA,auxASwPrev,controls.auxB,auxBSwPrev);
      updateRunState(runRunState());
    }
  }
}

int runRunState(){
  switch(runState){
    case 0:  // off
      armed = false;
      controller.zeroThrusters();
      controller.resetPidError();
      return controls.auxA ? (controls.auxB ? 2 : 1) : (controls.auxB ? 3 : 0);
    case 1:  // tune angle
      if(controller.tuneAngle(controls)){
        mpu.calibrate();
      }
      return controls.auxA ? (controls.auxB ? 2 : 1) : (controls.auxB ? -1 : 0);
    case 2:  // tune rate
      if(controller.tuneRate(controls)){
        mpu.calibrate();
      }
      return controls.auxA ? (controls.auxB ? 2 : 1) : (controls.auxB ? -1 : 0);
    case 3:  // fly angle
      if (armed)
      {
        controller.runAngle(controls, mpu.getAngles(), mpu.getFilteredAngles());
      } else {
        armed = radio.isOkToArm();
      }
      return controls.auxA ? (controls.auxB ? 4 : -2) : (controls.auxB ? 3 : 0);
    case 4:  // fly rate
      if (armed)
      {
        controller.runRate(controls, mpu.getAngles(), mpu.getFilteredAngles());
      } else {
        armed = radio.isOkToArm();
      }
      return controls.auxA ? (controls.auxB ? 4 : -2) : (controls.auxB ? 3 : 0);
    case -1:  // lock
    case -2:  // lock
    case -3:  // lock
      armed = false;
      controller.zeroThrusters();
      controller.resetPidError();
      return controls.auxA ? (controls.auxB ? -3 : -2) : (controls.auxB ? -1 : 0);
    default:
      Serial.printf("PANIC X0 %d\n", runState);
      return 0;
  }
  return runState;
}
