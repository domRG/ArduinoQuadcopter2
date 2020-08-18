////// TODO
/// Arming procedure
/// Custom libraries
/// Angle controller

#define RATE 1000  // rate to define speed of entire controller, MPU takes 834us so max rate is 1190.47619 (timeStep = 840us)
uint32_t timeStep = (uint32_t)(1000000 / (double)RATE);

bool radioLimitsSet = false;

#include <MPU6050_DRG.h>
#include <RADIO_DRG.h>
#include <PID_DRG.h>
#include <EEPROM.h>
#include <Servo.h>

Mpu6050 mpu;
Radio radio;
Servo fR;
Servo fL;
Servo bR;
Servo bL;

///// ===== EEPROM Addressbook
//// 0 - 13 = reserved for uint16_t MPU cals  // gpL, gpH, grL, grH, gyL, gyH,( aL, aH, aL, aH, aL, aH, tL, tH)
//// 14 - 25 = reserved for float (4 bytes) pid values

/// define location 'dictionary'
// pitch 0
// roll 1
// yaw 2
// throttle 3
// aux1 4
// aux2 5


#define P_LARGE_CHANGE 0.1
#define P_SMALL_CHANGE 0.01
#define I_LARGE_CHANGE 0.001
#define I_SMALL_CHANGE 0.0001
#define D_LARGE_CHANGE 0.1
#define D_SMALL_CHANGE 0.01

// working PID gains[0]: 0.5, 0.0001, 0.01
// working PID gains[1]: 0.9, 0.0016, 0.03
// working PID gains[2]: 1.75, 0.0038, 0.32

///// =====

bool armed = false;

// Adjustable definitions

bool doMpuCal = false;
bool doThrusterCal = true;
float motorLim[2] = {1100.0, 2000.0};
float throttleLim[2] = {1200.0, 1900.0};
float controlSensitivity = 75.0;  // 60 - 90 depending
float deadZoneSensitivity = 0.1;

// float pidInputs[4] = {1.4, 0.001, 0.4, 100.0};  // kP, kI, kD, windupLim  // 1.4, 0, 0
#define PID_I_LIM 100.0


typedef struct ThrusterData {
  float fR = 1000.0;
  float fL = 1000.0;
  float bR = 1000.0;
  float bL = 1000.0;
} thrusterData_t;

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapDeadzone(float x, float iL, float iH, float oL, float oH, float dzSf) {

  float negMultiplier;
  float iMedian = (iL + iH) / 2.0;
  float oMedian = (oL + oH) / 2.0;
  float iDz = (iH - iL) * dzSf;

  x -= iMedian;
  if (x > 0) {
    negMultiplier = 1.0;
  }
  else if (x < 0) {
    negMultiplier = -1.0;
    x = x * negMultiplier;
  }
  else {
    return oMedian;
  }

  if (x > iDz / 2) {
    return negMultiplier * map(x, iDz / 2, iH - iMedian, oMedian, oH);
  }
  else {
    return negMultiplier * oMedian;
  }
}

float limit(float x, float lowerLim, float upperLim) {
  if (x < lowerLim) {
    return lowerLim;
  }
  if (x > upperLim) {
    return upperLim;
  }
  return x;
}

float limitMotors(float x) {
  return limit(x, motorLim[0], motorLim[1]);
}

int Serialprint(thrusterData_t toPrint) {
  int toReturn = 0;
  toReturn += Serial.print(toPrint.fR);
  toReturn += Serial.print("\t");
  toReturn += Serial.print(toPrint.fL);
  toReturn += Serial.print("\t");
  toReturn += Serial.print(toPrint.bR);
  toReturn += Serial.print("\t");
  toReturn += Serial.print(toPrint.bL);
  toReturn += Serial.println();
  return toReturn;
}



int main() {

  // 0========================================================================================================0
  // |    _____  ______ ____   ____   ______   _    __ ___     ____   ____ ___     ____   __     ______ _____ |
  // |   / ___/ / ____// __ \ / __ \ / ____/  | |  / //   |   / __ \ /  _//   |   / __ ) / /    / ____// ___/ |
  // |   \__ \ / /    / / / // /_/ // __/     | | / // /| |  / /_/ / / / / /| |  / __  |/ /    / __/   \__ \  |
  // |  ___/ // /___ / /_/ // ____// /___     | |/ // ___ | / _, _/_/ / / ___ | / /_/ // /___ / /___  ___/ /  |
  // | /____/ \____/ \____//_/    /_____/     |___//_/  |_|/_/ |_|/___//_/  |_|/_____//_____//_____/ /____/   |
  // 0========================================================================================================0

  uint64_t timePrev = 0;
  uint8_t printCounter1 = 0;
  uint64_t prevPidTuneTime = 0;
  int pidTuneState = 0;

  // angleData_t & angles;
  // angleData_t & angles_filtered;
  angleData_t setpointAngles;
  radioData_t controls;
  thrusterData_t tSpeeds;

  float storedGains[3];
  EEPROM.get(EEPROM_P_GAIN, storedGains[0]);
  EEPROM.get(EEPROM_I_GAIN, storedGains[1]);
  EEPROM.get(EEPROM_D_GAIN, storedGains[2]);

  /*
   * while(!Serial){}
   * Serial.print(storedGains[0], 6); Serial.print("\t");
   * Serial.print(storedGains[1], 6); Serial.print("\t");
   * Serial.print(storedGains[2], 6); Serial.print("\t");
   * Serial.println();
   */
  
  bool pitchSaveGainsToEeprom = true;
  bool rollSaveGainsToEeprom = !pitchSaveGainsToEeprom;
  Pid pitchPid = Pid(storedGains[0], storedGains[1], storedGains[2], PID_I_LIM, pitchSaveGainsToEeprom);
  Pid rollPid = Pid(storedGains[0], storedGains[1], storedGains[2], PID_I_LIM, rollSaveGainsToEeprom);
  Pid yawPid = Pid(4, 0.03, 0.1, PID_I_LIM, false);

  // 0======================================0
  // |    _____  ______ ______ __  __ ____  |
  // |   / ___/ / ____//_  __// / / // __ \ |
  // |   \__ \ / __/    / /  / / / // /_/ / |
  // |  ___/ // /___   / /  / /_/ // ____/  |
  // | /____//_____/  /_/   \____//_/       |
  // 0======================================0

  radio.setup(
          []{radio.ch0Interrupt();},
          []{radio.ch1Interrupt();},
          []{radio.ch2Interrupt();},
          []{radio.ch3Interrupt();},
          []{radio.ch4Interrupt();},
          []{radio.ch5Interrupt();}
          );
  
  mpu.setup(doMpuCal, timeStep);

  setupThrusters(doThrusterCal);

  uint64_t radioLastUpdate = 0;

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
      angleData_t & angles = mpu.getAngles();
      angleData_t & angles_filtered = mpu.getFilteredAngles();

      // === if radio signal in the last 70ms - pulse every 20ms so missed 3 pulses
      if ((timeNow - radioLastUpdate) < 70000 && radioLimitsSet) {

        if (controls.auxB > 90.0 && controls.auxA < 10) {  // arm switch on and programming switch off
          float baseThrust = map(controls.throttle, 0.0, 100.0, throttleLim[0], throttleLim[1]);
          setpointAngles.dP = mapDeadzone(controls.pitch, 0.0, 100.0, controlSensitivity, -controlSensitivity, deadZoneSensitivity);  // inverted
          setpointAngles.dR = mapDeadzone(controls.roll, 0.0, 100.0, -controlSensitivity, controlSensitivity, deadZoneSensitivity);
          setpointAngles.dY = mapDeadzone(controls.yaw, 0.0, 100.0, -controlSensitivity, controlSensitivity, deadZoneSensitivity);

          float pitchAdjust = pitchPid.step(setpointAngles.dP, angles.dP, angles_filtered.dP);
          float rollAdjust = rollPid.step(setpointAngles.dR, angles.dR, angles_filtered.dP);
          float yawAdjust = yawPid.step(setpointAngles.dY, angles.dY, angles_filtered.dP);

          tSpeeds.fR = limitMotors(baseThrust + pitchAdjust - rollAdjust - yawAdjust);
          tSpeeds.fL = limitMotors(baseThrust + pitchAdjust + rollAdjust + yawAdjust);
          tSpeeds.bR = limitMotors(baseThrust - pitchAdjust - rollAdjust + yawAdjust);
          tSpeeds.bL = limitMotors(baseThrust - pitchAdjust + rollAdjust - yawAdjust);

          updateThrusters(&tSpeeds);
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
                pitchPid.updateKp(0);
                rollPid.updateKp(0);
                pitchPid.updateKi(0);
                rollPid.updateKi(0);
                pitchPid.updateKd(0);
                rollPid.updateKd(0);
              }
            }
            else{
              
              // tune:
              //  pitch up - increase a little bit
              //  pitch down - decrease a little bit
              //  roll right (up) - increase more
              //  roll left (down) - decrease more

              if (printCounter1++ == 0) {
                Serial.print("K{p,i,d} = "); Serial.print(pitchPid.updateKp(0, 1), 6); Serial.print("\t"); Serial.print(pitchPid.updateKi(0, 1), 6); Serial.print("\t"); Serial.print(pitchPid.updateKd(0, 1), 6);
                Serial.println();
              }

              switch (pidTuneState) {
                case 0:

                  if (controls.throttle > 66.6) {  // tune p
                    pidTuneState = 1;
                  }
                  else if (controls.throttle > 33.3) {  // tune i
                    pidTuneState = 2;
                  }
                  else {
                    pidTuneState = 3;
                  }

                  break;

                case 1:

                  if (controls.pitch > 75.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKp(P_SMALL_CHANGE, 1);
                    rollPid.updateKp(P_SMALL_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else if (controls.pitch < 25.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKp(P_SMALL_CHANGE, -1);
                    rollPid.updateKp(P_SMALL_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else if (controls.roll > 75.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKp(P_LARGE_CHANGE, 1);
                    rollPid.updateKp(P_LARGE_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else if (controls.roll < 25.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKp(P_LARGE_CHANGE, -1);
                    rollPid.updateKp(P_LARGE_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else {
                    pidTuneState = 0;
                  }

                  break;

                case 2:

                  if (controls.pitch > 75.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKi(I_SMALL_CHANGE, 1);
                    rollPid.updateKi(I_SMALL_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else if (controls.pitch < 25.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKi(I_SMALL_CHANGE, -1);
                    rollPid.updateKi(I_SMALL_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else if (controls.roll > 75.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKi(I_LARGE_CHANGE, 1);
                    rollPid.updateKi(I_LARGE_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else if (controls.roll < 25.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKi(I_LARGE_CHANGE, -1);
                    rollPid.updateKi(I_LARGE_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else {
                    pidTuneState = 0;
                  }

                  break;

                case 3:

                  if (controls.pitch > 75.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKd(D_SMALL_CHANGE, 1);
                    rollPid.updateKd(D_SMALL_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else if (controls.pitch < 25.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKd(D_SMALL_CHANGE, -1);
                    rollPid.updateKd(D_SMALL_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else if (controls.roll > 75.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKd(D_LARGE_CHANGE, 1);
                    rollPid.updateKd(D_LARGE_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else if (controls.roll < 25.0 && (millis() - prevPidTuneTime) > 50) {
                    pitchPid.updateKd(D_LARGE_CHANGE, -1);
                    rollPid.updateKd(D_LARGE_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                  }
                  else {
                    pidTuneState = 0;
                  }

                  break;

                case -1:  // waiting for pitch/roll to centre

                  if (controls.pitch > 40 && controls.pitch < 60 && controls.roll > 40 && controls.roll < 60) {
                    pidTuneState = 0;
                  }

                  break;
              }
            }
          }


          thrusterData_t zeroedSpeeds;
          updateThrusters(&zeroedSpeeds);  // turn thrusters off
          pitchPid.reset();  // clear PIDs memory
          rollPid.reset();
          yawPid.reset();
        }
      }
      else { // if radio disconnected
        thrusterData_t zeroedSpeeds;
        updateThrusters(&zeroedSpeeds);  // turn thrusters off
        pitchPid.reset();  // clear PIDs memory
        rollPid.reset();
        yawPid.reset();
      }
    }
  }
}
