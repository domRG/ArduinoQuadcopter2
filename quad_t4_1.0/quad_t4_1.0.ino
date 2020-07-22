////// TODO
/// use auxA to set flyable / pid tuning setting
/// option to recalibrate the gyro mid session
/// store PID values in EEPROM
///time delay on settings in Program mode - timer for enter, 0 on exit



#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>

Servo fR;
Servo fL;
Servo bR;
Servo bL;

///// ===== Addressbook
//// 0 - 13 = reserved for uint16_t MPU cals  // gpL, gpH, grL, grH, gyL, gyH,( aL, aH, aL, aH, aL, aH, tL, tH)0
//// 14 - 25 = reserved for float (4 bytes) pid values
#define EEPROM_P_GAIN 14
#define EEPROM_I_GAIN 18
#define EEPROM_D_GAIN 22

/// define location 'dictionary'
// pitch 0
// roll 1
// yaw 2
// throttle 3
// aux1 4
// aux2 5

#define P_LARGE_CHANGE 0.1
#define P_SMALL_CHANGE 0.01
#define I_LARGE_CHANGE 0.0001
#define I_SMALL_CHANGE 0.00001
#define D_LARGE_CHANGE 0.1
#define D_SMALL_CHANGE 0.01

///// =====

bool radioLimitsSet = false;
bool armed = false;

// Adjustable definitions

#define RATE 1000  // rate to define speed of entire controller, MPU takes 834us so max rate is 1190.47619 (timeStep = 840us)
uint32_t timeStep = (uint32_t)(1000000 / (double)RATE);
bool doMpuCal = false;
bool doThrusterCal = false;
float motorLim[2] = {1100.0, 2000.0};
float throttleLim[2] = {1200.0, 1900.0};
float controlSensitivity = 75.0;  // 60 - 90 depending
float deadZoneSensitivity = 0.1;

// float pidInputs[4] = {1.4, 0.001, 0.4, 100.0};  // kP, kI, kD, windupLim  // 1.4, 0, 0
#define PID_I_LIM 100.0

volatile static uint8_t radioNew = 0;

float gyScale = 65.5;

typedef struct PidGains {
  float p;
  float i;
  float d;
} pidGains_t;

class Pid {
  private:
    pidGains_t k;
    float prevError;
    float iError;
    float iErrorLim;
    bool saveToEeprom;
  public:
    Pid(float kP = 0.0, float kI = 0.0, float kD = 0.0, float iELim = 250.0, bool saveGains = false, float pE = 0.0, float iE = 0.0) {
      k.p = kP;
      k.i = kI;
      k.d = kD;
      prevError = pE;
      iError = iE;
      iErrorLim = iELim;
      saveToEeprom = saveGains;
    }

    float step(float error) {
      float dE = error - prevError;
      prevError = error;
      iError += error;
      if (iError > iErrorLim) {
        iError = iErrorLim;
      }
      else if (iErrorLim < -iErrorLim) {
        iError = -iErrorLim;
      }

      return k.p * error + k.i * iError + k.d * dE;
    }

    float updateKp(float newGain, int incDir = 0) {
      if (incDir > 0) {
        k.p += newGain;
      }
      else if (incDir < 0) {
        k.p -= newGain;
      }
      else { // incDir == 0
        k.p = newGain;
      }

      if (k.p < 0.0) {
        k.p = 0.0;
      }
      
      if(saveToEeprom){
        EEPROM.put(EEPROM_P_GAIN, k.p);
      }
      
      return k.p;
    }

    float updateKi(float newGain, int incDir = 0) {
      if (incDir > 0) {
        k.i += newGain;
      }
      else if (incDir < 0) {
        k.i -= newGain;
      }
      else { // incDir == 0
        k.i = newGain;
      }

      if (k.i < 0.0) {
        k.i = 0.0;
      }
      
      if(saveToEeprom){
        EEPROM.put(EEPROM_I_GAIN, k.i);
      }
      
      return k.i;
    }

    float updateKd(float newGain, int incDir = 0) {
      if (incDir > 0) {
        k.d += newGain;
      }
      else if (incDir < 0) {
        k.d -= newGain;
      }
      else { // incDir == 0
        k.d = newGain;
      }

      if (k.d < 0.0) {
        k.d = 0.0;
      }
      
      if(saveToEeprom){
        EEPROM.put(EEPROM_D_GAIN, k.d);
      }
      
      return k.d;
    }

    void reset() {
      iError = 0;
      prevError = 0;
    }
};

typedef union MpuData {
  struct Raw {
    uint8_t pL;
    uint8_t pH;
    uint8_t rL;
    uint8_t rH;
    uint8_t yL;
    uint8_t yH;
  } raw;
  struct Merged {
    int16_t p;
    int16_t r;
    int16_t y;
  } merged;
} mpuData_t;

typedef struct AngleData {
  float dP;
  float dR;
  float dY;
} angleData_t;

typedef struct RadioData {
  float roll;
  float throttle;
  float pitch;
  float yaw;
  float auxA;
  float auxB;
} radioData_t;

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
  uint8_t printCounter2 = 0;
  uint64_t prevPidTuneTime = 0;
  int pidTuneState = 0;

  mpuData_t mpuData;
  mpuData_t mpuOffsets;

  angleData_t angles;
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
  Pid yawPid = Pid(3, 0.02, 0, PID_I_LIM, false);

  // 0======================================0
  // |    _____  ______ ______ __  __ ____  |
  // |   / ___/ / ____//_  __// / / // __ \ |
  // |   \__ \ / __/    / /  / / / // /_/ / |
  // |  ___/ // /___   / /  / /_/ // ____/  |
  // | /____//_____/  /_/   \____//_/       |
  // 0======================================0

  setupRadio();
  setupMpu(&mpuOffsets);

  if (doMpuCal) {
    calibrateMpu(&mpuOffsets);
  }

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
    if (radioNew) {
      radioNew &= 0;  // reset flag
      radioLimitsSet = runRadio(&controls);
      radioLastUpdate = timeNow;
    }


    // === run once every timeStep
    if (timeDelta >= timeStep) {
      timePrev += timeDelta;

      // poll MPU for new data
      updateMpuData(&mpuData, &mpuOffsets);

      // update rate of rotations
      angles.dP = mpuData.merged.p / gyScale;
      angles.dR = -mpuData.merged.r / gyScale;
      angles.dY = mpuData.merged.y / gyScale;


      // === if radio signal in the last 70ms - pulse every 20ms so missed 3 pulses
      if ((timeNow - radioLastUpdate) < 70000 && radioLimitsSet) {

        if (controls.auxB > 90.0 && controls.auxA < 10) {  // arm switch on and programming switch off
          float baseThrust = map(controls.throttle, 0.0, 100.0, throttleLim[0], throttleLim[1]);
          setpointAngles.dP = mapDeadzone(controls.pitch, 0.0, 100.0, controlSensitivity, -controlSensitivity, deadZoneSensitivity);  // inverted
          setpointAngles.dR = mapDeadzone(controls.roll, 0.0, 100.0, -controlSensitivity, controlSensitivity, deadZoneSensitivity);
          setpointAngles.dY = mapDeadzone(controls.yaw, 0.0, 100.0, -controlSensitivity, controlSensitivity, deadZoneSensitivity);

          float pitchAdjust = pitchPid.step(setpointAngles.dP - angles.dP);
          float rollAdjust = rollPid.step(setpointAngles.dR - angles.dR);
          float yawAdjust = yawPid.step(setpointAngles.dY - angles.dY);

          tSpeeds.fR = limitMotors(baseThrust + pitchAdjust - rollAdjust + yawAdjust);
          tSpeeds.fL = limitMotors(baseThrust + pitchAdjust + rollAdjust - yawAdjust);
          tSpeeds.bR = limitMotors(baseThrust - pitchAdjust - rollAdjust - yawAdjust);
          tSpeeds.bL = limitMotors(baseThrust - pitchAdjust + rollAdjust + yawAdjust);

          if (printCounter2++ == 0) {
            // Serialprint(tSpeeds);
          }
          updateThrusters(&tSpeeds);
        }
        else { // if arm switch not on or limits not set

          if (controls.auxA > 90) {  // enter programming mode
            
            if (controls.yaw < 40){
              if(controls.throttle > 90 && controls.pitch > 90 && controls.roll < 10){  // calibrate MPU
                calibrateMpu(&mpuOffsets);
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
