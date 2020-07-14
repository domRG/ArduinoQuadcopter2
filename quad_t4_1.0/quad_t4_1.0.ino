////// TODO
/// change how PID values can be set without pc


#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>

Servo fR;
Servo fL;
Servo bR;
Servo bL;

///// ===== Addressbook
//// 0 - 13 = reserved for uint16_t MPU cals  // gpL, gpH, grL, grH, gyL, gyH,( aL, aH, aL, aH, aL, aH, tL, tH)0
//// 14 - 37 = reserved for radio min/max  // ch0minL, ch0minH, ch1minL,... etc, ch0maxL, ch0maxH, ch1maxL,... etc

/// define location 'dictionary'
// pitch 0
// roll 1
// yaw 2
// throttle 3
// aux1 4
// aux2 5

///// =====

#define RATE 1000  // rate to define speed of entire controller, MPU takes 834us so max rate is 1190.47619 (timeStep = 840us)
uint32_t timeStep = (uint32_t)(1000000 / (double)RATE);

bool doMpuCal = false;
bool doThrusterCal = false;
bool radioLimitsSet = false;
bool armed = false;
uint32_t armedTime;

float motorLim[2] = {1100.0, 2000.0};
float throttleLim[2] = {1200.0, 1900.0};
float controlSensitivity = 75.0;  // 60 - 90 depending
float deadZoneSensitivity = 0.1;
// control limits
#define c_BOTTOM -1
#define c_LEFT -1
#define c_CENTRE 0
#define c_TOP 1
#define c_RIGHT 1

float pidInputs[4] = {1.4, 0.00e1, 0.0, 100.0};  // kP, kI, kD, windupLim  // 1.4, 0, 0

volatile static uint8_t radioNew = 0;

float gyScale = 65.5;

class Pid {
  private:
    float gains[3];
    float prevError;
    float iError;
    float iErrorLim;
  public:
    Pid(float kP = 0.0, float kI = 0.0, float kD = 0.0, float iELim = 250.0, float pE = 0.0, float iE = 0.0) {
      gains[0] = kP;
      gains[1] = kI;
      gains[2] = kD;
      prevError = pE;
      iError = iE;
      iErrorLim = iELim;
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

      return gains[0] * error + gains[1] * iError + gains[2] * dE;
    }

    float updateKp(float newGain) {
      gains[0] = newGain;
      return gains[0];
    }

    float updateKi(float newGain) {
      gains[1] = newGain;
      return gains[1];
    }

    float updateKd(float newGain) {
      gains[2] = newGain;
      return gains[2];
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

  mpuData_t mpuData;
  mpuData_t mpuOffsets;

  angleData_t angles;
  angleData_t setpointAngles;
  radioData_t controls;
  thrusterData_t tSpeeds;

  Pid pitchPid = Pid(pidInputs[0], pidInputs[1], pidInputs[2], pidInputs[3]);
  Pid rollPid = Pid(pidInputs[0], pidInputs[1], pidInputs[2], pidInputs[3]);
  Pid yawPid = Pid(pidInputs[0], pidInputs[1], pidInputs[2], pidInputs[3]);

  // 0======================================0
  // |    _____  ______ ______ __  __ ____  |
  // |   / ___/ / ____//_  __// / / // __ \ |
  // |   \__ \ / __/    / /  / / / // /_/ / |
  // |  ___/ // /___   / /  / /_/ // ____/  |
  // | /____//_____/  /_/   \____//_/       |
  // 0======================================0

  // Serial.begin(115200);

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
      if ((timeNow - radioLastUpdate) < 70000) {

        if (controls.auxB > 90.0 && radioLimitsSet) {
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
        else { // if not armed
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
