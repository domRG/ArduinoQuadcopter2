////// TODO
/// reset iError when changing Ki


#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>

Servo fR;
Servo fL;
Servo bR;
Servo bL;

///// ===== EEPROM Addressbook
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
bool armed = false;
uint32_t armedTime;

float motorLim[2] = {1100.0, 2000.0};
float throttleLim[2] = {1200.0, 1900.0};
float controlSensitivity = 75.0;  // 60 - 90 depending
float deadZoneSensitivity = 0.1;

float pidInputs[4] = {1.4, 0.001, 0.0, 100.0};  // kP, kI, kD, windupLim  // 1.4, 0, 0

volatile static uint8_t radioNew = 0;

float gyScale = 65.5;

class Pid{
  private:
    float gains[3];
    float prevError;
    float iError;
    float iErrorLim;
  public:
    Pid(float kP = 0.0, float kI = 0.0, float kD = 0.0, float iELim = 250.0, float pE = 0.0, float iE = 0.0){
      gains[0] = kP;
      gains[1] = kI;
      gains[2] = kD;
      prevError = pE;
      iError = iE;
      iErrorLim = iELim;
    }

    float step(float error){
      float dE = error - prevError;
      prevError = error;
      iError += error;
      if(iError > iErrorLim){
        iError = iErrorLim;
      }
      else if(iErrorLim < -iErrorLim){
        iError = -iErrorLim;
      }

      return gains[0] * error + gains[1] * iError + gains[2] * dE;
    }

    float updateKp(float newGain){
      gains[0] = newGain;
      return gains[0];
    }
    
    float updateKi(float newGain){
      gains[1] = newGain;
      return gains[1];
    }

    float updateKd(float newGain){
      gains[2] = newGain;
      return gains[2];
    }
};

typedef union MpuData{
  struct Raw{
    uint8_t pL;
    uint8_t pH;
    uint8_t rL;
    uint8_t rH;
    uint8_t yL;
    uint8_t yH;
  } raw;
  struct Merged{
    int16_t p;
    int16_t r;
    int16_t y;
  } merged;
} mpuData_t;

typedef struct AngleData{
  float dP;
  float dR;
  float dY;
} angleData_t;

typedef struct RadioData{
  float roll;
  float throttle;
  float pitch;
  float yaw;
  float auxA;
  float auxB;
} radioData_t;

typedef struct ThrusterData{
  float fR = 1000.0;
  float fL = 1000.0;
  float bR = 1000.0;
  float bL = 1000.0;
} thrusterData_t;

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapDeadzone(float x, float iL, float iH, float oL, float oH, float dzSf){
  
  float negMultiplier;
  float iMedian = (iL + iH) / 2.0;
  float oMedian = (oL + oH) / 2.0;
  float iDz = (iH - iL) * dzSf;
  
  x -= iMedian;
  if(x > 0){
    negMultiplier = 1.0;
  }
  else if(x < 0){
    negMultiplier = -1.0;
    x = x * negMultiplier;
  }
  else{
    return oMedian;
  }

  if(x > iDz / 2){
    return negMultiplier * map(x, iDz / 2, iH - iMedian, oMedian, oH);
  }
  else{
    return negMultiplier * oMedian;
  }
}

float limit(float x, float lowerLim, float upperLim){
  if(x < lowerLim){
    return lowerLim;
  }
  if(x > upperLim){
    return upperLim;
  }
  return x;
}

float limitMotors(float x){
  return limit(x, motorLim[0], motorLim[1]);
}

int Serialprint(thrusterData_t toPrint){
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
  // ===== Scope Variables =====
  
  uint64_t timePrev = 0;
  uint8_t printCounter = 0;

  mpuData_t mpuData;
  mpuData_t mpuOffsets;
  
  angleData_t angles;
  angleData_t setpointAngles;
  radioData_t controls;
  thrusterData_t tSpeeds;

  Pid pitchPid = Pid(pidInputs[0], pidInputs[1], pidInputs[2], pidInputs[3]);
  Pid rollPid = Pid(pidInputs[0], pidInputs[1], pidInputs[2], pidInputs[3]);
  Pid yawPid = Pid(pidInputs[0], pidInputs[1], pidInputs[2], pidInputs[3]);
  
  // ===== SETUP =====
  
  // Serial.begin(115200);

  setupRadio();
  setupMpu(&mpuOffsets);
  
  if(doMpuCal){
    calibrateMpu(&mpuOffsets);
  }
  
  setupThrusters(doThrusterCal);

  uint64_t radioLastUpdate = 0;

  // ===== LOOP =====
  while(1){  // run loop
    uint64_t timeNow = micros();
    uint32_t timeDelta = timeNow - timePrev;
    if(radioNew){
      radioNew &= 0;  // reset flag
      runRadio(&controls);
      // Serial.println((uint32_t)(timeNow - radioLastUpdate));
      radioLastUpdate = timeNow;
    }
    if(timeDelta >= timeStep){
      timePrev += timeDelta;
      updateMpuData(&mpuData, &mpuOffsets);
      angles.dP = mpuData.merged.p/gyScale;
      angles.dR = -mpuData.merged.r/gyScale;
      angles.dY = mpuData.merged.y/gyScale;
      // Serial.print(angles.dP); Serial.print("\t"); Serial.print(angles.dR); Serial.print("\t"); Serial.print(angles.dY); Serial.print("\t");
      // Serial.println();

      float tmp1 = pitchPid.updateKi(map(controls.auxA, 0.0, 100.0, 0.0, 0.01));
      float tmp2 = pitchPid.updateKd(map(controls.auxB, 0.0, 100.0, 0.0, 1.0));
      // controlSensitivity = map(controls.auxA, 0.0, 100.0, 45.0, 90.0);
      if(printCounter++ == 0){
        // Serial.print(controlSensitivity); Serial.print("\t");
        Serial.print(tmp1*1000); Serial.print(" /1000\t"); Serial.print(tmp2*1000); Serial.print(" /1000\t");
        // Serial.print(controls.throttle); Serial.print("\t"); Serial.print(controls.pitch); Serial.print("\t"); Serial.print(controls.roll); Serial.print("\t"); Serial.print(controls.yaw); Serial.print("\t"); 
        Serial.println();
      }

      if((timeNow - radioLastUpdate) < 50000){
        if(controls.throttle < 0.5 && controls.yaw < 0.5 && controls.roll < 0.5 && controls.pitch < 0.5 && millis() > armedTime + 1000){
          // Serial.print(controls.throttle); Serial.print("\t"); Serial.print(controls.yaw); Serial.print("\t"); Serial.print(controls.roll); Serial.print("\t"); Serial.print(controls.pitch); Serial.print("\t"); 
          // Serial.println();
          armed++;
          armedTime = millis();
        }
        
        if(armed){
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
          
          if(printCounter++ == 0){
            Serialprint(tSpeeds);
          }
          updateThrusters(&tSpeeds);
        }
        else{
          thrusterData_t zeroedSpeeds;
          updateThrusters(&zeroedSpeeds);  // turn thrusters off
          if(printCounter++ == 0){
            Serialprint(zeroedSpeeds);
            // Serial.print(armed); Serial.println("this");
          }
        }
      }
      else{
          thrusterData_t zeroedSpeeds;
          updateThrusters(&zeroedSpeeds);  // turn thrusters off
          if(printCounter++ == 0){
            Serialprint(zeroedSpeeds);
            // Serial.println("that");
          }
        }
    }
  }
}
