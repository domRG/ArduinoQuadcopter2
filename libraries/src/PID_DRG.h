#ifndef __PID_DRG_H__
#define __PID_DRG_H__

#include <Arduino.h>
#include <EEPROM.h>


typedef struct PidGains {
  float p;
  float i;
  float d;
} pidGains_t;

class Pid {
  private:
    pidGains_t k;
    float prev;
    float iError;
    float iErrorLim;
    int eepromLocation;
  public:
    Pid(
            float kP = 0.0,
            float kI = 0.0,
            float kD = 0.0,
            float iELim = 250.0,
            int gainSaveStart = -1,
            float p = 0.0,
            float iE = 0.0
    );

    float step(
		float setPoint,
		float current,
		float current_filtered
		);

    float updateKp(float newGain, int incDir = 0);

    float updateKi(float newGain, int incDir = 0);

    float updateKd(float newGain, int incDir = 0);

    void reset();
};

#endif // __PID_DRG_H__