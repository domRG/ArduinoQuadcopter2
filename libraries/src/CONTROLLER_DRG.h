#ifndef __CONTROLLER_DRG_H
#define __CONTROLLER_DRG_H

#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>
#include "PID_DRG.h"
#include "RADIO_DRG.h"
#include "MPU6050_DRG.h"

#define P_LARGE_CHANGE 0.1
#define P_SMALL_CHANGE 0.01
#define I_LARGE_CHANGE 0.001
#define I_SMALL_CHANGE 0.0001
#define D_LARGE_CHANGE 0.1
#define D_SMALL_CHANGE 0.01

typedef struct ThrusterData {
    float fR = 1000.0;
    float fL = 1000.0;
    float bR = 1000.0;
    float bL = 1000.0;
} thrusterData_t;

// working PID gains[0]: 0.5, 0.0001, 0.01
// working PID gains[1]: 0.9, 0.0016, 0.03
// working PID gains[2]: 1.75, 0.0038, 0.32
// working PID gains[2]: 1.45, 0.0018, 0.32  // reasonable and stable - not too twitchy!!!


class Controller{
    private:
        Servo fR;
        Servo fL;
        Servo bR;
        Servo bL;
        
        float pidWindupLim = 100.0;
        float thrustLim[2] = {1000.0, 2000.0};
        float motorLim[2] = {1100.0, 2000.0};
        float throttleLim[2] = {1200.0, 1900.0};
        float controlSensitivity = 75.0;  // 60 - 90 depending
        float deadZoneSensitivity = 0.1;
        float storedGains[3];
        
        uint8_t printCounter1 = 0;
        uint64_t prevPidTuneTime = 0;
        int pidTuneState = 0;
        
        Pid pitchPid;
        Pid rollPid;
        Pid yawPid;
        
        thrusterData_t tSpeeds;
        
        void updateThrusters(thrusterData_t* speeds);
        float map(float x, float in_min, float in_max, float out_min, float out_max);
        float mapDeadzone(float x, float iL, float iH, float oL, float oH, float dzSf);
        float limit(float x, float lowerLim, float upperLim);
        float limitMotors(float x);
    
    public:
        void setup(bool thrusterCal);
        void run(radioData_t & controls, angleData_t & angles, angleData_t & angles_filtered);
        void resetPidK();
        void tunePidK(radioData_t & controls);
        void zeroThrusters();
        void resetPidError();
};



#endif //__CONTROLLER_DRG_H
