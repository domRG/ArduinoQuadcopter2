#include "CONTROLLER_DRG.h"
#include "CONSTANTS_DRG.h"

void Controller::setup(bool doCal)
{
    EEPROM.get(EEPROM_P_GAIN, storedGains[0]);
    EEPROM.get(EEPROM_I_GAIN, storedGains[1]);
    EEPROM.get(EEPROM_D_GAIN, storedGains[2]);
    
    bool pitchSaveGainsToEeprom = true;
    bool rollSaveGainsToEeprom = !pitchSaveGainsToEeprom;
    pitchPid = Pid(storedGains[0], storedGains[1], storedGains[2], pidWindupLim, pitchSaveGainsToEeprom);
    rollPid = Pid(storedGains[0], storedGains[1], storedGains[2], pidWindupLim, rollSaveGainsToEeprom);
    yawPid = Pid(4, 0.03, 0.1, pidWindupLim, false);
    
    fR.attach(2);
    fL.attach(3);
    bR.attach(4);
    bL.attach(5);
    if(doCal){
        fR.writeMicroseconds((int)thrustLim[1]);
        fL.writeMicroseconds((int)thrustLim[1]);
        bR.writeMicroseconds((int)thrustLim[1]);
        bL.writeMicroseconds((int)thrustLim[1]);
        delay(2500);
    }
    fR.writeMicroseconds((int)thrustLim[0]);
    fL.writeMicroseconds((int)thrustLim[0]);
    bR.writeMicroseconds((int)thrustLim[0]);
    bL.writeMicroseconds((int)thrustLim[0]);
}

void Controller::updateThrusters(thrusterData_t *speeds)
{
    fR.writeMicroseconds((int)speeds->fR);
    fL.writeMicroseconds((int)speeds->fL);
    bR.writeMicroseconds((int)speeds->bR);
    bL.writeMicroseconds((int)speeds->bL);
}

float Controller::mapDeadzone(float x, float iL, float iH, float oL, float oH, float dzSf)
{
    
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

float Controller::limit(float x, float lowerLim, float upperLim)
{
    if (x < lowerLim) {
        return lowerLim;
    }
    if (x > upperLim) {
        return upperLim;
    }
    return x;
}

float Controller::limitMotors(float x)
{
    return limit(x, motorLim[0], motorLim[1]);
}

float Controller::map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Controller::run(radioData_t &controls, angleData_t &angles, angleData_t &angles_filtered)
{
    angleData_t setpointAngles;
    
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

void Controller::resetPidK()
{
    pitchPid.updateKp(0);
    rollPid.updateKp(0);
    pitchPid.updateKi(0);
    rollPid.updateKi(0);
    pitchPid.updateKd(0);
    rollPid.updateKd(0);
}

void Controller::tunePidK(radioData_t & controls)
{
    if (printCounter1++ == 0) {
        Serial.print("K{gP,i,d} = "); Serial.print(pitchPid.updateKp(0, 1), 6); Serial.print("\t"); Serial.print(pitchPid.updateKi(0, 1), 6); Serial.print("\t"); Serial.print(pitchPid.updateKd(0, 1), 6);
        Serial.println();
    }
    
    switch (pidTuneState) {
        case 0:
            
            if (controls.throttle > 66.6) {  // tune gP
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

void Controller::zeroThrusters()
{
    thrusterData_t zeroedSpeeds;
    updateThrusters(&zeroedSpeeds);  // turn thrusters off
}

void Controller::resetPidError()
{
    pitchPid.reset();
    rollPid.reset();
    yawPid.reset();
}
