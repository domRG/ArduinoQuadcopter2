#include "CONTROLLER_DRG.h"
#include "CONSTANTS_DRG.h"

void Controller::setup(bool doCal)
{
    for(int i = 0; i < 6; i++){
        EEPROM.get(EEPROM_GAINS + i * sizeof(float), storedGains[i]);
    }
    
    pRpid = Pid(storedGains[0], storedGains[1], storedGains[2], pidWindupLim, EEPROM_GAINS);
    rRpid = Pid(storedGains[0], storedGains[1], storedGains[2], pidWindupLim, DO_NOT_STORE_GAINS);
    pApid = Pid(storedGains[3], storedGains[4], storedGains[5], pidWindupLim, EEPROM_ANGLE_GAINS);
    rApid = Pid(storedGains[3], storedGains[4], storedGains[5], pidWindupLim, DO_NOT_STORE_GAINS);
    yRpid = Pid(4, 0.03, 0.1, pidWindupLim, false);
    
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

void Controller::updateThrusters(thrusterData_t & speeds)
{
//    static uint8_t counter = 0;
//    if(counter++ == 0){
//        Serial.printf("%f\t%f\t%f\t%f\n", speeds.fR, speeds.fL, speeds.bR, speeds.bL);
//    }
    fR.writeMicroseconds((int)speeds.fR);
    fL.writeMicroseconds((int)speeds.fL);
    bR.writeMicroseconds((int)speeds.bR);
    bL.writeMicroseconds((int)speeds.bL);
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

void Controller::runRate(radioData_t &controls, angleData_t &angles, angleData_t &angles_filtered){
    angleData_t setpointAngles;
    setpointAngles.dP = mapDeadzone(controls.pitch, 0.0, 100.0, rateSensitivity, -rateSensitivity, deadZoneSensitivity);  // inverted
    setpointAngles.dR = mapDeadzone(controls.roll, 0.0, 100.0, -rateSensitivity, rateSensitivity, deadZoneSensitivity);
    
    run(controls, angles, angles_filtered, setpointAngles);
}

void Controller::runAngle(radioData_t &controls, angleData_t &angles, angleData_t &angles_filtered){
    angleData_t setpointAngles;
    setpointAngles.p = mapDeadzone(controls.pitch, 0.0, 100.0, angleSensitivity, -angleSensitivity,
                                   deadZoneSensitivity);  // inverted
    setpointAngles.r = mapDeadzone(controls.roll, 0.0, 100.0, -angleSensitivity, angleSensitivity,
                                   deadZoneSensitivity);
    
    setpointAngles.dP = pApid.step(setpointAngles.p, angles.p, angles.p);
    setpointAngles.dR = rApid.step(setpointAngles.r, angles.r, angles.r);
    
    run(controls, angles, angles_filtered, setpointAngles);
}

void Controller::run(radioData_t &controls, angleData_t &angles, angleData_t &angles_filtered, angleData_t & setpointAngles)
{
    
    float baseThrust = map(controls.throttle, 0.0, 100.0, throttleLim[0], throttleLim[1]);
    setpointAngles.dY = mapDeadzone(controls.yaw, 0.0, 100.0, -rateSensitivity, rateSensitivity, deadZoneSensitivity);
    
    float pitchAdjust = pRpid.step(setpointAngles.dP, angles.dP, angles_filtered.dP);
    float rollAdjust = rRpid.step(setpointAngles.dR, angles.dR, angles_filtered.dP);
    float yawAdjust = yRpid.step(setpointAngles.dY, angles.dY, angles_filtered.dP);
    
    tSpeeds.fR = limitMotors(baseThrust + pitchAdjust - rollAdjust - yawAdjust);
    tSpeeds.fL = limitMotors(baseThrust + pitchAdjust + rollAdjust + yawAdjust);
    tSpeeds.bR = limitMotors(baseThrust - pitchAdjust - rollAdjust + yawAdjust);
    tSpeeds.bL = limitMotors(baseThrust - pitchAdjust + rollAdjust - yawAdjust);
    
    updateThrusters(tSpeeds);
}

bool Controller::tuneRate(radioData_t & controls)
{
    return tunePidK(controls, &pRpid, &rRpid);
}

bool Controller::tuneAngle(radioData_t & controls)
{
    return tunePidK(controls, &pApid, &rApid);
}

bool Controller::tunePidK(radioData_t & controls, Pid * toTuneP, Pid * toTuneR)
{
    if (printCounter1++ == 0) {
        Serial.printf("rK{p,i,d}; aK{p,i,d} = %.4f, %.4f, %.4f; %.4f, %.4f, %.4f\n", pRpid.updateKp(0, 1), pRpid
                .updateKi(0, 1), pRpid.updateKd(0, 1), pApid.updateKp(0, 1), pApid
                .updateKi(0, 1), pApid.updateKd(0, 1));
    }
    if(controls.yaw < 40.0) {
        return (controls.throttle > 80.0 && controls.pitch > 80.0 && controls.roll < 20.0);
    }
    else if(controls.yaw > 60.0) {
        if(controls.throttle < 20.0 && controls.pitch < 20.0 && controls.roll > 80.0){
            toTuneP->updateKp(0);
            toTuneR->updateKp(0);
            toTuneP->updateKi(0);
            toTuneR->updateKi(0);
            toTuneP->updateKd(0);
            toTuneR->updateKd(0);
        }
    }
    else {
        switch (pidTuneState) {
            case 0:
            
                if (controls.throttle > 66.6)
                {  // tune gP
                    pidTuneState = 1;
                }
                else if (controls.throttle > 33.3)
                {  // tune i
                    pidTuneState = 2;
                }
                else
                {
                    pidTuneState = 3;
                }
            
                break;
        
            case 1:
            
                if (controls.pitch > 75.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKp(P_SMALL_CHANGE, 1);
                    toTuneR->updateKp(P_SMALL_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else if (controls.pitch < 25.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKp(P_SMALL_CHANGE, -1);
                    toTuneR->updateKp(P_SMALL_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else if (controls.roll > 75.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKp(P_LARGE_CHANGE, 1);
                    toTuneR->updateKp(P_LARGE_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else if (controls.roll < 25.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKp(P_LARGE_CHANGE, -1);
                    toTuneR->updateKp(P_LARGE_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else
                {
                    pidTuneState = 0;
                }
            
                break;
        
            case 2:
            
                if (controls.pitch > 75.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKi(I_SMALL_CHANGE, 1);
                    toTuneR->updateKi(I_SMALL_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else if (controls.pitch < 25.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKi(I_SMALL_CHANGE, -1);
                    toTuneR->updateKi(I_SMALL_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else if (controls.roll > 75.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKi(I_LARGE_CHANGE, 1);
                    toTuneR->updateKi(I_LARGE_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else if (controls.roll < 25.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKi(I_LARGE_CHANGE, -1);
                    toTuneR->updateKi(I_LARGE_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else
                {
                    pidTuneState = 0;
                }
            
                break;
        
            case 3:
            
                if (controls.pitch > 75.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKd(D_SMALL_CHANGE, 1);
                    toTuneR->updateKd(D_SMALL_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else if (controls.pitch < 25.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKd(D_SMALL_CHANGE, -1);
                    toTuneR->updateKd(D_SMALL_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else if (controls.roll > 75.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKd(D_LARGE_CHANGE, 1);
                    toTuneR->updateKd(D_LARGE_CHANGE, 1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else if (controls.roll < 25.0 && (millis() - prevPidTuneTime) > 50)
                {
                    toTuneP->updateKd(D_LARGE_CHANGE, -1);
                    toTuneR->updateKd(D_LARGE_CHANGE, -1);
                    prevPidTuneTime = millis();
                    pidTuneState = -1;
                }
                else
                {
                    pidTuneState = 0;
                }
            
                break;
        
            case -1:  // waiting for pitch/roll to centre
            
                if (controls.pitch > 40 && controls.pitch < 60 && controls.roll > 40 && controls.roll < 60)
                {
                    pidTuneState = 0;
                }
            
                break;
        }
    }
    return false;
}

void Controller::zeroThrusters()
{
    thrusterData_t zeroedSpeeds;
    updateThrusters(zeroedSpeeds);  // turn thrusters off
}

void Controller::resetPidError()
{
    pRpid.reset();
    rRpid.reset();
    yRpid.reset();
    pApid.reset();
    rApid.reset();
}
