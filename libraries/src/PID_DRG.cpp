#include "PID_DRG.h"
#include "CONSTANTS_DRG.h"

Pid::Pid(
		float kP /*= 0.0*/,
		float kI /*= 0.0*/,
		float kD /*= 0.0*/,
		float iELim /*= 250.0*/,
		int gainSaveStart /*= -1*/,
		float p /*= 0.0*/,
		float iE /*= 0.0*/
)
{
	k.p = kP;
	k.i = kI;
	k.d = kD;
	prev = p;
	iError = iE;
	iErrorLim = iELim;
	eepromLocation = gainSaveStart;
}

float Pid::step(float setPoint, float current, float current_filtered)
{
	float dE = current_filtered - prev;
	prev = current_filtered;
	float error = setPoint - current;
	iError += error;
	if (iError > iErrorLim) {
		iError = iErrorLim;
	}
	else if (iError < -iErrorLim) {
		iError = -iErrorLim;
	}

	return k.p * error + k.i * iError + k.d * dE;
}

float Pid::updateKp(float newGain, int incDir /*= 0*/)
{
	float old = k.p;
    
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

	if(old != k.p && eepromLocation >= 0){
		EEPROM.put(eepromLocation + 0 * sizeof(float), k.p);
	}

	return k.p;
}

float Pid::updateKi(float newGain,int incDir /*= 0*/)
{
    float old = k.i;
    
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

	if(old != k.i && eepromLocation >= 0){
		EEPROM.put(eepromLocation + 1 * sizeof(float), k.i);
	}

	return k.i;
}

float Pid::updateKd(float newGain, int incDir /*= 0*/)
{
    float old = k.d;
    
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

	if(old != k.d && eepromLocation >= 0){
		EEPROM.put(eepromLocation + 2 * sizeof(float), k.d);
	}

	return k.d;
}

void Pid::reset() {
	iError = 0;
	// prev = 0;  // not required?
}