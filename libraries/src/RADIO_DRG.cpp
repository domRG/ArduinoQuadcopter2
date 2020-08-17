#include "RADIO_DRG.h"

Radio::Radio(){};

void Radio::setup(void (*ch0Isr)(void), void (*ch1Isr)(void), void (*ch2Isr)(void), void (*ch3Isr)(void), void (*ch4Isr)(void), void (*ch5Isr)(void))
{
	for (int i = 0; i < 6; i++) {
		pinMode(ch[i], INPUT);
	}
	attachInterrupt(digitalPinToInterrupt(ch[0]), ch0Isr, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
	attachInterrupt(digitalPinToInterrupt(ch[1]), ch1Isr, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
	attachInterrupt(digitalPinToInterrupt(ch[2]), ch2Isr, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
	attachInterrupt(digitalPinToInterrupt(ch[3]), ch3Isr, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
	attachInterrupt(digitalPinToInterrupt(ch[4]), ch4Isr, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
	attachInterrupt(digitalPinToInterrupt(ch[5]), ch5Isr, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
}
/*
void Radio::setup(){
	// put your setup code here, to run once:
	for (int i = 0; i < 6; i++) {
		pinMode(ch[i], INPUT);
	}
	attachInterrupt(digitalPinToInterrupt(ch[0]), (void(*)())&(this->ch0Interrupt), CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
	attachInterrupt(digitalPinToInterrupt(ch[1]), static_cast<void(*)()>(this->ch1Interrupt), CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
	attachInterrupt(digitalPinToInterrupt(ch[2]), (void(*)())std::bind(&Radio::ch2Interrupt,this), CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
	attachInterrupt(digitalPinToInterrupt(ch[3]), this::ch3Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
	attachInterrupt(digitalPinToInterrupt(ch[4]), ch4Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
	attachInterrupt(digitalPinToInterrupt(ch[5]), ch5Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
}
*/
bool Radio::run(radioData_t* outControls){
	outControls->roll = inputPercentages[0];
	outControls->pitch = inputPercentages[1];
	outControls->throttle = inputPercentages[2];
	outControls->yaw = inputPercentages[3];
	outControls->auxA = inputPercentages[4];
	outControls->auxB = inputPercentages[5];
	return (limitsSet == 0b0000111111111111) ? true : false;  // ignore
}

uint8_t Radio::getNew(){
	return newFlag;
}

void Radio::resetNew(){
	newFlag &= 0;
}

void Radio::interruptHandling(uint8_t channelId) {
	uint32_t noww = micros();
	float delta = (float)(noww - startTime[channelId]) * deltaBalance + (1 - deltaBalance) * prevDelta[channelId];  // calculate pulse width including small exponential filter to decrease noise
	if ((delta < 2050 && delta > 950)) { // within expected range
		if (delta < deltaMin[channelId]) { // update minimum recorded and flag limit set for channel min
			deltaMin[channelId] = delta;
			limitsSet |= (1 << (2 * channelId) );
		}
		else if (delta > deltaMax[channelId]) { // update maximum recorded and flag limit set for channel max
			deltaMax[channelId] = delta;
			limitsSet |= (1 << (2 * channelId + 1) );
		}
		inputPercentages[channelId] = map(delta, deltaMin[channelId], deltaMax[channelId], 0.0, 100.0);  // convert delta to percentage
		newFlag |= (1 << channelId);  // flag new signal
		prevDelta[channelId] = delta;
		dropped[channelId] = 0;
	}
	else {
		dropped[channelId]++;  // outside range = dropped frame, record for lost signal handling
	}
	startTime[channelId] = noww;
}

/*
void *Radio::fPointerGet(void(*fun)()){
	return &
}
*/

void Radio::ch0Interrupt(void) {
	interruptHandling(0);
}

void Radio::ch1Interrupt(void) {
	interruptHandling(1);
}

void Radio::ch2Interrupt(void) {
	interruptHandling(2);
}

void Radio::ch3Interrupt(void) {
	interruptHandling(3);
}

void Radio::ch4Interrupt(void) {
	interruptHandling(4);
}

void Radio::ch5Interrupt(void) {
	interruptHandling(5);
}