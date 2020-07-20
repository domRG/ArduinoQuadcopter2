uint64_t startTime[6] = {0, 0, 0, 0, 0, 0};
float prevDelta[6] = {1500, 1500, 1500, 1500, 1500, 1500};
float deltaBalance = 0.85;
float deltaMax[6] = {1750, 1750, 1750, 1750, 1750, 1750};
float deltaMin[6] = {1250, 1250, 1250, 1250, 1250, 1250};
uint16_t limitsSet = 0b0000100000000000;  // bit flags for when limits have been set (channel moved to an extreme value to set edge limits, ie throttleMax) - ch5Max ch5Min ch4Max etc ch0Min - preset ch5Max as this is 'arm' so will sit at min until switched on
float deadZone = 10.0;
uint8_t dropped[6] = {0, 0, 0, 0, 0, 0};

float inputPercentages[6] = {0, 0, 0, 0, 0, 0};

int ch[6] = {21, 20, 17, 16, 15, 14};

void setupRadio() {
  // put your setup code here, to run once:
  for (int i = 0; i < 6; i++) {
    pinMode(ch[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(ch[0]), ch0Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
  attachInterrupt(digitalPinToInterrupt(ch[1]), ch1Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
  attachInterrupt(digitalPinToInterrupt(ch[2]), ch2Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
  attachInterrupt(digitalPinToInterrupt(ch[3]), ch3Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
  attachInterrupt(digitalPinToInterrupt(ch[4]), ch4Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
  attachInterrupt(digitalPinToInterrupt(ch[5]), ch5Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
}

bool runRadio(radioData_t* outControls) { // sets control structure to contain radio data, returns true only if all control limits have been changed - ie sticks and channels moved though entire range to calibrate max and min
  outControls->roll = inputPercentages[0];
  outControls->pitch = inputPercentages[1];
  outControls->throttle = inputPercentages[2];
  outControls->yaw = inputPercentages[3];
  outControls->auxA = inputPercentages[4];
  outControls->auxB = inputPercentages[5];
  return (limitsSet == 0b0000111111111111) ? true : false;  // ignore
}

void interruptHandling(uint8_t channelId) {
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
    radioNew |= (1 << channelId);  // flag new signal
    prevDelta[channelId] = delta;
    dropped[channelId] = 0;
  }
  else {
    dropped[channelId]++;  // outside range = dropped frame, record for lost signal handling
  }
  startTime[channelId] = noww;
}

void ch0Interrupt() {
  interruptHandling(0);
}

void ch1Interrupt() {
  interruptHandling(1);
}

void ch2Interrupt() {
  interruptHandling(2);
}

void ch3Interrupt() {
  interruptHandling(3);
}

void ch4Interrupt() {
  interruptHandling(4);
}

void ch5Interrupt() {
  interruptHandling(5);
}
