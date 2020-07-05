uint64_t startTime[6] = {0, 0, 0, 0, 0, 0};
float prevDelta[6] = {1500, 1500, 1500, 1500, 1500, 1500};
float deltaBalance = 0.85;
float deltaMax[6] = {1750, 1750, 1750, 1750, 1750, 1750};
float deltaMin[6] = {1250, 1250, 1250, 1250, 1250, 1250};
uint16_t limitsSet = 0;  // bit flags for when limits have been set (channel moved to an extreme value to set edge limits, ie throttleMax)
float deadZone = 10.0;
uint8_t dropped[6] = {0, 0, 0, 0, 0, 0};

float inputs[6] = {0, 0, 0, 0, 0, 0};

int ch[6] = {21, 20, 17, 16, 15, 14};

void setupRadio() {
  // put your setup code here, to run once:
  for(int i = 0; i < 6; i++){
    pinMode(ch[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(ch[0]), ch0Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
  attachInterrupt(digitalPinToInterrupt(ch[1]), ch1Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
  attachInterrupt(digitalPinToInterrupt(ch[2]), ch2Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
  attachInterrupt(digitalPinToInterrupt(ch[3]), ch3Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
  attachInterrupt(digitalPinToInterrupt(ch[4]), ch4Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
  attachInterrupt(digitalPinToInterrupt(ch[5]), ch5Interrupt, CHANGE);  // 14 = ch6, 15 = ch5, 16 = ch4, 17 = ch3, 20 = ch2, 21 = ch1
}

bool runRadio(radioData_t* outControls){
  outControls->roll = inputs[0];
  outControls->pitch = inputs[1];
  outControls->throttle = inputs[2];
  outControls->yaw = inputs[3];
  outControls->auxA = inputs[4];
  outControls->auxB = inputs[5];
  return (limitsSet == 0b0000111111111111) ? true : false;
}

void ch0Interrupt() {
  uint8_t i = 0;
  uint32_t noww = micros();
  float delta = (float)(noww - startTime[i]) * deltaBalance + (1 - deltaBalance) * prevDelta[i];
  if ((delta < 2050 && delta > 950)){
    if(delta < deltaMin[i]){
      deltaMin[i] = delta;
      limitsSet |= (1 << (2*i) );
    }
    else if(delta > deltaMax[i]){
      deltaMax[i] = delta;
      limitsSet |= (1 << (2*i + 1) );
    }
    inputs[i] = map(delta, deltaMin[i], deltaMax[i], 0.0, 100.0);  // delta;
    radioNew |= (1 << i);
    prevDelta[i] = delta;
    dropped[i] = 0;
  }
  else{
    dropped[i]++;
  }
  startTime[i] = noww;
}

void ch1Interrupt() {
  uint8_t i = 1;
  uint32_t noww = micros();
  float delta = (float)(noww - startTime[i]) * deltaBalance + (1 - deltaBalance) * prevDelta[i];
  if ((delta < 2050 && delta > 950)){
    if(delta < deltaMin[i]){
      deltaMin[i] = delta;
      limitsSet |= (1 << (2*i) );
    }
    else if(delta > deltaMax[i]){
      deltaMax[i] = delta;
      limitsSet |= (1 << (2*i + 1) );
    }
    inputs[i] = map(delta, deltaMin[i], deltaMax[i], 0.0, 100.0);  // delta;
    radioNew |= (1 << i);
    prevDelta[i] = delta;
    dropped[i] = 0;
  }
  else{
    dropped[i]++;
  }
  startTime[i] = noww;
}

void ch2Interrupt() {
  uint8_t i = 2;
  uint32_t noww = micros();
  float delta = (float)(noww - startTime[i]) * deltaBalance + (1 - deltaBalance) * prevDelta[i];
  if ((delta < 2050 && delta > 950)){
    if(delta < deltaMin[i]){
      deltaMin[i] = delta;
      limitsSet |= (1 << (2*i) );
    }
    else if(delta > deltaMax[i]){
      deltaMax[i] = delta;
      limitsSet |= (1 << (2*i + 1) );
    }
    inputs[i] = map(delta, deltaMin[i], deltaMax[i], 0.0, 100.0);  // delta;
    radioNew |= (1 << i);
    prevDelta[i] = delta;
    dropped[i] = 0;
  }
  else{
    dropped[i]++;
  }
  startTime[i] = noww;
}

void ch3Interrupt() {
  uint8_t i = 3;
  uint32_t noww = micros();
  float delta = (float)(noww - startTime[i]) * deltaBalance + (1 - deltaBalance) * prevDelta[i];
  if ((delta < 2050 && delta > 950)){
    if(delta < deltaMin[i]){
      deltaMin[i] = delta;
      limitsSet |= (1 << (2*i) );
    }
    else if(delta > deltaMax[i]){
      deltaMax[i] = delta;
      limitsSet |= (1 << (2*i + 1) );
    }
    inputs[i] = map(delta, deltaMin[i], deltaMax[i], 0.0, 100.0);  // delta;
    radioNew |= (1 << i);
    prevDelta[i] = delta;
    dropped[i] = 0;
  }
  else{
    dropped[i]++;
  }
  startTime[i] = noww;
}

void ch4Interrupt() {
  uint8_t i = 4;
  uint32_t noww = micros();
  float delta = (float)(noww - startTime[i]) * deltaBalance + (1 - deltaBalance) * prevDelta[i];
  if ((delta < 2050 && delta > 950)){
    if(delta < deltaMin[i]){
      deltaMin[i] = delta;
      limitsSet |= (1 << (2*i) );
    }
    else if(delta > deltaMax[i]){
      deltaMax[i] = delta;
      limitsSet |= (1 << (2*i + 1) );
    }
    inputs[i] = map(delta, deltaMin[i], deltaMax[i], 0.0, 100.0);  // delta;
    radioNew |= (1 << i);
    prevDelta[i] = delta;
    dropped[i] = 0;
  }
  else{
    dropped[i]++;
  }
  startTime[i] = noww;
}

void ch5Interrupt() {
  uint8_t i = 5;
  uint32_t noww = micros();
  float delta = (float)(noww - startTime[i]) * deltaBalance + (1 - deltaBalance) * prevDelta[i];
  if ((delta < 2050 && delta > 950)){
    if(delta < deltaMin[i]){
      deltaMin[i] = delta;
      limitsSet |= (1 << (2*i) );
    }
    else if(delta > deltaMax[i]){
      deltaMax[i] = delta;
      limitsSet |= (1 << (2*i + 1) );
    }
    inputs[i] = map(delta, deltaMin[i], deltaMax[i], 0.0, 100.0);  // delta;
    radioNew |= (1 << i);
    prevDelta[i] = delta;
    dropped[i] = 0;
  }
  else{
    dropped[i]++;
  }
  startTime[i] = noww;
}
