#define MPU_addr 0x68  // default MPU6050 address
#define ACC_addr 0x3B
#define TEMP_addr 0x41
#define GYRO_addr 0x43
#define mpuCalDelay 3000  // samples to bin befor offset calibration
#define mpuCalSamples 3000  // samples to average for offset

//Low pass butterworth filter order=2 alpha1=0.12   -----   http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=bu&passmode=lp&order=2&usesr=usesr&sr=250&frequencyLow=30&noteLow=&noteHigh=&pw=pw&calctype=long&bitres=16&run=Send
class FilterBuLp2
{
  public:
    FilterBuLp2()
    {
      for(int i=0; i <= 2; i++)
        v[i]=0;
    }
  private:
    short v[3];
  public:
    short step(short x)
    {
      v[0] = v[1];
      v[1] = v[2];
      long tmp = ((((x *  23938L) >>  4)  //= (   9.1314900436e-2 * x)
        + ((v[0] * -22785L) >> 2) //+( -0.3476653949*v[0])
        + ((v[1] * 32191L) >> 1)  //+(  0.9824057931*v[1])
        )+8192) >> 14; // round and downshift fixed point /16384

      v[2]= (short)tmp;
      return (short)((
         (v[0] + v[2])
        +2 * v[1])); // 2^
    }
};

//Low pass chebyshev filter order=2 alpha1=0.05 
class  FilterChLp2
{
  public:
    FilterChLp2()
    {
      v[0]=0.0;
      v[1]=0.0;
    }
  private:
    float v[3];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (5.698533564421289638e-2 * x)
         + (-0.48488625972370702488 * v[0])
         + (1.25694491714685541162 * v[1]);
      return 
         (v[0] + v[2])
        +2 * v[1];
    }
};

FilterChLp2 mpuFilterDp = FilterChLp2();
FilterChLp2 mpuFilterDr = FilterChLp2();
FilterChLp2 mpuFilterDy = FilterChLp2();

void setupMpu(void* offsets){
  Wire.begin();

  Wire.beginTransmission(MPU_addr);

  Wire.write(0x6B);  // address of accel config register?
  Wire.write(0b00000000);  // {RESET, SLEEP, CYCLE, -, TEMP_DISABLE, CLK_SELECT[3]}

  Wire.endTransmission(true);  // false? then remove next beginTransmission(MPU_addr)???

  // Configuration of gyroscope
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // address of gyro config register
  Wire.write(0b00001000);  // gyro config - 500d/s - scale factor = 65.5
  Wire.endTransmission(true);

  // Configuration of the accelerometer
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // address of accel config register
  Wire.write(0b00010000);  // accel config - 8g - scale factor = 4096
  Wire.endTransmission(true);
  
  for(unsigned int i = 0; i < sizeof(mpuData_t); i++){
    *((uint8_t*)offsets + i) = EEPROM.read(i);
  }
}

void calibrateMpu(mpuData_t* offsets){
  int valsToCal = sizeof(mpuData_t)/sizeof(int16_t);
  // Serial.println(valsToCal);
  int32_t* sum = (int32_t*)calloc(valsToCal, sizeof(int32_t));
  mpuData_t *test = (mpuData_t*)calloc(1, sizeof(mpuData_t));
  uint32_t samples = 0;
  uint64_t timePrev = 0;
  uint8_t printCounter = 0;
  Serial.print("Calibrating. Discarding startup:");
  while(samples < mpuCalDelay){
    uint32_t timeDelta = micros() - timePrev;
    if(timeDelta >= timeStep){
      timePrev += timeDelta;
      updateMpuData(test, offsets);
      samples++;
      if(printCounter++ == 0){
        Serial.print(".");
      }
    }
  }
  Serial.println();
  Serial.print("Recording average offset:");
  samples = 0;
  while(samples < mpuCalSamples){
    uint32_t timeDelta = micros() - timePrev;
    if(timeDelta >= timeStep){
      timePrev += timeDelta;
      updateMpuData(test, offsets);
      for(uint8_t i = 0; i < valsToCal; i++){
        *(sum + i) += *(((int16_t*)test) + i);
      }
      samples++;
      if(printCounter++ == 0){
        Serial.print(".");
      }
    }
  }
  Serial.println();
  Serial.println("Recording complete");
  //Serial.print(*(int16_t*)offsets); Serial.print("\t"); Serial.print(*sum); Serial.print("\t"); Serial.print((int16_t)(*sum/(float)samples)); Serial.print("\t"); Serial.print((int16_t)(*sum/(float)samples)-*(int16_t*)offsets);
  //Serial.println();
  for(int i = 0; i < valsToCal; i++){
    Serial.print(*((int16_t*)offsets + i)); Serial.print("\t");
    *((int16_t*)offsets + i) = (int16_t)((*(sum + i)/(float)samples) + *((int16_t*)offsets + i));
    Serial.print(*((int16_t*)offsets + i)); Serial.print("\t");
  }
  Serial.println();
  for(unsigned int i = 0; i < (valsToCal * 2); i++){
    EEPROM.write(i, *((uint8_t*)(void*)offsets + i));
  }
}

void updateMpuData(mpuData_t* data, mpuData_t* offsets){
//  uint32_t tstart = micros();
  Wire.beginTransmission(MPU_addr);
  Wire.write(GYRO_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  while(Wire.available() < 6){
    Serial.println("Waiting for MPU");
  }
  data->raw.pH = Wire.read();
  data->raw.pL = Wire.read();
  data->merged.p -= offsets->merged.p;
  data->raw.rH = Wire.read();
  data->raw.rL = Wire.read();
  data->merged.r -= offsets->merged.r;
  data->raw.yH = Wire.read();
  data->raw.yL = Wire.read();
  data->merged.y -= offsets->merged.y;
  
  data_filtered->merged.p = mpuFilterDp.step(data->merged.p);
  data_filtered->merged.r = mpuFilterDr.step(data->merged.r);
  data_filtered->merged.y = mpuFilterDy.step(data->merged.y);
  
//  Wire.endTransmission(true);  // adds 150us (frees Wire for use by other code - not required)
//  Serial.print(data->merged.p); Serial.print("\t"); Serial.print(data->merged.r); Serial.print("\t"); Serial.print(data->merged.y); Serial.print("\t"); Serial.print((uint32_t)micros() - tstart);
//  Serial.println();
}
