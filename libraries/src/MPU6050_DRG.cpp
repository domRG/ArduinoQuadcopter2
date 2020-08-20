#include "MPU6050_DRG.h"
#include "CONSTANTS_DRG.h"

static void add(angleData_t & newData, angleData_t & fillMe);

//Low pass butterworth filter order=2 alpha1=0.12   -----   http://www.schwietering.com/jayduino/filtuino/index.php?characteristic=bu&passmode=lp&order=2&usesr=usesr&sr=250&frequencyLow=30&noteLow=&noteHigh=&pw=pw&calctype=long&bitres=16&run=Send
FilterBuLp2::FilterBuLp2() {
	for (int i = 0; i <= 2; i++)
		v[i] = 0;
}

short FilterBuLp2::step(short x) {
	v[0] = v[1];
	v[1] = v[2];
	long tmp = ((((x * 23938L) >> 4) //= (   9.1314900436e-2 * x)
		+
		((v[0] * -22785L) >> 2) //+( -0.3476653949*v[0])
		+
		((v[1] * 32191L) >> 1) //+(  0.9824057931*v[1])
		) + 8192) >> 14; // round and downshift fixed point /16384

	v[2] = (short) tmp;
	return (short)((
		(v[0] + v[2]) +
		2 * v[1])); // 2^
}

//Low pass chebyshev filter order=2 alpha1=0.05 
FilterChLp2::FilterChLp2() {
	v[0] = 0.0;
	v[1] = 0.0;
}

float FilterChLp2::step(float x) //class II 
{
	v[0] = v[1];
	v[1] = v[2];
	v[2] = (5.698533564421289638e-2 * x) +
		(-0.48488625972370702488 * v[0]) +
		(1.25694491714685541162 * v[1]);
	return (v[0] + v[2]) + 2 * v[1];
}

Mpu6050::Mpu6050() {
	filterDp = FilterChLp2();
	filterDr = FilterChLp2();
	filterDy = FilterChLp2();
//    angles.p = 0;
//    angles.r = 0;
//    angles.y = 0;
}

void Mpu6050::setup(bool cal)
{
	Wire.begin();

	Wire.beginTransmission(MPU_addr);

	Wire.write(0x6B); // address of accel config register?
	Wire.write(0b00000000); // {RESET, SLEEP, CYCLE, -, TEMP_DISABLE, CLK_SELECT[3]}

	Wire.endTransmission(true); // false? then remove next beginTransmission(MPU_addr)???

	// Configuration of gyroscope
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x1B); // address of gyro config register
	Wire.write(0b00001000); // gyro config - 500d/s - scale factor = 65.5
	Wire.endTransmission(true);

	// Configuration of the accelerometer
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x1C); // address of accel config register
	Wire.write(0b00010000); // accel config - 8g - scale factor = 4096
	Wire.endTransmission(true);
	
	readBaselineFromEeprom();
	
	if (cal) {
		calibrate();
	}
}

void Mpu6050::calibrate() {
	uint32_t timePrev = 0;
	for (int samples = 0; samples < mpuCalDelay; ) {
		uint32_t timeDelta = micros() - timePrev;
		if (timeDelta >= timeStep) {
			timePrev += timeDelta;
			waitForNewAngles();
			if (((samples++) & 0xFF) == 0) {
				Serial.print(".");
			}
		}
	}
	Serial.println();
	Serial.print("Recording average offset:");
	
	for (int samples = 0; samples < mpuCalSamples; ) {
		uint32_t timeDelta = micros() - timePrev;
		if (timeDelta >= timeStep) {
			timePrev += timeDelta;
			waitForNewAngles();
			if (((samples++) & 0xFF) == 0) {
				Serial.print(".");
			}
		}
	}

	Serial.println();
	Serial.print("Recording complete");
	
	add(angles_filtered, angles_baseline);
	writeBaselineToEeprom();
}

void Mpu6050::readBaselineFromEeprom()
{
	const size_t len = sizeof(float) * 3;
	uint8_t* toRead = (uint8_t*)(&angles_baseline);
	for (unsigned int i = 0; i < len; i++) {
		toRead[i] = EEPROM.read(i);
	}
}

void Mpu6050::writeBaselineToEeprom() const
{
	const size_t len = sizeof(float) * 3;
	uint8_t* toWrite = (uint8_t*)(&angles_baseline);
	for (unsigned int i = 0; i < len; i++) {
		EEPROM.write(i, toWrite[i]);
	}
}

void Mpu6050::update()
{
	//  uint32_t tstart = micros();
	Wire.beginTransmission(MPU_addr);
	Wire.write(ACC_addr);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr, 14, true);
	while (Wire.available() < 14)
	{
//		Serial.println("Waiting for MPU");
	}
    data.raw.ayH = Wire.read();
    data.raw.ayL = Wire.read();
    data.raw.axH = Wire.read();
    data.raw.axL = Wire.read();
    data.raw.azH = Wire.read();
    data.raw.azL = Wire.read();
    data.raw.tH = Wire.read();
    data.raw.tL = Wire.read();
    data.raw.gpH = Wire.read();
	data.raw.gpL = Wire.read();
	data.raw.grH = Wire.read();
	data.raw.grL = Wire.read();
	data.raw.gyH = Wire.read();
	data.raw.gyL = Wire.read();
	
//	Serial.printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n", data.merged.ay, data.merged.ax, data.merged.az, data.merged.t, data.merged.gP, data.merged.gR, data.merged.gY);
    // Wire.endTransmission(true);  // adds 150us (frees Wire for use by other code - not required)
}

void Mpu6050::waitForNewAngles()
{
	update();
	
	// update rate of rotations
	angles.dP = data.merged.gP / gyScale - angles_baseline.dP;
	angles.dR = -data.merged.gR / gyScale - angles_baseline.dR;
	angles.dY = data.merged.gY / gyScale - angles_baseline.dY;
    
	angles_filtered.dP = filterDp.step(angles.dP);
	angles_filtered.dR = filterDr.step(angles.dR);
	angles_filtered.dY = filterDy.step(angles.dY);
 
    
    angles.aP = atan(-1*data.merged.ax/sqrt(pow(data.merged.ay,2) + pow(data.merged.az,2)))*RAD2DEG;
    angles.aR = atan(data.merged.ay/sqrt(pow(data.merged.ax,2) + pow(data.merged.az,2)))*RAD2DEG;
    
    Serial.printf("%f\t%f\n", angles.aP, angles.aR);
}

static void add(angleData_t & newData, angleData_t & fillMe)
{
	fillMe.dP += newData.dP;
	fillMe.dR += newData.dR;
	fillMe.dY += newData.dY;
}
