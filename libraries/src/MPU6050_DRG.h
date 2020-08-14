#ifndef __MPU6050_DRG_H__
#define __MPU6050_DRG_H__

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#define MPU_addr 0x68  // default MPU6050 address
#define ACC_addr 0x3B
#define TEMP_addr 0x41
#define GYRO_addr 0x43
#define mpuCalDelay 3000  // samples to bin befor offset calibration
#define mpuCalSamples 3000  // samples to average for offset

typedef union MpuData {
  struct Raw {
    uint8_t pL;
    uint8_t pH;
    uint8_t rL;
    uint8_t rH;
    uint8_t yL;
    uint8_t yH;
  } raw;
  struct Merged {
    int16_t p;
    int16_t r;
    int16_t y;
  } merged;
} mpuData_t;

typedef struct AngleData {
  float dP;
  float dR;
  float dY;
} angleData_t;

class MPU6050
{
private:
	
public:
	
};

class FilterBuLp2
{
	private:
		short v[3];
	public:
		FilterBuLp2();
		short step(short x);
};

class FilterChLp2
{
	private:
		float v[3];
	public:
		FilterChLp2();
		float step(float x);
};

void setupMpu(void* offsets);

void calibrateMpu(mpuData_t* offsets, uint32_t timeStep);

void updateMpuData(mpuData_t* data, mpuData_t* data_filtered, mpuData_t* offsets);

#endif // __MPU6050_DRG_H__