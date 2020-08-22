#ifndef __MPU6050_DRG_H__
#define __MPU6050_DRG_H__

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#define MPU_addr 0x68 // default MPU6050 address
#define ACC_addr 0x3B
#define TEMP_addr 0x41
#define GYRO_addr 0x43
#define mpuCalDelay 3000 // samples to bin befor offset calibration
#define mpuCalSamples 3000 // samples to average for offset

typedef union MpuData {
	struct Raw {
        uint8_t ayL;
        uint8_t ayH;
        uint8_t axL;
        uint8_t axH;
        uint8_t azL;
        uint8_t azH;
        uint8_t tL;
        uint8_t tH;
        uint8_t gpL;
		uint8_t gpH;
		uint8_t grL;
		uint8_t grH;
		uint8_t gyL;
		uint8_t gyH;
	}
	raw;
	struct Merged {
        int16_t ay;
        int16_t ax;
        int16_t az;
        int16_t t;
        int16_t gP;
		int16_t gR;
		int16_t gY;
	}
	merged;
}
mpuData_t;

typedef struct AngleData {
	float dP = 0;
	float dR = 0;
	float dY = 0;
    float aY = 0;
    float aX = 0;
    float aZ = 0;
    float aP = 0;
    float aR = 0;
    float p = 0;
    float r = 0;
    float y = 0;
} angleData_t;

class FilterBuLp2 {
	private:
		short v[3];
	public:
		FilterBuLp2();
		short step(short x);
};

class FilterChLp2 {
	private:
		float v[3];
	public:
		FilterChLp2();
		float step(float x);
};

class Mpu6050 {
	private:
		FilterChLp2 filterDp;
		FilterChLp2 filterDr;
        FilterChLp2 filterDy;
        FilterChLp2 filterAx;
        FilterChLp2 filterAy;
        FilterChLp2 filterAz;
		mpuData_t data;
	
		angleData_t angles;
		angleData_t angles_filtered;
		angleData_t angles_baseline;
	
		uint32_t timeStep;
        uint32_t counter;
	
		float gyScale = 65.5;
	
		void readBaselineFromEeprom();
		void writeBaselineToEeprom() const;
	
	public:
		Mpu6050();
  
		void setup(bool cal);
		void calibrate();
        void update();
	
		void waitForNewAngles();
	
		inline angleData_t & getAngles() {
			return angles;
		}
		inline angleData_t & getFilteredAngles() {
			return angles_filtered;
		}
        
        inline uint32_t getCounter(){
            return counter;
        }
	
	
};

#endif // __MPU6050_DRG_H__