#ifndef __RADIO_DRG_H__
#define __RADIO_DRG_H__

#include <Arduino.h>

typedef struct RadioData {
  float roll;
  float throttle;
  float pitch;
  float yaw;
  float auxA;
  float auxB;
} radioData_t;

class Radio {
	private:
		uint64_t startTime[6] = {0, 0, 0, 0, 0, 0};
		float prevDelta[6] = {1500, 1500, 1500, 1500, 1500, 1500};
		float deltaBalance = 0.85;
		float deltaMax[6] = {1750, 1750, 1750, 1750, 1750, 1750};
		float deltaMin[6] = {1250, 1250, 1250, 1250, 1250, 1250};
		uint16_t limitsSet = 0b0000111100000000;  // bit flags for when limits have been set (channel moved to an
        // extreme value to set edge limits, ie throttleMax) - ch5Max ch5Min ch4Max etc ch0Min - bitset '1' for ignore
		float deadZone = 10.0;
		uint8_t dropped[6] = {0, 0, 0, 0, 0, 0};
		float inputPercentages[6] = {0, 0, 0, 0, 0, 0};
		int ch[6] = {21, 20, 17, 16, 15, 14};
		uint8_t newFlag = 0;
		
		void interruptHandling(uint8_t channelId);

	public:
		Radio();
		void setup(void (*ch0Isr)(void), void (*ch1Isr)(void), void (*ch2Isr)(void), void (*ch3Isr)(void), void (*ch4Isr)(void), void (*ch5Isr)(void));
		bool run(radioData_t* outControls);
		void ch0Interrupt(void);
		void ch1Interrupt(void);
		void ch2Interrupt(void);
		void ch3Interrupt(void);
		void ch4Interrupt(void);
		void ch5Interrupt(void);
		uint8_t getNew();
		void resetNew();
};


#endif // __RADIO_DRG_H__