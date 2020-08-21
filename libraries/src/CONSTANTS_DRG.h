#ifndef __EEPROM_DRG_H
#define __EEPROM_DRG_H

#define RATE 1000  // rate to define speed of entire controller, MPU takes 834us so max rate is 1190.47619 (timeStep = 840us)
const uint32_t timeStep = (uint32_t)(1000000 / (double)RATE);

#define EEPROM_GAINS 114
#define EEPROM_RATE_GAINS 114
#define EEPROM_ANGLE_GAINS 126
#define DO_NOT_STORE_GAINS -1

#define RAD2DEG 57.29577951308232087679815481410517
#define DEG2RAD 0.017453292519943295769236907684886

#define gyAcMix 0.98

#endif //__EEPROM_DRG_H
