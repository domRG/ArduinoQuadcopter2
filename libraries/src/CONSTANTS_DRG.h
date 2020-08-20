#ifndef __EEPROM_DRG_H
#define __EEPROM_DRG_H

#define RATE 1000  // rate to define speed of entire controller, MPU takes 834us so max rate is 1190.47619 (timeStep = 840us)
const uint32_t timeStep = (uint32_t)(1000000 / (double)RATE);

#define EEPROM_P_GAIN 14
#define EEPROM_I_GAIN 18
#define EEPROM_D_GAIN 22

#define RAD2DEG 57.29577951308232087679815481410517
#define DEG2RAD 0.017453292519943295769236907684886

#endif //__EEPROM_DRG_H
