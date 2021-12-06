#if (ARDUINO >= 100)
	#include "Arduino.h" // for Arduino 1.0
#else
	#include "WProgram.h" // for Arduino 23
#endif

// object indexes for Inputs, Image Addresses + Input Array
#define iSmartGauge1H     0x0000
#define iSmartGauge1L     0x0000
#define iSmartGauge2H     0x0075
#define iSmartGauge2L     0x8200
#define iSmartGauge3H     0x0206
#define iSmartGauge3L     0x0400
#define iImage1H          0x0396
#define iImage1L          0x8600
