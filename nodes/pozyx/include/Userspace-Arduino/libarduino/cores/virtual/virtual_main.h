#ifndef virtual_main_h
#define virtual_main_h

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "sysfs.h"
#include "i2c.h"

#ifdef __cplusplus
extern "C"{
#endif

/* Definition and types for pins */
typedef enum _PinTypes {
	GPIO,
	PWM,
	_SPI,
	UART,
	I2C,
	ANALOG,
	LED
} PinTypes ;

/* Types used for the tables below */
typedef struct _PinDescription {
	uint32_t headerPin ;
	uint32_t gpioPin ;
	PinTypes pinType ;
	uint32_t analogChannel ;
	uint32_t pinOffset ;
} PinDescription ;

/* Pins table to be instanciated into variant.cpp */
extern PinDescription g_APinDescription[] ;

/* Program start time for millis and micros. Instantiated in main.cpp */
extern struct timespec prog_start_time;
#ifdef __cplusplus
}
#include "HardwareSerial.h"
#include "WMath.h"
#endif


#include "wiring.h"
#include "wiring_digital.h"
#include "wiring_analog.h"
#endif

