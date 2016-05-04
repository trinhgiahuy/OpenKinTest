#ifndef linux_virtual_h
#define linux_virtual_h

#include "virtual_main.h"
#include <math.h>
#include "binary.h"

#ifdef __cplusplus
extern "C"{
#endif

#define true 0x1
#define false 0x0

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifdef abs
#undef abs
#endif
#define abs(x) ((x) > 0 ? (x) : -(x))

#define constrain(amt, low, high) ((amt) < (low) ? \
				(low) : ((amt) > (high) ? (high) : (amt)))
#define round(x) ((x) >= 0 ? (long)((x)+0.5) : (long)((x)-0.5))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? \
				bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif
