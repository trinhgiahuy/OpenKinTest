/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _WIRING_DIGITAL_
#define _WIRING_DIGITAL_
#include <inttypes.h>
#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Configures the specified pin to behave either as an input or an output. See the description of digital pins for details.
 *
 * \param ulPin The number of the pin whose mode you wish to set
 * \param ulMode Either INPUT or OUTPUT
 */
extern void pinMode(uint8_t, uint8_t);

extern void digitalWrite(uint8_t, uint8_t);

/**
 * \brief Reads the value from a specified digital pin, either HIGH or LOW.
 *
 * \param ulPin The number of the digital pin you want to read (int)
 *
 * \return HIGH or LOW
 */
extern int digitalRead(uint8_t);

#ifdef __cplusplus
}
#endif

#endif /* _WIRING_DIGITAL_ */
