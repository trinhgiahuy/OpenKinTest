/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "wiring.h"

#include "HardwareSerial.h"

// Constructors ////////////////////////////////////////////////////////////////

HardwareSerial::HardwareSerial(void)
{
}

// Public Methods //////////////////////////////////////////////////////////////

void HardwareSerial::begin(long baud)
{
	printf("serial:setting baud rate to %ld\n",baud);
}

void HardwareSerial::end()
{
	printf("serial:end!\n");
}

int HardwareSerial::available(void)
{
printf("serial:available return 0x00\n");
return 0;
}

int HardwareSerial::peek(void)
{
printf("serial:peek return 0x00\n");
return 0;
}

int HardwareSerial::read(void)
{
printf("serial:read return value of 0x33\n");
return 0x33;
}

void HardwareSerial::flush()
{
printf("serial:calling flush()\n");
}

size_t HardwareSerial::write(uint8_t c)
{
printf("%c",c);
return 0;
}

// Preinstantiate Objects //////////////////////////////////////////////////////

  HardwareSerial Serial;


