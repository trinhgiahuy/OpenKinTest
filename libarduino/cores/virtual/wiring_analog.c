/*
 * wiring_analog.c - analog input and output
 * Part of Arduino - http://www.arduino.cc/
 *
 * Copyright (c) 2013 Parav Nagarsheth
 * Copyright (c) 2005-2006 David A. Mellis
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA  02111-1307  USA
 */

#include "wiring_analog.h"
#include "virtual_main.h"
#include "sysfs.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

static uint32_t _readResolution = 10;
static uint32_t _writeResolution = 8;

void analogReadResolution(uint32_t res)
{
	_readResolution = res;
}

void analogWriteResolution(uint32_t res)
{
	_writeResolution = res;
}

uint32_t analogRead(uint32_t pin)
{
	uint32_t value;
	if (g_APinDescription[pin].pinType == ANALOG) {
		value = sysfs_adc_getvalue(g_APinDescription[pin].analogChannel);
		/* Scale 12 bit ADC value as per _readResolution */
		if (_readResolution < 12) {
			value = (value << _readResolution) - value;
			value = value / 4095;
			return value;
		} else {
			value = value << (_readResolution - 12);
			return value;
		}
	} else {
		return 0;
	}
}

int analogWrite(uint8_t pin, uint32_t value)
{
	int pr;
	char prev[10];
	char buf[MAX_BUF];
	value = value * 20000;
	value = value / pow(2, _writeResolution);
	printf("pin: %d, value: %d\n", pin, value);

	switch (pin) {
	case 3:
		snprintf(buf, sizeof(buf), "/sys/class/pwm/pwm%d/", 1);
		break;
	case 5:
		snprintf(buf, sizeof(buf), "/sys/class/pwm/pwm%d/", 2);
		break;
	case 6:
		snprintf(buf, sizeof(buf), "/sys/class/pwm/pwm%d/", 0);
		break;
	default:
		/* FIXME: Handle other pins */
		break;
	}

	sysfs_read(buf, "run", prev);
	pr = atoi(prev);

	if (pr == 0) {
		sysfs_write(buf, "duty_ns", value);
		sysfs_write(buf, "period_ns", 20000);
		sysfs_write(buf, "run", 1);
	} else {
		sysfs_write(buf, "duty_ns", value);
	}

	if (!value)
		sysfs_write(buf, "run", 0);

	/* FIXME: Return 0 on success, negative on error, or similar */
	return pin;
}

#ifdef __cplusplus
}
#endif
