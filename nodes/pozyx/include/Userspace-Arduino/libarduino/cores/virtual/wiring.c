/*
  wiring.c - Partial implementation of the Wiring API for the ATmega8.
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

*/

#include <wiring.h>
#include <unistd.h>
#include <time.h>

struct timespec prog_start_time;

unsigned long millis()
{
	return micros()/1000;
}

unsigned long micros()
{
	struct timespec gettime_now;
	clock_gettime(CLOCK_MONOTONIC, &gettime_now);
	return ((gettime_now.tv_sec - prog_start_time.tv_sec)*1000000 +
			(gettime_now.tv_nsec - prog_start_time.tv_nsec)/1000);
}

void delay(unsigned long ms)
{
	usleep(ms*1000);
}

void delayMicroseconds(unsigned int us)
{
	/* usleep(us); */
	long int start_time;
	long int time_difference;
	struct timespec gettime_now;

	clock_gettime(CLOCK_MONOTONIC, &gettime_now);
	start_time = gettime_now.tv_nsec;		//Get nS value
	while (1)
	{
		clock_gettime(CLOCK_MONOTONIC, &gettime_now);
		time_difference = gettime_now.tv_nsec - start_time;
		if (time_difference < 0)
			time_difference += 1000000000;				//(Rolls over every 1 second)
		if (time_difference > (us * 1000))		//Delay for # nS
			break;
	}

}

