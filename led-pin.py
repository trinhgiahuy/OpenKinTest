#!/usr/bin/env python
try:
	import RPi.GPIO as GPIO
except RuntimeError:
	print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")
import os
import signal
import sys
import time

def led_on(signum, stack):
        GPIO.output(channel, GPIO.HIGH)

def led_off(signum, stack):
        GPIO.output(channel, GPIO.LOW)

def quit_gracefully(signum, stack):
        GPIO.cleanup(channel)
        sys.exit(0)

if __name__ == "__main__":

	if len(sys.argv) != 2:
		print("Not enough or too many parameters! ({})".format(len(sys.argv)))
		sys.exit(1)

	signal.signal(signal.SIGUSR1, led_on)
	signal.signal(signal.SIGUSR2, led_off)
	signal.signal(signal.SIGINT, quit_gracefully)

	# Pin no 18, GPIO5, BCM 24
	#channel=24
	channel = int(sys.argv[1])

	GPIO.setmode(GPIO.BCM)

	GPIO.setup(channel, GPIO.OUT)

	while True:
		time.sleep(2)

	GPIO.cleanup(channel)

