#!/usr/bin/env python

try:
	import RPi.GPIO as GPIO
except RuntimeError:
	print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")
import os

# Pin no 12, GPIO1, BCM 18
channel=18

GPIO.setmode(GPIO.BCM)

# Pull up so grounding triggers
GPIO.setup(channel, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Wait for switch to be flipped
GPIO.wait_for_edge(channel, GPIO.FALLING)

os.system("rm /home/pi/measurement.lock")
os.system("sudo shutdown -h now")

GPIO.cleanup(channel)
