#!/bin/sh
#TODO might be a good idea to start the nodes at the beginning and only  start/stop the bag recording
screen -r grive -X quit
screen -dmS grive
while true; do 
	nc -w 3 -z 8.8.8.8 53  >/dev/null 2>&1
	online=$?
	if [ $online -eq 0 ]; then
		echo "Online, shutting down the nodes"
		screen -S gps -X quit
		screen -S imu -X quit
		
		#The bag needs to exit gracefully
		screen -S bag -X stuff $'\003'
		sleep 2
		screen -S bag -X quit
		
		#Uploading the data
		echo "Waiting for everything to shut down"
		sleep 1
		echo "Uploading the data"
		screen -r grive -X stuff $'\ngrive\n'
	else		
		#NOTE! The gps is a launch file which will take care of starting the ros core
		#If this is not used, separate core process must be launched
		if ! screen -list | grep -q "gps"; then
			echo "Offline: Starting GPS and ROSCORE"
			screen -dmS gps
			screen -r gps -X stuff $'\nroslaunch ublox_gps ublox_gps.launch\n'
		fi
		
		if ! screen -list | grep -q "imu"; then
			echo "Offline: Starting IMU"
			screen -dmS imu
			screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py _device:=/dev/serial/by-id/usb-Xsens_Xsens_COM_port_00340764-if00\n'
		fi
		
		if ! screen -list | grep -q "bag"; then
			echo "Offline: Starting RECORDING"
			screen -dmS bag
			screen -r bag -X stuff $'\nrosbag record -a\n'
		fi
	fi
	
	#Sleeping a while
	sleep 2
done
