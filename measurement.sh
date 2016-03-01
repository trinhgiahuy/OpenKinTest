#!/bin/bash
#TODO might be a good idea to start the nodes at the beginning and only  start/stop the bag recording

# Get Xsens device
XSENS=""
for f in /dev/serial/by-id/usb-Xsens_Xsens_COM_port*; do

    ## Check if the glob gets expanded to existing files.
    ## If not, f here will be exactly the pattern above
    ## and the exists test will evaluate to false.
    #[ -e "$f" ] && echo "files do exist" || echo "files do not exist"

    XSENS=$f

    ## This is all we needed to know, so we can break after the first iteration
    break
done

echo "Trying to quit grive-screen"
screen -r grive -X quit
echo "Starting up grive-screen"
screen -dmS grive
while true; do
	nc -w 3 -z 8.8.8.8 53  >/dev/null 2>&1
#	if [ -f "online" ]; then
#		online=0
#	else
#		online=1
#	fi
	online=$?
	if [ $online -eq 0 ]; then
		echo "Online, shutting down the nodes"
		screen -S gps -X quit
		screen -S imu -X quit

		echo "Shutting down bag"
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
			screen -r gps -X stuff $'\nrm ../gpstemp.log\nroslaunch ublox_gps ublox_gps.launch 2> ../gpstemp.log\n'

			# should have time to error, if going to
	                sleep 15

	                # if file exists and not empty
	                if [ -s ../gpstemp.log ]; then
	                        screen -S gps -X quit
	                        # start over
	                        continue
	                fi

		fi

		if ! screen -list | grep -q "imu"; then
			echo "Offline: Starting IMU"
			screen -dmS imu
			# /dev/serial/by-id/usb-Xsens_Xsens_COM_port_00342762-if00
			#screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py _device:=/dev/serial/by-id/usb-Xsens_Xsens_COM_port_00340764-if00\n'
			screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py _device:='$XSENS$'\n'
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
