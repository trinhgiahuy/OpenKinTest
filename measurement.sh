#!/bin/bash
#TODO might be a good idea to start the nodes at the beginning and only start/stop the bag recording

# Needed for rostopic
#source /opt/ros/indigo/setup.bash
source /opt/ros/kinetic/setup.bash
#source ~/catkin_ws/devel/setup.bash

GPSTEMPFILE="/home/pi/gpstemp.log"
LOGFILE="/home/pi/measurement.log"
DATADIR="/home/pi/data"
ROSTOPICFILE="/home/pi/rostmp.log"
# empty logfile
#> $LOGFILE

# primary led connected (linux-gpio) = 0, not = 1, pwm = 2, rpigpio = 3
LED[0]=3
# GPIO pin or pwm device number
LEDGPIO[0]=24
LEDPID[0]=0

# secondary led connected = 0, not = 1, pwm = 2, rpigpio = 3
LED[1]=1
LEDGPIO[1]=0
LEDPID[1]=0

LED[2]=1
LEDGPIO[2]=0
LEDPID[2]=0

# linux-shutdown switch connected = 0, not = 1, rpi python = 2
PWRSWITCH=2

#TODO: options for gps, imu, pozyx

GRIVEPID=-1

STOPPED=0
UPLOADED=1
STARTEDUPLOAD=1

IMUERR=1
GPSERR=1
POZYXERR=1

TIMECORRECTED=1

online=1

# Functions

function logger {
	echo "[$(date)] $1" >> $LOGFILE
	echo "[$(date)] $1"
}

logger "Starting datalogger"

if [ "$PWRSWITCH" -eq 0 ]; then
	/usr/bin/sudo /home/pi/openkin/linux-shutdown/pwr-switch &
elif [ "$PWRSWITCH" -eq 2 ]; then
	/usr/bin/sudo /usr/bin/python /home/pi/openkin/shutdown_flip.py &
fi

function led_on {
	if [ "${LED[$1]}" -eq 3 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		/usr/bin/sudo kill -USR1 ${LEDPID[$1]}
	fi

	if [ "${LED[$1]}" -eq 2 -o "${LED[$1]}" -eq 0 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		echo "ON" | /usr/bin/sudo /usr/bin/tee /tmp/ledpipe$1
	fi
}

function led_off {
	if [ "${LED[$1]}" -eq 3 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		/usr/bin/sudo kill -USR2 ${LEDPID[$1]}
	fi

	if [ "${LED[$1]}" -eq 2 -o "${LED[$1]}" -eq 0 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		echo "OFF" | /usr/bin/sudo /usr/bin/tee /tmp/ledpipe$1
	fi

}

function led_blink {
	if [ "${LED[$1]}" -eq 3 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		for n in {1..10}; do
			led_on $1
			sleep 0.4
			led_off $1
			sleep 0.2
		done
	fi

	if [ "${LED[$1]}" -eq 2 -o "${LED[$1]}" -eq 0 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		for n in {1..10}; do
			echo "BLINK" | /usr/bin/sudo /usr/bin/tee /tmp/ledpipe$1
		done
	fi
}

function led_blink_f {
	if [ "${LED[$1]}" -eq 3 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		for n in {1..30}; do
			led_on $1
			sleep 0.1
			led_off $1
			sleep 0.1
		done
	fi

	if [ "${LED[$1]}" -eq 2 -o "${LED[$1]}" -eq 0 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		for n in {1..30}; do
			echo "FASTER" | /usr/bin/sudo /usr/bin/tee /tmp/ledpipe$1
		done
	fi
}


# start led controllers
for i in {0..2}; do
	if [ "${LED[$i]}" -eq 3 -o "${LED[$i]}" -eq 2 -o "${LED[$i]}" -eq 0 ]; then
		if [ "${LED[$i]}" -eq 3 ]; then
			/usr/bin/sudo /usr/bin/python /home/pi/openkin/led-pin.py ${LEDGPIO[$i]} > /home/pi/led.log 2>&1 &
		fi

		if [ "${LED[$i]}" -eq 2 ]; then
			/usr/bin/sudo /bin/bash /home/pi/openkin/led-linuxpwm.sh $i ${LEDGPIO[$i]} > /dev/null 2>&1 &
		fi

		if [ "${LED[$i]}" -eq 0 ]; then
			/usr/bin/sudo /bin/bash /home/pi/openkin/led-linuxgpio.sh $i ${LEDGPIO[$i]} > /dev/null 2>&1 &
		fi

		SUDOPID[$i]=$!
		sleep 1
		LEDPID[$i]=$(/bin/ps --ppid ${SUDOPID[$i]} -o pid=)
		echo ${LEDPID[$i]}
		logger "Led $i connected"
		sleep 2
		led_on $i
		sleep 1
		led_off $i
	fi
done

function quitScreens {
	led_off 0
	led_off 2
	led_on 1

	if screen -list | grep -q "log"; then
		logger "Shutting down logger"
		screen -S log -X stuff $'\003'
		sleep 3
		screen -S log -X quit
	fi

	if screen -list | grep -q "bag"; then
		logger "Shutting down bag"
		#The bag needs to exit gracefully
		screen -S bag -X stuff $'\003'
		sleep 5
		screen -S bag -X quit
	fi

	if screen -list | grep -q "pozyx"; then
		logger "Shutting down the pozyx"
		screen -S pozyx -X quit
	fi

	if screen -list | grep -q "imu"; then
		logger "Shutting down the imu"
		screen -S imu -X quit
	fi

	if screen -list | grep -q "gps"; then
		logger "Shutting down the gps"
		screen -S gps -X quit
	fi

	# Reset errors
	IMUERR=1
	GPSERR=1
	POZYXERR=1

}

function check3G {
	if [ "$online" -ne 0 ]; then
		if lsusb | grep -q 12d1:1003; then
			logger "Connecting 3G"
			# Connect to 3G with Huawei E160, saunalahti
			sudo sakis3g connect OTHER="USBMODEM" USBMODEM="12d1:1003" APN="internet.saunalahti"
		fi
	fi
}

## Get Xsens device
#XSENS=""
##for f in /dev/serial/by-id/usb-Xsens_Xsens_COM_port*; do
#for f in /dev/serial/by-id/usb-Xsens_Xsens_*; do
#
#    ## Check if the glob gets expanded to existing files.
#    ## If not, f here will be exactly the pattern above
#    ## and the exists test will evaluate to false.
#    if [ -e "$f" ]; then
#         XSENS=$f
#    fi
#
#    ## This is all we needed to know, so we can break after the first iteration
#    break
#done

#/dev/serial/by-id/usb-Xsens_Xsens_USB-serial_converter_XSUO65V1-if00-port0

#I2C_ADAPTER=0
#for i in {0..10}; do
#	sudo i2cdetect -y -r $i 0x4b 0x4b | grep -q 4b
#	if [ $? -eq 0 ]; then
#		I2C_ADAPTER=$i
#		break
#	fi
#done

/usr/bin/sudo /usr/local/bin/pigpiod

logger "Starting loop"
while true; do

	check3G

	# enough timeout for 3g
	nc -w 30 -z 8.8.8.8 53  &> /dev/null
	online=$?
	if [ -f "online" ]; then
		online="$(cat online)"
	fi
	if [ $online -eq 0 ]; then

		if [[ $TIMECORRECTED -ne 0 ]] && sudo ntpdate time1.mikes.fi; then
			TIMECORRECTED=0
			logger "Time corrected"
			led_blink_f 0
			led_on 1
		elif [ "$TIMECORRECTED" -ne 0 ]; then
			logger "Failed to update time"
		fi

		# Stop IMU, GPS and recording
		quitScreens

		if [ "$STOPPED" -ne 0 ]; then

			#Uploading the data
			logger "Waiting for everything to shut down"
			while screen -list | grep "imu\|pozyx\|gps\|bag\|log"; do
				sleep 2
			done

			STOPPED=0

		fi


		if [ "$UPLOADED" -ne 0 ]; then
			if [ "$STARTEDUPLOAD" -ne 0 ] && ! ps -p $GRIVEPID > /dev/null 2>&1; then
				STARTEDUPLOAD=0
				logger "Uploading the data"
				#screen -r grive -X stuff $'\ngrive\n'
				# upload in subshell
				led_blink 1
				(cd $DATADIR; grive >> $LOGFILE) & >> $LOGFILE
				GRIVEPID=$!
			elif ps -p $GRIVEPID > /dev/null 2>&1; then
				led_blink 1
				sleep 2
			else
				UPLOADED=0
				STARTEDUPLOAD=1
				logger "Done uploading!"
			fi
			#UPLOADED=0
		fi
	else

		UPLOADED=1

#		if [ "$GRIVEPID" -ne -1 ]; then
#			logger "Stopping GRIVE, measuring, $GRIVEPID"
#			kill -- -"$GRIVEPID"
#			GRIVEPID=-1
#		fi

		if [ "$GPSERR" -gt 10 ]; then
			logger "GPS-errors more than 10, restarting"
			GPSERR=1
			quitScreens
		fi

		#NOTE! The gps is a launch file which will take care of starting the ros core
		#If this is not used, separate core process must be launched
		if ! screen -list | grep -q "gps"; then
			logger "Offline: Starting GPS and ROSCORE"
			screen -dmS gps
			screen -r gps -X stuff $'\nrm '$GPSTEMPFILE$'\nroslaunch ublox_gps ublox_gps.launch 2> '$GPSTEMPFILE$'\n'

			# should have time to error, if going to
			sleep 15

			# if file exists and not empty
			if [ -s $GPSTEMPFILE ]; then
				logger "Error on starting gps"
				logger "$(cat $GPSTEMPFILE)"
				screen -S gps -X quit
				led_off 0
				led_off 2
				# start over
				continue
			elif [ ! -f $GPSTEMPFILE ]; then
				logger "GPSlog not found"
				screen -S gps -X quit
				led_off 0
				led_off 2
				# start over
				continue
			fi

		fi

		## Not unless gps running
		if screen -list | grep -q "gps"; then

			if [ "$IMUERR" -gt 10 ]; then
				logger "IMU-errors more than 10, restaring IMU"
				IMUERR=1
				led_off 0
				led_off 2
				screen -S imu -X quit
			fi

			if [ "$POZYXERR" -gt 10 ]; then
				logger "Pozyx-errors more than 10, restaring Pozyx"
				POZYXERR=1
				led_off 0
				led_off 2
				screen -S pozyx -X quit
			fi

			if ! screen -list | grep -q "pozyx"; then
				logger "Offline: Starting Pozyx"
				screen -dmS pozyx
				#screen -r pozyx -X stuff $'\nsudo -s\nrosrun pozyx pozyx _adapter:='$I2C_ADAPTER$' > /home/pi/data/pozyx.log 2>&1\n'
				screen -r pozyx -X stuff $'\nrosrun pozyx pozyx > /home/pi/data/pozyx.log 2>&1\n'
				led_off 0
				led_off 2
			fi

			if ! screen -list | grep -q "imu"; then
				logger "Offline: Starting IMU"
				screen -dmS imu
				# /dev/serial/by-id/usb-Xsens_Xsens_COM_port_00342762-if00
				#screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py _device:=/dev/serial/by-id/usb-Xsens_Xsens_COM_port_00340764-if00\n'
				# Alignment reset will be soon
				led_blink 0
				screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py\n'
				led_off 0
				led_off 2
			fi

			if ! screen -list | grep -q "bag"; then
				logger "Offline: Starting RECORDING"
				screen -dmS bag
				screen -r bag -X stuff $'\ncd ~/data\nrosbag record -a\n'
				led_off 0
				led_off 2
			fi

			if ! (screen -list | grep -q "log") && [[ $IMUERR -eq 0 ]] && [[ $GPSERR -eq 0 ]] && [[ $POZYXERR -eq 0 ]]; then
				logger "Offline: Starting logger"
				screen -dmS log
				screen -r log -X stuff $'\nrosrun ascii_logger listener.py > /home/pi/ascii.log\n'
				led_on 2
				led_on 0
			fi
		else
			logger "GPS wasn't running!"
		fi

		STOPPED=1

		# Check we have wanted topics (so working)
		rostopic list > $ROSTOPICFILE 2>&1
		ROSRET=$?
		#logger "rostopic $ROSRET"
		#logger "$(cat $ROSTOPICFILE)"

		if ! grep -q "^/imu/data$" $ROSTOPICFILE; then
			logger "/imu/data not found"
			((IMUERR++))
		else
			#logger "zeroing imuerr"
			IMUERR=0
		fi

		if ! grep -q "^/pozyx/data$" $ROSTOPICFILE; then
			logger "/pozyx/data not found"
			((POZYXERR++))
		else
			POZYXERR=0
		fi

		if ! (grep -q "^/gps/navsol$" $ROSTOPICFILE && grep -q "^/gps/fix$" $ROSTOPICFILE && grep -q "^/gps/navposllh$" $ROSTOPICFILE && grep -q "^/gps/navvelned$" $ROSTOPICFILE); then
			logger "/gps/navsol, navposllh, navvelned or /gps/fix not found on rostopic"
			((GPSERR++))
		else
			#logger "zeroing gpserr"
			GPSERR=0
		fi

	fi

	#Sleeping a while
	sleep 2

done
