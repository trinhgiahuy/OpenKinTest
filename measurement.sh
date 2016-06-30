#!/bin/bash
#TODO might be a good idea to start the nodes at the beginning and only start/stop the bag recording

# Needed for rostopic
#source /opt/ros/indigo/setup.bash
source /opt/ros/kinetic/setup.bash
#source ~/catkin_ws/devel/setup.bash

GPSTEMPFILE="/home/openkin/gpstemp.log"
LOGFILE="/home/openkin/measurement.log"
DATADIR="/home/openkin/data"
ROSTOPICFILE="/home/openkin/rostmp.log"
# empty logfile
#> $LOGFILE

# led connected = 0, not = 1, pwm = 2
LED=2
LEDPID=0

# shutdown switch connected = 0, not = 1
PWRSWITCH=0

#TODO: options for gps, imu, pozyx

GRIVEPID=-1

STOPPED=0
UPLOADED=1

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
	sudo /home/openkin/openkin/linux-shutdown/pwr-switch &
fi

function led_on {
	if [ "$LED" -eq 0 ] && [ "$LEDPID" -ne 0 ]; then
		/usr/bin/sudo kill -USR1 $LEDPID
	fi

	if [ "$LED" -eq 2 ] && [ "$LEDPID" -ne 0 ]; then
		echo "ON" | /usr/bin/sudo /usr/bin/tee /tmp/ledpipe
	fi
}

function led_off {
	if [ "$LED" -eq 0 ] && [ "$LEDPID" -ne 0 ]; then
		/usr/bin/sudo kill -USR2 $LEDPID
	fi

	if [ "$LED" -eq 2 ] && [ "$LEDPID" -ne 0 ]; then
		echo "OFF" | /usr/bin/sudo /usr/bin/tee /tmp/ledpipe
	fi

}

function led_blink {
	if [ "$LED" -eq 0 ] && [ "$LEDPID" -ne 0 ]; then
		for n in {1..10}; do
			led_on
			sleep 0.4
			led_off
			sleep 0.2
		done
	fi

	if [ "$LED" -eq 2 ] && [ "$LEDPID" -ne 0 ]; then
		for n in {1..10}; do
			echo "BLINK" | /usr/bin/sudo /usr/bin/tee /tmp/ledpipe
		done
	fi
}

function led_blink_f {
	if [ "$LED" -eq 0 ] && [ "$LEDPID" -ne 0 ]; then
		for n in {1..50}; do
			led_on
			sleep 0.1
			led_off
			sleep 0.1
		done
	fi

	if [ "$LED" -eq 2 ] && [ "$LEDPID" -ne 0 ]; then
		for n in {1..50}; do
			echo "FASTER" | /usr/bin/sudo /usr/bin/tee /tmp/ledpipe
		done
	fi
}

if [ "$LED" -eq 0 -o "$LED" -eq 2 ]; then
	if [ "$LED" -eq 0 ]; then
		/usr/bin/sudo /usr/bin/python /home/openkin/openkin/led-pin.py > /home/openkin/led.log 2>&1 &
	fi

	if [ "$LED" -eq 2 ]; then
		/usr/bin/sudo /bin/bash /home/openkin/openkin/led-linuxpwm.sh > /dev/null 2>&1 &
	fi
	SUDOPID=$!
	sleep 1
	LEDPID=$(/bin/ps --ppid $SUDOPID -o pid=)
	echo $LEDPID
	logger "Led connected"
	led_on
	sleep 1
	led_off
fi


function quitScreens {
	led_off


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

# Get Xsens device
XSENS=""
#for f in /dev/serial/by-id/usb-Xsens_Xsens_COM_port*; do
for f in /dev/serial/by-id/usb-Xsens_Xsens_*; do

    ## Check if the glob gets expanded to existing files.
    ## If not, f here will be exactly the pattern above
    ## and the exists test will evaluate to false.
    if [ -e "$f" ]; then
         XSENS=$f
    fi

    ## This is all we needed to know, so we can break after the first iteration
    break
done

#/dev/serial/by-id/usb-Xsens_Xsens_USB-serial_converter_XSUO65V1-if00-port0

I2C_ADAPTER=0
for i in {0..10}; do
	sudo i2cdetect -y -r $i 0x4b 0x4b | grep -q 4b
	if [ $? -eq 0 ]; then
		I2C_ADAPTER=$i
		break
	fi
done

while true; do

	check3G

	# enough timeout for 3g
	nc -w 30 -z 8.8.8.8 53  >/dev/null 2>&1
	online=$?
	if [ -f "online" ]; then
		online="$(cat online)"
	fi
	if [ $online -eq 0 ]; then

		if [[ $TIMECORRECTED -ne 0 ]] && sudo ntpdate time1.mikes.fi; then
			TIMECORRECTED=0
			logger "Time corrected"
			led_blink_f
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
			if ! ps -p $GRIVEPID > /dev/null 2>&1; then
				logger "Uploading the data"
				#screen -r grive -X stuff $'\ngrive\n'
				# upload in subshell, wait for finish (should we?)
				(cd $DATADIR; grive >> $LOGFILE) & >> $LOGFILE
				GRIVEPID=$!
			fi
			UPLOADED=0
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
				led_off
				# start over
				continue
			elif [ ! -f $GPSTEMPFILE ]; then
				logger "GPSlog not found"
				screen -S gps -X quit
				led_off
				# start over
				continue
			fi

		fi

		## Not unless gps running
		if screen -list | grep -q "gps"; then

			if [ "$IMUERR" -gt 10 ]; then
				logger "IMU-errors more than 10, restaring IMU"
				IMUERR=1
				led_off
				screen -S imu -X quit
			fi

			if [ "$POZYXERR" -gt 10 ]; then
				logger "Pozyx-errors more than 10, restaring Pozyx"
				POZYXERR=1
				led_off
				screen -S pozyx -X quit
			fi

			if ! screen -list | grep -q "pozyx"; then
				logger "Offline: Starting Pozyx"
				screen -dmS pozyx
				screen -r pozyx -X stuff $'\nsudo -s\nrosrun pozyx pozyx _adapter:='$I2C_ADAPTER$' > /home/openkin/data/pozyx.log 2>&1\n'
				led_off
			fi

			if ! screen -list | grep -q "imu"; then
				logger "Offline: Starting IMU"
				screen -dmS imu
				# /dev/serial/by-id/usb-Xsens_Xsens_COM_port_00342762-if00
				#screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py _device:=/dev/serial/by-id/usb-Xsens_Xsens_COM_port_00340764-if00\n'
				# Alignment reset will be soon
				led_blink
				screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py _device:='$XSENS$'\n'
				led_off
			fi

			if ! screen -list | grep -q "bag"; then
				logger "Offline: Starting RECORDING"
				screen -dmS bag
				screen -r bag -X stuff $'\ncd ~/data\nrosbag record -a\n'
				led_off
			fi

			if ! (screen -list | grep -q "log") && [[ $IMUERR -eq 0 ]] && [[ $GPSERR -eq 0 ]] && [[ $POZYXERR -eq 0 ]]; then
				logger "Offline: Starting logger"
				screen -dmS log
				screen -r log -X stuff $'\nrosrun ascii_logger listener.py > /home/openkin/ascii.log\n'
				led_on
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
