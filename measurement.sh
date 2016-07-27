#!/bin/bash

# VARIABLES TO CONFIGURE

# Needed for rostopic etc
#source /opt/ros/indigo/setup.bash
#source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

HOME="/home/pi"

GPSTEMPFILE="$HOME/gpstemp.log"
LOGFILE="$HOME/measurement.log"
DATADIR="$HOME/data"
ROSTOPICFILE="$HOME/rostmp.log"
POZYXLOG="$HOME/data/pozyx.log"
MTWLOG="$HOME/data/mtw.log"
ASCIILOG="$HOME/ascii.log"
LEDLOG="$HOME/led.log"
# empty logfile
#> $LOGFILE

# GNSS connected = 0, not = 1
GPS=0

# wireless MTw connected = 0, not = 1
MTW=1

# wired MTi connected = 0, not = 1
MTI=0

# Pozyx connected = 0, not = 1
POZYX=0
# Linux I2C device = 0, not (pigpiod) = 1
LINUXI2C=1

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

# CONFIGURATION ENDS HERE

function logger {
	echo "[$(date)] $1" >> $LOGFILE
	echo "[$(date)] $1"
}

function testCmd {
	if ! which "$1"; then
		logger "Error: Didn't find $1!"
		return 1
	fi
	return 0
}

if [ "$1" = "check" ]; then
	testCmd rostopic
	testCmd sudo
	testCmd kill
	testCmd echo
	testCmd tee
	testCmd python
	testCmd bash
	testCmd ps
	testCmd screen
	testCmd grep
	testCmd lsusb
	testCmd sakis3g
	testCmd udevadm
	if [ "$POZYX" -eq 0 ] && [ "$LINUXI2C" -eq 0 ]; then
		testCmd i2cdetect
	fi
	testCmd nc
	testCmd ntpdate
	testCmd grive
	testCmd roslaunch
	testCmd cat
	testCmd rosrun
	testCmd rosbag
	testCmd $HOME/openkin/linux-shutdown/pwr-switch
	testCmd readlink

	exit 0
fi

GRIVEPID=-1

STOPPED=0
UPLOADED=1
STARTEDUPLOAD=1

IMUERR=1
GPSERR=1
POZYXERR=1

TIMECORRECTED=1

online=1

logger "Starting datalogger"

if [ "$PWRSWITCH" -eq 0 ]; then
	sudo $HOME/openkin/linux-shutdown/pwr-switch &
elif [ "$PWRSWITCH" -eq 2 ]; then
	sudo python $HOME/openkin/shutdown_flip.py &
fi

function led_on {
	if [ "${LED[$1]}" -eq 3 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		sudo kill -USR1 ${LEDPID[$1]}
	fi

	if [ "${LED[$1]}" -eq 2 ] || [ "${LED[$1]}" -eq 0 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		echo "ON" | sudo tee /tmp/ledpipe"$1"
	fi
}

function led_off {
	if [ "${LED[$1]}" -eq 3 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		sudo kill -USR2 ${LEDPID[$1]}
	fi

	if [ "${LED[$1]}" -eq 2 ] || [ "${LED[$1]}" -eq 0 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		echo "OFF" | sudo tee /tmp/ledpipe"$1"
	fi

}

function led_blink {
	if [ "${LED[$1]}" -eq 3 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		for _ in {1..10}; do
			led_on "$1"
			sleep 0.4
			led_off "$1"
			sleep 0.2
		done
	fi

	if [ "${LED[$1]}" -eq 2 ] || [ "${LED[$1]}" -eq 0 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		for _ in {1..10}; do
			echo "BLINK" | sudo tee /tmp/ledpipe"$1"
		done
	fi
}

function led_blink_f {
	if [ "${LED[$1]}" -eq 3 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		for _ in {1..30}; do
			led_on "$1"
			sleep 0.1
			led_off "$1"
			sleep 0.1
		done
	fi

	if [ "${LED[$1]}" -eq 2 ] || [ "${LED[$1]}" -eq 0 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		for _ in {1..30}; do
			echo "FASTER" | sudo tee /tmp/ledpipe"$1"
		done
	fi
}


# start led controllers
for i in {0..2}; do
	if [ "${LED[$i]}" -eq 3 ] || [ "${LED[$i]}" -eq 2 ] || [ "${LED[$i]}" -eq 0 ]; then
		if [ "${LED[$i]}" -eq 3 ]; then
			sudo python $HOME/openkin/led-pin.py ${LEDGPIO[$i]} > $LEDLOG 2>&1 &
		fi

		if [ "${LED[$i]}" -eq 2 ]; then
			sudo bash $HOME/openkin/led-linuxpwm.sh $i ${LEDGPIO[$i]} > /dev/null 2>&1 &
		fi

		if [ "${LED[$i]}" -eq 0 ]; then
			sudo bash $HOME/openkin/led-linuxgpio.sh $i ${LEDGPIO[$i]} > /dev/null 2>&1 &
		fi

		SUDOPID[$i]=$!
		sleep 1
		LEDPID[$i]=$(ps --ppid ${SUDOPID[$i]} -o pid=)
		echo "${LEDPID[$i]}"
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
		screen -S pozyx -X stuff $'\003'
		sleep 5
		screen -S pozyx -X quit
	fi

	if screen -list | grep -q "imu"; then
		logger "Shutting down the imu (MTi)"
		screen -S imu -X quit
	fi

	if screen -list | grep -q "mtw"; then
		logger "Shutting down the MTw"
		screen -S mtw -X stuff $'\003'
		sleep 4
		screen -S mtw -X quit
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
#	logger "Checking 3G..."
	if [ "$online" -ne 0 ]; then
		if lsusb | grep -q 12d1:1003; then
			logger "Connecting 3G"
			# Connect to 3G with Huawei E160, saunalahti
			sudo sakis3g connect OTHER="USBMODEM" USBMODEM="12d1:1003" APN="internet.saunalahti"
		fi
	fi
}

function findXsens {

	logger "Finding XSens..."
	# Get Xsens device
	XSENS=""
	AWINDA=""
	#for f in /dev/serial/by-id/usb-Xsens_Xsens_COM_port*; do
	for f in /dev/serial/by-id/usb-Xsens_Xsens_*; do

		## Check if the glob gets expanded to existing files.
		## If not, f here will be exactly the pattern above
		## and the exists test will evaluate to false.
		if [ -e "$f" ]; then
			ID=$(udevadm info -n "$f" | grep -oP 'ID_MODEL_ID=\K.+$')
			if [ "$ID" = "d38b" ]; then
				XSENS=$f
			elif [ "$ID" = "d38d" ]; then
				AWINDA=$(readlink -f "$f")
			fi
		fi

		## This is all we needed to know, so we can break after the first iteration
		if [ "$MTI" -eq 0 ] && [ "$MTW" -eq 1 ]; then
			if [ "$XSENS" != "" ]; then
				break
			fi
		elif [ "$MTI" -eq 1 ] && [ "$MTW" -eq 0 ]; then
			if [ "$AWINDA" != "" ]; then
				break
			fi
		else
			if [ "$AWINDA" != "" ] && [ "$XSENS" != "" ]; then
				break
			fi
		fi
	done

	#/dev/serial/by-id/usb-Xsens_Xsens_USB-serial_converter_XSUO65V1-if00-port0
}

if [ "$MTI" -eq 0 ] || [ "$MTW" -eq 0 ]; then
	findXsens
fi

if [ "$POZYX" -eq 0 ] && [ "$LINUXI2C" -eq 0 ]; then
	logger "Finding Pozyx..."
	I2C_ADAPTER=0
	for i in {0..10}; do
		sudo i2cdetect -y -r $i 0x4b 0x4b | grep -q 4b
		if [ $? -eq 0 ]; then
			I2C_ADAPTER=$i
			break
		fi
	done
fi

logger "Starting loop"
while true; do
#	logger "Running loop"
	check3G

	# enough timeout for 3g
	nc -w 30 -z 8.8.8.8 53  &> /dev/null
	online=$?
	if [ -f "online" ]; then
		online="$(cat online)"
	fi
	if [ "$online" -eq 0 ]; then

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
			COUNTER=0
			while screen -list | grep "imu\|mtw\|pozyx\|gps\|bag\|log"; do
				sleep 2
				((COUNTER++))
				if [ "$COUNTER" -gt 10 ]; then
					quitScreens
					COUNTER=0
				fi
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
				(cd $DATADIR || exit; grive >> $LOGFILE) & >> $LOGFILE
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
		GRIVEPID=0
		STARTEDUPLOAD=1

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

		if [ "$GPS" -eq 0 ]; then
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
		else
			if ! screen -list | grep -q "gps"; then
				logger "Offline: Starting ROSCORE"
				screen -dmS gps
				screen -r gps -X stuff $'\nroscore\n'
			fi
		fi

		## Not unless gps/roscore running
		if screen -list | grep -q "gps"; then

			if [ "$IMUERR" -gt 10 ]; then
				logger "IMU-errors more than 10, restarting IMUs"
				IMUERR=1
				led_off 0
				led_off 2
				if [ "$MTI" -eq 0 ]; then
					screen -S imu -X quit
				fi
				if [ "$MTW" -eq 0 ]; then
					screen -S mtw -X quit
				fi
			fi

			if [ "$POZYXERR" -gt 10 ]; then
				logger "Pozyx-errors more than 10, restarting Pozyx"
				POZYXERR=1
				led_off 0
				led_off 2
				screen -S pozyx -X quit
			fi

			if [ "$POZYX" -eq 0 ]; then
				if ! screen -list | grep -q "pozyx"; then
					logger "Offline: Starting Pozyx"
					screen -dmS pozyx
					screen -r pozyx -X stuff $'\nsudo -s\nrosrun pozyx pozyx _adapter:='$I2C_ADAPTER$' > '$POZYXLOG$' 2>&1\n'
					led_off 0
					led_off 2
				fi
			fi

			if [ "$MTI" -eq 0 ]; then
				if ! screen -list | grep -q "imu"; then
					logger "Offline: Starting MTi"
					screen -dmS imu
					# /dev/serial/by-id/usb-Xsens_Xsens_COM_port_00342762-if00
					#screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py _device:=/dev/serial/by-id/usb-Xsens_Xsens_COM_port_00340764-if00\n'
					# Alignment reset will be soon
					led_blink 0
					#screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py\n'
					if [ "$XSENS" -eq "" ]; then
						findXsens
					else
						screen -r imu -X stuff $'\nrosrun xsens_driver mtnode_new.py _device:='"$XSENS"$'\n'
					fi
					led_off 0
					led_off 2
				fi
			fi

			if [ "$MTW" -eq 0 ]; then
				if ! screen -list | grep -q "mtw"; then
					logger "Offline: Starting MTw"
					screen -dmS mtw
					# Alignment reset will be soon
					led_blink 0
					if [ "$AWINDA" -eq "" ]; then
						findXsens
					else
						#screen -r mtw -X stuff $'\nsudo -s\nrosrun mtw_node mtw_node _device:='"$AWINDA"$' &> '$MTWLOG$'\n'
						screen -r mtw -X stuff $'\nrosrun mtw_node mtw_node _device:='"$AWINDA"$' &> '$MTWLOG$'\n'
					fi
					led_off 0
					led_off 2
				fi
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
				screen -r log -X stuff $'\nrosrun ascii_logger listener.py > '$ASCIILOG$'\n'
				led_on 2
				led_on 0
			fi
		else
			logger "GPS/roscore wasn't running!"
		fi

		STOPPED=1

		# Check we have wanted topics (so working)
		rostopic list > $ROSTOPICFILE 2>&1
		#ROSRET=$?
		#logger "rostopic $ROSRET"
		#logger "$(cat $ROSTOPICFILE)"

		if [ "$MTI" -eq 0 ] || [ "$MTW" -eq 0 ] && ! grep -q "^/imu/data$" $ROSTOPICFILE; then
			logger "/imu/data not found"
			((IMUERR++))
		else
			#logger "zeroing imuerr"
			IMUERR=0
		fi

		if [ "$POZYX" -eq 0 ] && ! grep -q "^/pozyx/.*$" $ROSTOPICFILE; then
			logger "/pozyx/.* not found"
			((POZYXERR++))
		else
			POZYXERR=0
		fi

		if [ "$GPS" -eq 0 ] && ! (grep -q "^/gps/navsol$" $ROSTOPICFILE && grep -q "^/gps/fix$" $ROSTOPICFILE && grep -q "^/gps/navposllh$" $ROSTOPICFILE && grep -q "^/gps/navvelned$" $ROSTOPICFILE); then
			logger "/gps/navsol, navposllh, navvelned or /gps/fix not found on rostopic"
			((GPSERR++))
		elif [ "$GPS" -eq 1 ] && ! grep -q "^/rosout$" $ROSTOPICFILE; then
			logger "/rosout not found"
			((GPSERR++))
		else
			#logger "zeroing gpserr"
			GPSERR=0
		fi

	fi

	#Sleeping a while
	sleep 2

done
