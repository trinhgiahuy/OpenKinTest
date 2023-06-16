#!/bin/bash

#VARIABLE TO CONFIGURE
HOME="/home/pi"

#source_setup=$(source /home/pi/ros_catkin_ws/devel_isolated/setup.bash)
source_setup=$(source /home/pi/ros_catkin_ws/devel/setup.bash)
echo $source_setup

GPSTEMPFILE="$HOME/gpstemp.log"
INSTEMPFILE="$HOME/instemp.log"
LOGFILE="$HOME/measurement.log"
DATADIR="$HOME/data"
ROSTOPICFILE="$HOME/rostmp.log"
ASCIILOG="$HOME/ascii.log"

LEDLOG="$HOME/led.log"
INSRECORDING="/tmp/insrecording"
BUZZLOG="$HOME/buzz.log"
BUZZRECORDING="/tmp/buzzrecording"

# LOGFILE="$HOME/logfile.log"

# GPS connected = 0, not = 1
GPS=1

# Vectornav is connected = 0, not = 1
INS=0

# LED mode connected = 0, not = 1, pwh = 2, rpigpio =3, inverser gpio = 4
LED[0]=4
#GPIO pin number
LEDGPIO[0]=24
LEDPID[0]=0

LED[1]=4
LEDGPIO[1]=25
LEDPID[1]=0

LED[2]=1
LEDGPIO[2]=10
LEDPID[2]=0

BUZZ[0]=4
BUZZGPIO[0]=21
BUZZPID[0]=0



#========END CONFIGURATION



function logger {
	echo "[$(date)] $1" >> $LOGFILE
	echo "[$(date)] $1"
}

function testCmd {
    if ! which $1; then
        logger "ERROR: Could not find $1!"
        return 1
    fi
    return 0
}

if [ "$1" = check ]; then
    testCmd rostopic
    testCmd sudo
    testCmd kill
    testCmd echo
    testCmd tee 
    testCmd python
    testCmd bash
    testCmd ps
    testCmd bash 
    testCmd screen
    testCmd lsusb

    # TODO: Implement this
    testCmd sakis3g
    
    testCmd udevadm
    testCmd ntpdate
    testCmd grive
    testCmd roslaunch
    testCmd cat
    testCmd rosrun
    testCmd rosbag
    testCmd readlink

    exit 0
fi


GRIVEID=-1

STOPPED=0
UPLOAD=1
STATEUPLOAD=1

IMUERR=1
GPSERR=0
INSERR=0

online=1

logger "START DATALOGGER"

rm $INSRECORDING
if [ ! -p "$INSRECORDING" ]; then
        touch $INSRECORDING
fi

function led_on {
	if [ "${LED[$1]}" -eq 3 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		sudo kill -USR1 ${LEDPID[$1]}
	fi

	if [ "${LED[$1]}" -eq 2 ] || [ "${LED[$1]}" -eq 0 ] || [ "${LED[$1]}" -eq 4 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		echo "ON" | sudo tee /tmp/ledpipe"$1"
	fi
}

function led_off {
	if [ "${LED[$1]}" -eq 3 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		sudo kill -USR2 ${LEDPID[$1]}
	fi

	if [ "${LED[$1]}" -eq 2 ] || [ "${LED[$1]}" -eq 0 ] || [ "${LED[$1]}" -eq 4 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
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

	if [ "${LED[$1]}" -eq 2 ] || [ "${LED[$1]}" -eq 0 ] || [ "${LED[$1]}" -eq 4 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
		echo "BLINK" | sudo tee /tmp/ledpipe"$1"
	fi
}

function buzz_on {
    if [ "${BUZZ[$1]}" -eq 4 ] && [ "${BUZZPID[$1]}" -ne 0 ]; then
	echo "ON" | sudo tee /tmp/buzzpipe"$1"
    fi    
}

function buzz_off {
    if [ "${BUZZ[$1]}" -eq 4 ] && [ "${BUZZPID[$1]}" -ne 0 ]; then
	echo "OFF" | sudo tee /tmp/buzzpipe"$1"
    fi    
}

function buzz_pulse {
    if [ "${BUZZ[$1]}" -eq 4 ]; then
	echo "PULSE" | sudo tee /tmp/buzzpipe"$1"
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

	if [ "${LED[$1]}" -eq 2 ] || [ "${LED[$1]}" -eq 0 ] || [ "${LED[$1]}" -eq 4 ] && [ "${LEDPID[$1]}" -ne 0 ]; then
			echo "FASTER" | sudo tee /tmp/ledpipe"$1"
	fi
}


# start led controllers
# for i in {0..2}; do
# 	if [ "${LED[$i]}" -eq 3 ] || [ "${LED[$i]}" -eq 2 ] || [ "${LED[$i]}" -eq 0 ] || [ "${LED[$i]}" -eq 4 ]; then
# 		if [ "${LED[$i]}" -eq 3 ]; then
# 			sudo python $HOME/openkin/led-pin.py ${LEDGPIO[$i]} >> $LEDLOG 2>&1 &
# 		fi

# 		if [ "${LED[$i]}" -eq 2 ]; then
# 			sudo bash $HOME/openkin/led-linuxpwm.sh $i ${LEDGPIO[$i]} >> $LEDLOG 2>&1 &
# 		fi

# 		if [ "${LED[$i]}" -eq 0 ]; then
# 			sudo bash $HOME/openkin/led-linuxgpio.sh $i ${LEDGPIO[$i]} >> $LEDLOG 2>&1 &
# 		fi

# 		if [ "${LED[$i]}" -eq 4 ]; then
# 			sudo bash $HOME/openkin/led-linuxgpio.sh $i ${LEDGPIO[$i]} 1 >> $LEDLOG 2>&1 &
# 		fi

# 		SUDOPID[$i]=$!
# 		echo $!
# 		sleep 1
# 		LEDPID[$i]=$(ps --ppid ${SUDOPID[$i]} -o pid=)
# 		echo "${LEDPID[$i]}"
# 		logger "Led $i connected"
# 		sleep 2
# 		led_on $i
# 		sleep 1
# 		led_off $i
# 	fi
# done


function quitScreens {
	led_off 0
	led_off 2

    if screen -list | grep -q "log"; then
        logger "Shutting down logger"

        # Send Ctrl+C signal to terminal session named log
        screen -S log -X stuff $'\003'
        sleep 3
        screen -S log -X quit
    fi


    if screen -list | grep -q "bag"; then
        logger "Shutting down bag"
        screen -S bag -X stuff $'\003'
        sleep 3
        screen -S bag -X quit
    fi


	if screen -list | grep -q "ins"; then
		logger "Shutting down the INS"
		screen -S ins -X stuff $'\003'
		sleep 4
		screen -S ins -X quit
	fi

    # Reset error
    INSERR=1
}

function check3G {
    
    logger "CHECKING 3G ..."
    
    # No online
    if [ "$online" -ne 0 ]; then
        # Change number according to dongle

        if lsusb | grep -q 12d1:1003; then
            logger "ATTEMPT CONNECTING TO 3G"

            # TODO: Check command this beforehand
	        sudo sakis3g connect OTHER="USBMODEM" USBMODEM="12d1:1003" APN="internet.saunalahti"        
        fi
    fi
}


#======== START MEASURMENT
logger "STARTING MEASURING LOOP ..."
#while true; do
for ((i=0;i<5;i++))
do
    # check3G
   logger "======== ITERATION $i ============"
	# enough timeout for 3g
	nc -w 30 -z 8.8.8.8 53  &> /dev/null


    # OFFLINE MDOE
    # led_off 1
    UPLOADED=1
    GRIVEID=0
    STARTEDUPLOAD=1
    if [ "$GPSERR" -gt 10 ]; then
        logger "GPS-errors more than 10, restarting"
        GPSERR=1
        quitScreens
    fi

    # UPDATED:

    if [ "$GPS" -eq 0 ]; then
        #NOTE! The gps is a launch file which will take care of starting the ros core
        #If this is not used, separate core process must be launched
        if ! screen -list | grep -q "gps"; then
            logger "Offline: Starting GPS and ROSCORE"
            screen -dmS gps
            screen -r gps -X stuff $'\nsudo -E bash\nrm '$GPSTEMPFILE$'\nnice -n -10 roslaunch ublox_gps ublox_gps.launch 2> '$GPSTEMPFILE$'\n'

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
    elif [ "$INS" -ne 0 ]; then
        if ! screen -list | grep -q "gps"; then 
            logger "Offline: Starting ROSCORE"
            screen -dmS gps
            screen -r gps -X stuff $'\nroscore\n'
        fi
    fi

    logger "Screen list before launch"
    logger "$(screen -list)"
    logger "INS before launch? $INS"

    # IF ROSCORE IS RUNNING
    if [ "$INS" -eq 0 ] || screen -list | grep -q "gps"; then
        if [ "$IMUERR" -gt 10 ]; then
            logger "IMU-errors more than 10, restarting IMUs"
            IMUERR=1
            led_off 0
            led_off 2
            screen -S imu -X quit
        fi

        if [ "$INSERR" -gt 10 ]; then
            logger "INS-errors more than 10, restarting vectornav"
            INSERR=1
            led_off 0
            led_off 2
            screen -S ins -X quit
        fi


        # INS connected
        echo "INS connected? $INS"
        if [ "$INS" -eq 0 ]; then
            if ! screen -list | grep -q "ins"; then
                logger "OFFLINE: Starting INS"
                screen -dmS ins
                screen -r ins -X stuff $'\n sudo -E bash\nrm '$INSTEMPFILE$'\nsource /home/pi/ros_catkin_ws/devel/setup.bash\nnice -n -10 roslaunch vectornav vn200.launch 2> '$INSTEMPFILE$'\n'
                
                sleep 15
            fi

            # If INS file exists and not empty
            if [ -s $INSTEMPFILE ]; then
                logger "Error on starting ins"
                logger "$(cat $INSTEMPFILE)"
                screen -S ins -X quit
                led_off 0
                led_off 2
                led_blink_f 1
                (sleep 2 && led_off 1)
                # Start over 
                continue 
            elif [ ! -f $INSTEMPFILE ]; then
                logger "INSlog not found"
                screen -S ins -X quit
                led_off 0
                led_off 2
                led_blink_f 1
                (sleep 2 && led_off 1)
                # start over 
                continue
            fi
        fi

        logger "Screen list after launch"
        logger "$(screen -list)"
        logger "INS after launch? $INS"

        logger "Check before launching logger"
        logger "IMUERR $IMUERR, INSERR $INSERR"
        if ! (screen -list | grep -q "log") && [[ $INSERR -eq 0 ]]; then
            logger "OFFLINE: Starting logger"
            screen -dmS log
            screen -r log -X stuff $'\nsource /home/pi/ros_catkin_ws/devel/setup.bash\nrosrun ascii_logger listener.py > '$ASCIILOG$'\n'
            #led_blink 2
            #led_blink 0
            #sleep 15
            echo "ON" > $INSRECORDING
        fi
    else
        logger "GPS/roscore wasn't running!"
    fi

    STOPPED=1

    # Check topics we want
    # echo $source_setup
    rostopic list > $ROSTOPICFILE 2>&1

    # Print rostopic file
    logger "cat ROSTOPICFILE"
    logger "$(cat $ROSTOPICFILE )"



    logger "[INSERR] check for topic $INSERR"
    logger "[GPSERR] check for topic $GPSERR" 
    logger "[INS] $INS"
    logger "[GPS] $GPS"
    if [ "$INS" -eq 0 ] && ! (grep -q "^/vectornav/imugps$" $ROSTOPICFILE); then
        logger "/vectornav/imugps not found"
        ((INSERR++))
    else
        INSERR=0
    fi

    if [ "$GPS" -eq 0 ] && ! (grep -q "^/gps/navpvtwh$" $ROSTOPICFILE && grep -q "^/gps/fix$" $ROSTOPICFILE); then
        logger "/.gps/navpvtwh or /gps/fix not found on rostopic"
        ((GPSERR++))
    elif [ "$GPS" -eq 1 ] && grep -q "^/rosout" $ROSTOPICFILE; then
        logger "/rosout not found"
        ((GPSERR++))
    else
        GPSERR=0
    fi

#    sudo -i
#    sync
#    sudo echo 1 > /proc/sys/vm/drop_caches
    sudo /home/pi/clear_cache.sh

    # Sleeping a while
    sleep 2
    exit

done

#sudo -i
#sudo echo 1 > /proc/sys/vm/drop_caches
#exit
sudo /home/pi/clear_cache.sh

