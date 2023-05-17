#!/bin/bash

#VARIABLE TO CONFIGURE
HOME="/home/pi"

source "$HOME/catkin_ws/devel/setup.bash"

LOGFILE="$HOME/measurement.log"
DATADIR="$HOME/data"
ROSTOPICFILE="$HOME/rostmp.log"
LEDLOG="$HOME/led.log"
INSRECORDING="/tmp/insrecording"

# GPS connected = 0, not = 1
GPS=1

# Vectornav is connected = 0, not = 1
INS=0

# LED mode connected = 0, not = 1, pwh = 2, rpigpio =3, inverser gpio = 4
LED[0]=4
#GPIO pin number
LEDGPIO[0]=24
LEDPID[0]=0

#========END CONFIGURATION



function logger {
	echo "[$(date)] $1" >> $LOGFILE
	echo "[$(date)] $1"
}

function testCmd{
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
GPSERR=1
INSERR=1

online=1

logger "START DATALOGGER"

rm $INSRECORDING
if [ ! -p "$INSRECORDING" ]; then
        touch $INSRECORDING
fi

function led_on {
    if [ "${LED[$1]}" -eq 3 ] && [ LEDGPIO[] -ne 0];then
        sudo kill -USR1 ${LEDPID[$1]}
    fi

    if [ "${LED[$1]}" -eq 2 ] || [ "${LED[$1]}" - eq 0 ] || [ "${LED[$1]}}" -eq 4] && [ "${LED[$1]}" -ne 0 ]; then 
        echo "OFF" | sudo tee /tmp/ledpip"$1"
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

# Start led controller
for in {0..2}; do
    if [ "${LED[$i]}" -eq 3 ] || [ "${LED[$i]}" -eq 2 ] || [ "${LED[$i]}" -eq 0 ] || [ "${LED[$i]}" -eq 4 ];then
        if [ "${LED[$i]}" -eq 4];then
            sudo python $HOME/gaitmaven/led-linuxgpio.sh $i ${LEDGPIO[$i]} >> $LEDLOG 2>1 &
        fi
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
done


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
while true; do

    check3G

	# enough timeout for 3g
	nc -w 30 -z 8.8.8.8 53  &> /dev/null

    online=$?
    if [ -f "online"]; then
        online="$(cat online)"
    fi

    # ONLINE MODE
    if [ "$online" -eq 0 ] then


        # TIME IS NOT CORRECTED
        if [ "$TIMECORRECTED" -ne 0 ]; then
            sudo date +%Y%m%d -s "20230101"
            logger "TIME TO THE PAST..."
        fi


        # TODO: Test this comment
        if [[ "$TIMECORRECTED" -ne  0 ]] && sudo ntpdate -b time1.mike.fi time2.mikes.fi; then
            TIMECORRECTED=0
            logger "TIME CORRECTED"
            led_blink_f 0
            (sleep 5 && led_off 0) &
        fi

        # Stop IMU, GPS and recording
		quitScreens

		if [ "$STOPPED" -ne 0 ]; then

			echo "OFF" > $INSRECORDING
			#Uploading the data
			logger "Waiting for everything to shut down"
			COUNTER=0
			while screen -list | grep "ins\|imu\|mtw\|pozyx\|gps\|bag\|log"; do
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
            if [ "$STARTUPLOADED" -ne 0 ] && ! ps -p $GRIVEID > /dev/null 2>&1; then
                STARTUPLOADED=0
                logger "UPLOADING THE DATA..."
                led_blink 1
                DATE="$(date)"
                (cd $DATADIR || exit; bash autoupdate.sh "${DATE}") >> $LOGFILE &
                logger "CALLED AUTOUPDATE BASH FILE"
                GRIVEID=$!
            elif ps -p $GRIVEID > /dev/null 2>&1; then
                led_blink 1
                sleep 2
            else
                UPLOADED=0
                STARTUPLOADED=1
                led_on 1
                logger "DONE UPLOADING"
            fi
        fi
    else
    # OFFLINE MDOE
        led_off 1
        UPLOADED=1
        GRIVEID=0
        STARTEDUPLOAD=1
		if [ "$GPSERR" -gt 10 ]; then
			logger "GPS-errors more than 10, restarting"
			GPSERR=1
			quitScreens
		fi

        # UPDATED:
        if [ "$INS" -ne 0 ]; then
            if ! screen -list | grep -q "gps"; then 
                logger "Offline: Starting ROSCORE"
                screen -dmS gps
               screen -r gps -X stuff $'\nroscore\n'
            fi
        fi

        # IF ROSCORE IS RUNNING
        if [ "$INS" -eq 0] || screen -list | grep -q "gps"; then
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
            if [ "$INS" -eq 0 ]; then
                if ! screen -list | grep -q "ins"; then
                    logger "OFFLINE: Starting INS"
                    screen -dmS ins
                    screen -r ins -X stuff $'\n sudo -E bash\nrm '$INSTEMPFILE$'\nnice -n -10 roslaunch vectornav vn200.launch 2> '$INSTEMPFILE$'\n'
                    
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


            if ! (screen -list | grep -q "log") && [[ $IMUERR -eq 0 ]] && [[ $INS -eq 0 ]]; then
                logger "OFFLINE: Starting logger"
                screen --dmS log
                screen -r log -X stuff $'\nrosrun ascii_logger listener.py > '$ASCIILOG$'\n'
                led_blink 2
                led_blink 0
                echo "ON" > $INSRECORDING
            fi
        else
            logger "GPS/roscore wasn't running!"
        fi

        STOPPED=1

        # Check topics we want
        rostopic list > $ROSTOPICFILE 2>&1

        if [ "$INS" -eq 0 ] && ! grep -q "^/vectornav/imugps$" $ROSTOPICFILE; then
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
    fi

    # Sleeping a while
    sleep 2



    # USBFOLDER="/media/pi/USB_DISK"
    # if [ -e $USBFOLDER ]; then
    #     led_blink_f 0
    #     ( sleep 5 && led_off 0)&
    #     quitScreens


    #     # STOP MEASURING FO 
    #     if [ "$STOPPED" -ne 0 ]; then
    #         logger "STOP MEASURING FOR UPLOADING DATA"
            
    #         echo "OFF" > $INSRECORDING
    #         # UPLOADING THE DATA
    #         logger "Waitting for everything to shutdown"
    #         COUNTER=0
    #         while screen -list | grep "ins\|bag\|log"; do
    #             sleep 2
    #             ((COUNTER++))
    #             if [ "$COUNTER" -gt 10 ]; then
    #                 quitScreens
    #                 COUNTER=0
    #             fi
    #         done
    #         STOPPED=0
    #     fi
    
    # # NOT YET UPLOAD (1), NOW START UPLOADING
    # if [ "$UPLOADED" -ne 0 ]; then
    #     if [ "$STARTUPLOADED" -ne 0 ] && ! ps -p $GRIVEID > /dev/null 2>&1; then
    #         STARTUPLOADED=0
    #         logger "UPLOADING DATA ..."

    #         led_blink 1
    #         DATE="($date)"

    #         # CALL UPLOAD SCRIPT HERE
    #         # TODO: Check this file
    #         bash autoupdate.sh
    #     fi
    # fi

done
