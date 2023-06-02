#!/bin/bash

HOME="/home/pi"
source "$HOME/catkin_ws/devel/setup.bash"

LEDLOG="$HOME/led_small.log"

LED[0]=4
LEDGPIO[0]=24
LEDPID[0]=0

LED[1]=4
LEDGPIO[1]=25
LEDPID[1]=0

LED[2]=1
LEDGPIO[2]=10
LEDPID[2]=0

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
for i in {0..2}; do
	if [ "${LED[$i]}" -eq 3 ] || [ "${LED[$i]}" -eq 2 ] || [ "${LED[$i]}" -eq 0 ] || [ "${LED[$i]}" -eq 4 ]; then
		if [ "${LED[$i]}" -eq 3 ]; then
			sudo python $HOME/openkin/led-pin.py ${LEDGPIO[$i]} >> $LEDLOG 2>&1 &
		fi

		if [ "${LED[$i]}" -eq 2 ]; then
			sudo bash $HOME/openkin/led-linuxpwm.sh $i ${LEDGPIO[$i]} >> $LEDLOG 2>&1 &
		fi

		if [ "${LED[$i]}" -eq 0 ]; then
			sudo bash $HOME/openkin/led-linuxgpio.sh $i ${LEDGPIO[$i]} >> $LEDLOG 2>&1 &
		fi

		if [ "${LED[$i]}" -eq 4 ]; then
			sudo bash $HOME/openkin/led-linuxgpio.sh $i ${LEDGPIO[$i]} 1 >> $LEDLOG 2>&1 &
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
