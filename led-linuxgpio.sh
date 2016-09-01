#!/bin/bash

if [ "$#" -lt 2 ]; then
	echo "Not enough paramters!"
	return 1
fi

NUM=$1
GPIO=$2
if [ "$#" -gt 2 ]; then
	ON=0
	OFF=1
else
	ON=1
	OFF=0
fi

if [ ! -f "/sys/class/gpio/gpio$GPIO/value" ]; then
	echo $GPIO > /sys/class/gpio/export
fi
sleep 2
echo "out" > /sys/class/gpio/gpio$GPIO/direction
echo 0 > /sys/class/gpio/gpio$GPIO/value

rm /tmp/ledpipe$NUM

if [ ! -p "/tmp/ledpipe$NUM" ]; then
	mkfifo /tmp/ledpipe$NUM
fi

function led_on {
	echo $ON > /sys/class/gpio/gpio$GPIO/value
}

function led_off {
	echo $OFF > /sys/class/gpio/gpio$GPIO/value
}

function led_blink {
	echo $ON > /sys/class/gpio/gpio$GPIO/value
	sleep 0.3
	echo $OFF > /sys/class/gpio/gpio$GPIO/value
	sleep 0.15
}

function led_blink_f {
	echo $ON > /sys/class/gpio/gpio$GPIO/value
	sleep 0.1
	echo $OFF > /sys/class/gpio/gpio$GPIO/value
	sleep 0.05
}

#trap led_on SIGUSR1
#trap led_off SIGUSR2

for (( ; ; )); do
#echo "New round"

while read SIGNAL; do
	case "$SIGNAL" in
		*EXIT*)break;;
		*ON*)led_on;;
		*OFF*)led_off;;
		*BLINK*)led_blink;;
		*FASTER*)led_blink_f;;
		*)echo "Signal $SIGNAL is unsupported" > /dev/stderr;;
	esac
#	echo $SIGNAL
done < /tmp/ledpipe$NUM
#wait
#kill %1

done
