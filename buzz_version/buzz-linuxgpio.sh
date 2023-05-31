#!/bin/bash

if [ "$#" -lt 2 ]; then
    echo "Not enough paramters!"
fi

NUM=$1
#GPIO here is GPIO 21
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

rm /tmp/buzzpipe$NUM

if [ ! -p "/tmp/buzzpipe$NUM" ]; then
    mkfifo /tmp/buzzpipe$NUM
fi

function buzz_on {
    echo $ON > /sys/class/gpio/gpio$GPIO/value
}

function buzz_off {
    echo $OFF > /sys/class/gpio/gpio$GPIO/value
}

function buzz_pulse {
    while true; do 
        echo $ON > /sys/class/gpio/gpio$GPIO/value
        sleep 0.1
	echo $OFF > /sys/class/gpio/gpio$GPIO/value
        sleep 0.1
	echo $ON > /sys/class/gpio/gpio$GPIO/value
        sleep 0.1
	echo $OFF > /sys/class/gpio/gpio$GPIO/value
        sleep 0.1
	echo $ON > /sys/class/gpio/gpio$GPIO/value
        sleep 0.1
	#echo $OFF > /sys/class/gpio/gpio$GPIO/value
        #sleep 0.3
	#echo $ON > /sys/class/gpio/gpio$GPIO/value
        #sleep 0.3
        echo $OFF > /sys/class/gpio/gpio$GPIO/value
        sleep 5
    done
}

function buzz_pulse_f {
    while true; do 
        echo $ON > /sys/class/gpio/gpio$GPIO/value
        sleep 0.1
        echo $OFF > /sys/class/gpio/gpio$GPIO/value
        sleep 0.05
    done
}

for(( ; ; )); do
#echo "New round"

while read SIGNAL; do
    case "$SIGNAL" in
        *EXIT*)break;;
        *ON*)
            kill $!
            buzz_on
            ;;
        *OFF*)
            kill $!
            buzz_off
            ;;
        *PULSE*)
            kill $!
            buzz_pulse &
            ;;
        *PULSE_F*)
            kill $!
            buzz_pulse_f &
            ;;
        *)echo "Signal $SIGNAL is unsupported" > /dev/stderr;;
    esac
    echo $SIGNAL
    echo 0.2
done < /tmp/buzzpipe$NUM
#wait
#kill %1

done
