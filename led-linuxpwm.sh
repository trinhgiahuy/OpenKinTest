#!/bin/bash

if [ "$#" -ne 2 ]; then
	echo "Not enough paramters!"
	return 1
fi

NUM=$1
CHIP=$2

if [ ! -f "/sys/class/pwm/pwmchip$CHIP/pwm0/period" ]; then
	echo 0 > /sys/class/pwm/pwmchip$CHIP/export
fi
sleep 2
echo 2000 > /sys/class/pwm/pwmchip$CHIP/pwm0/period
echo 0 > /sys/class/pwm/pwmchip$CHIP/pwm0/duty_cycle
echo 1 > /sys/class/pwm/pwmchip$CHIP/pwm0/enable

rm /tmp/ledpipe$NUM

if [ ! -p "/tmp/ledpipe$NUM" ]; then
	mkfifo /tmp/ledpipe$NUM
fi

function led_on {
	echo 2000 > /sys/class/pwm/pwmchip$CHIP/pwm0/duty_cycle
}

function led_off {
	echo 0 > /sys/class/pwm/pwmchip$CHIP/pwm0/duty_cycle
}

function led_blink {
	while true; do
		for i in {1..2000..128}; do
			echo $i > /sys/class/pwm/pwmchip$CHIP/pwm0/duty_cycle
			sleep 0.0008
		done
		sleep 0.3
		for i in {2000..1..128}; do
			echo $i > /sys/class/pwm/pwmchip$CHIP/pwm0/duty_cycle
			sleep 0.0008
		done
		sleep 0.15
		echo 0 > /sys/class/pwm/pwmchip$CHIP/pwm0/duty_cycle
	done
}

function led_blink_f {
	while true; do
		for i in {1..2000..768}; do
			echo $i > /sys/class/pwm/pwmchip$CHIP/pwm0/duty_cycle
			sleep 0.016
		done
		for i in {2000..1..768}; do
			echo $i > /sys/class/pwm/pwmchip$CHIP/pwm0/duty_cycle
			sleep 0.016
		done
		echo 0 > /sys/class/pwm/pwmchip$CHIP/pwm0/duty_cycle
	done
}

#trap led_on SIGUSR1
#trap led_off SIGUSR2

for (( ; ; )); do
#echo "New round"

while read SIGNAL; do
        case "$SIGNAL" in
                *EXIT*)break;;
                *ON*)
                        kill $!
                        led_on
                        ;;
                *OFF*)
                        kill $!
                        led_off
                        ;;
                *BLINK*)
                        kill $!
                        led_blink &
                        ;;
                *FASTER*)
                        kill $!
                        led_blink_f &
                        ;;
                *)echo "Signal $SIGNAL is unsupported" > /dev/stderr;;
        esac
#       echo $SIGNAL
        sleep 0.2
done < /tmp/ledpipe$NUM
#wait
#kill %1

done
