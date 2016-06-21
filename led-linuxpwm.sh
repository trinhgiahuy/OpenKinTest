#!/bin/bash

if [ ! -f "/sys/class/pwm/pwmchip0/pwm0/period" ]; then
	echo 0 > /sys/class/pwm/pwmchip0/export
fi
sleep 2
echo 0 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
echo 2000 > /sys/class/pwm/pwmchip0/pwm0/period
echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable

if [ ! -p "/tmp/ledpipe" ]; then
	mkfifo /tmp/ledpipe
fi

function led_on {
	echo 2000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
}

function led_off {
	echo 0 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
}

function led_blink {
	for i in {1..2000..128}; do
		echo $i > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
		sleep 0.0008
	done
	sleep 0.3
	for i in {2000..1..128}; do
		echo $i > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
		sleep 0.0008
	done
	sleep 0.15
	echo 0 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
}

function led_blink_f {
	for i in {1..2000..768}; do
		echo $i > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
		sleep 0.016
	done
	for i in {2000..1..768}; do
		echo $i > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
		sleep 0.016
	done
	echo 0 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
}

#trap led_on SIGUSR1
#trap led_off SIGUSR2

for (( ; ; )); do

while read SIGNAL; do
	case "$SIGNAL" in
		*EXIT*)break;;
		*ON*)led_on;;
		*OFF*)led_off;;
		*BLINK*)led_blink;;
		*FASTER*)led_blink_f;;
		*)echo "Signal $SIGNAL is unsupported" > /dev/stderr;;
	esac
done < /tmp/ledpipe
#wait
#kill %1

done
