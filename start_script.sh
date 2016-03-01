#!/bin/bash

/usr/bin/flock -n /home/ubuntu/measurement.lock -c "/home/ubuntu/openkin/measurement.sh &" > /dev/null 2>&1

exit $?
