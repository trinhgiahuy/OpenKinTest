#!/bin/bash

/usr/bin/flock -n /home/openkin/openkin.lock -c "/home/openkin/openkin/measurement.sh &" > /dev/null 2>&1

exit $?
