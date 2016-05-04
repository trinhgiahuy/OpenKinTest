#!/bin/bash


# COMMAND LINE ARGUMENTS
# ./send_exec.sh arduinofile

# A bash script to upload an executable binary to beagleboard and return the output

IP="192.168.7.2"
USERNAME="root"
FILE=$1
EXECUTABLE=${FILE}.elf

scp build-userspace/${EXECUTABLE} ${USERNAME}@${IP}:~/${FILE}

