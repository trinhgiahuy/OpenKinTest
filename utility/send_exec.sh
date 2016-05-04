#!/bin/bash


# COMMAND LINE ARGUMENTS
# ./send_exec.sh arduinofile

# A bash script to upload an executable binary to beagleboard and return the output

IP="192.168.7.2"
USERNAME="root"
FILE=$1
EXECUTABLE=${FILE}.elf

echo
echo -n "Transfering ${FILE} to ${USERNAME}@${IP}..."
scp -q build-userspace/${EXECUTABLE} ${USERNAME}@${IP}:~/${FILE}
echo "Executing!"
echo
echo
echo "Begin output log"
echo "===================="
ssh ${USERNAME}@${IP} -t "./"${FILE}

return_result=$?

if [ ${return_result} = 130 ] ; then
    exit 0
fi

