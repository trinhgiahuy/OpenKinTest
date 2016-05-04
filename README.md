# Arduino Libraries for Embedded Linux platforms

This project is dedicated to document the implementation of Arduino Libraries for
Linux Userspace. It is part of two GSOC projects at Beagleboard.org.

## Getting Started

### Prerequisites

Make sure you have the latest production image of the Angstrom Distribution on the Beaglebone Black
[here](http://circuitco.com/support/index.php?title=Updating_The_Software).

*If you prefer working on your host machine, go on. If you would prefer using the Beaglebone in a standalone environment,
skip to [development on a Beaglebone Black](#development-on-a-beaglebone-black)*

### Development on a linux host machine

* Get the Angstrom toolchain from [here](http://www.angstrom-distribution.org/toolchains/angstrom-2011.03-x86_64-linux-armv7a-linux-gnueabi-toolchain-qte-4.6.3.tar.bz2)

        wget http://www.angstrom-distribution.org/toolchains/angstrom-2011.03-x86_64-linux-armv7a-linux-gnueabi-toolchain-qte-4.6.3.tar.bz2
* Extract it to / (or any other place you prefer):

        sudo tar -C / -xjf angstrom-2011.03-x86_64-linux-armv7a-linux-gnueabi-toolchain-qte-4.6.3.tar.bz2
* Set up the environment variables

        export PATH=/usr/local/angstrom/arm/bin:$PATH # or wherever you extracted the toolchain
        export CROSS_COMPILE=arm-angstrom-linux-gnueabi-
* Get this repository

        git clone https://github.com/prpplague/Userspace-Arduino.git && cd Userspace-Arduino
* Copy the Bacon-overlay to the Beaglebone Black

        scp overlay/BB-BONE-BACONE-00A0.dtbo root@192.168.7.2:/lib/firmware
    *The default username is root and password is blank.*
    *The above syntax will change if you have the Beaglebone connected over Ethernet instead of USB.*
    *In that case, you need to know the IP address of the Bone and change the syntax accordingly*
* Run an example

        cd arduino-makefile/examples/BlinkUserspace/
        make upload # will compile on the host machine and execute on the Beaglebone Black
    *You should see one of the 4 LEDs blinking at a steady interval*
        
### Development on a Beaglebone Black

    	git clone http://github.com/prpplague/Userspace-Arduino.git

Change directory to the examples/BlinkUserspace folder in the downloaded project	

       cd Userspace-Arduino/arduino-makefile/examples/BlinkUserspace

Compile the code
	
       make

Change to new directory that is created, which contains the executable file

       cd build-userspace

Execute code

       ./BlinkUserspace.elf

You will now see USR LED 1 blinking at one second intervals. Press Ctrl-C to break from the loop. 	

## PinMap
The project uses the pin map found [here](http://www.elinux.org/Userspace_Arduino#Beaglebone_Black)
## About

This project will be worked upon by two students, Parav and Anuj; David Anders,
Matt Porter and Andrew Bradford have been assigned as the mentors.

project pages:

http://www.elinux.org/Userspace_Arduino

http://www.google-melange.com/gsoc/project/google/gsoc2013/anujdeshpande92/51001

https://www.google-melange.com/gsoc/project/google/gsoc2013/hatguy/24001

http://beagleboard.org/
