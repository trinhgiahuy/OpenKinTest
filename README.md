
# Usage
This section describes the basic usage of the device

## Connecting the devices
* The Xsens IMU units are currently connected through the USB ports and should be connected before powering the device.
* GPS is connected to the GPIO pins UART pins. The device is enumerated as /dev/ttyAMA0 and use 57600 baud. This all is configured in the ROS launch files
* It is recommemded that the ethernat cable is plugged to a router which will assign an IP address to the device, currently the OS has issues if you switch to a different router while the device is powered.

## Powering
* The device is powered by the powerbank from the USB connector. The runtime with this seems to be around 5-6h.

## Measurements
* The measurement mode is activated in the boot
* The device will start recording as soon as it loses internet connectivity
* Recording will be stopped once the internet connectivity is restored.
* After being stopped, the fiels will be uploaded to googledrive (tutopenkin)

## Shutting down
* The power switch on top can be flipped to give shutdown commands to the unit
* After flipping the switch, the USB cable powering the device can be unplugged

# Connecting to the device
Standard SSH conenction can be made to the device with:
    
    ssh ubuntu$[IP]
    Password: ubuntu


# Libraries utilized

ROS Driver for XSens MT/MTi/MTi-G devices.

See: http://ros.org/wiki/ethzasl_xsens_driver

# Motion Fusion

http://msthub.ain.rd.tut.fi/mobilemachines/motion_fusion/blob/master/src/KalmanImu.py


# Script startup
Currently the startup of the script is located at ~/openkin/measurement.sh
This script is executed with the crontab but this will likely need more thorough testing.
/etc/rc.local would be another potential place.