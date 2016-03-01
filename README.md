
# Usage
This section describes the basic usage of the device

## Connecting the devices
* The Xsens IMU units are currently connected through the USB ports and should be connected before powering the device.
* GPS is connected to the GPIO pins UART pins. The device is enumerated as /dev/ttyAMA0 and use 57600 baud. This all is configured in the ROS launch files
* It is recommended that the ethernet cable is plugged to a router which will assign an IP address to the device, currently the OS has issues if you switch to a different router while the device is powered.
* The device gets time from internet, so logs are dated to 1970 if no internet connection has been available before starting measurements

## Powering
* The device is powered by the powerbank from the USB connector. The runtime with this seems to be around 5-6h.

## Measurements
* The measurement mode is activated in the boot
* The device will start recording as soon as it loses internet connectivity
* It might take a few minutes to start the measurements, if there are errors with gps or imu. Device will try again until everything works.
* It seems, the IMU wont start if it has been stationary for some time, so light shake might be needed before use.
* Recording will be stopped once the internet connectivity is restored.
* After being stopped, the files will be uploaded to googledrive (tutopenkin)

## Shutting down
* The power switch on top can be flipped to give shutdown commands to the unit.
* After flipping the switch and a few seconds, the USB cable powering the device can be unplugged.

# Connecting to the device
Standard SSH conenction can be made to the device with:
    
    ssh ubuntu@[IP]
    Password: ubuntu


# Libraries utilized

ROS Driver for XSens MT/MTi/MTi-G devices.

See: http://ros.org/wiki/ethzasl_xsens_driver

ROS driver for ublox

See: https://github.com/KumarRobotics/ublox

Both are added as subtrees to this repo, so updates can be fetched with commands:

```
git subtree pull --prefix nodes/ublox https://github.com/KumarRobotics/ublox.git master --squash
```

and

```
git subtree pull --prefix nodes/ethzasl_xsens_driver https://github.com/ethz-asl/ethzasl_xsens_driver.git master --squash
```

# Motion Fusion

http://msthub.ain.rd.tut.fi/mobilemachines/motion_fusion/blob/master/src/KalmanImu.py


# Script startup
The measurements are started by running ~/openkin/start_script.sh, which in turn calls the main script at ~/openkin/measurement.sh using locks.
The script now starts from /etc/rc.local, with lockfile at /home/ubuntu/measurement.lock and as user ubuntu.
