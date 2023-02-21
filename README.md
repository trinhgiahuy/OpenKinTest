## Paper: **Surrogate Modelling for Oxygen Uptake Prediction Using LSTM Neural Network**

The article can be accessed through [mdpi](https://www.mdpi.com/1424-8220/23/4/2249)

## Source code for devices measuring in the campaign

## Old schemantic plot

![Old_schema](https://user-images.githubusercontent.com/65078173/216986042-6d5d5640-f352-429a-8253-3752d5ab2262.jpg)


## TC219 CPU info

```
Device name	HTC219-720-SPC
Full device name	HTC219-720-SPC.ad.tuni.fi
Processor	Intel(R) Core(TM) i7-6700 CPU @ 3.40GHz   3.41 GHz
Installed RAM	32,0 GB
Device ID	bbbf3aca-47cd-4ad8-9fd0-ad23414388c6
Product ID	00329-00000-00003-AA689
System type	64-bit operating system, x64-based processor
Pen and touch	No pen or touch input is available for this display
```

# Usage

## Matlab instruction

Test repo for matab: `D:\(...BackupDdrive)\(..RA)\....PAVEL\MATLAB-APP\test`

- In matlab folder, create folder name ${subjectID}_out, for example `O1_out`
- The directory should include these files:
  * 2022-XX-XX-XX-XX-XX_s.txt (**sliced** ID's gaitpod file)
  * OXYGAX_marker_XXXX_s.txt(**sliced** ID'oxygen file)
  * `run.m` executable file run the everything
  * `features_OK.m`
  * `velSeg.m`
  * `input_features.m`
  * `trainingPartitions.m`
  
- At the begging `run.mat`, change subjectID and change `sortedInputFeatures` to true if want output input features in ordet (false by default) and run the file
- After about 5 mins it will produce input feature file


This section describes the basic usage of the device

## Connecting the devices
* The Xsens IMU units are currently connected through the USB ports and should be connected before powering the device.
* GPS is connected to the UART pins. The device is enumerated as /dev/ttyAMA0 on RPi and /dev/ttyS4 on minnowboard and uses 57600 baud. This all is configured in the ROS launch files.
* It is recommended that the ethernet cable is plugged to a router which will assign an IP address to the device (unless using 3G), currently the OS has issues if you switch to a different router while the device is powered?
* The device gets time from internet, so logs are dated to 1970 or some other wrong time if no internet connection has been available before starting measurements

## Powering
* The device is powered by the powerbank from the USB connector. The runtime with this seems to be around 3-4 h for RPi and 2,5 h for minnowboard.

## Measurements
* The measurement mode is activated in the boot
* Primary led will flash when time is corrected from interney
* The device will start recording as soon as it loses internet connectivity
* It might take a few minutes to start the measurements, if there are errors with gps or imu. Device will try again until everything works.
* Primary led will flash when making IMU alignment, so IMU should be stationary that time and a bit after.
* It seems, the IMU wont start if it has been stationary for some time (MTw wired), so light shake might be needed before use.
* When recording, primary led will be lit.
* Recording will be stopped once the internet connectivity is restored.
* After being stopped, the files will be uploaded to Google Drive (tutopenkin@gmail.com)

## Shutting down
* The power switch on top can be flipped to give shutdown commands to the unit.
* After flipping the switch and waiting some seconds, the USB cable powering the device can be unplugged.

# Connecting to the device
Standard SSH conenction can be made to the device with ssh:

    ssh ubuntu@[IP]
    Password: ubuntu

For RPi with kinetic user: pi, password: raspberry
For minnowboard user: openkin, password: openkin

# Libraries utilized

ROS Driver for XSens MT/MTi/MTi-G devices.

See: http://ros.org/wiki/ethzasl_xsens_driver

ROS driver for ublox

See: https://github.com/KumarRobotics/ublox

Pozyx Arduino library

See: https://github.com/pozyxLabs/Pozyx-Arduino-library

All are added as subtrees to this repo, so updates can be fetched with commands (from git root):

```
git subtree pull --prefix nodes/ublox https://github.com/KumarRobotics/ublox.git master --squash
```
,

```
git subtree pull --prefix nodes/ethzasl_xsens_driver https://github.com/ethz-asl/ethzasl_xsens_driver.git master --squash
```
and
```
git subtree pull --prefix nodes/pozyx/include/Pozyx-custom-library/ https://github.com/pozyxLabs/Pozyx-Arduino-library.git master --squash
cd nodes/pozyx/ínclude
patch -p0 < pozyx-arduino-to-linux.patch
```

# Motion Fusion

http://msthub.ain.rd.tut.fi/mobilemachines/motion_fusion/blob/master/src/KalmanImu.py


# Script startup
The measurements are started by running ~/openkin/start_script.sh, which in turn calls the main script at ~/openkin/measurement.sh using locks.
The script now starts from /etc/rc.local, with lockfile at /home/ubuntu/measurement.lock and as user ubuntu (or pi or openkin).

# Old documentation...
... can be found in Google Docs with the tutopenkin@gmail.com -account.

# For latest documentation, please look at [doc/](doc/)
