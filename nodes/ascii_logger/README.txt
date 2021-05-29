Format of ASCII data files:
Column 1: GPS sequence number,
Column 2: Timestamp (seconds since epoch),
Column 3: IMU sequence number, # IMU data could be either from Xsens or Pozyx
Column 4: GPS latitude,
Column 5: GPS longitude,
Column 6: GPS altitude,
Column 7: IMU angular velocity X, # rad/s
Column 8: IMU angular velocity Y,
Column 9: IMU angular velocity Z,
Column 10: IMU orientation.x, # always quaternion
Column 11: IMU orientation.y,
Column 12: IMU orientation.z,
Column 13: IMU orientation.w,
Column 14: IMU linear acceleration X, # m/s^2
Column 15: IMU linear acceleration Y,
Column 16: IMU linear acceleration Z
Column 17: iTOW     # GPS Millisecond time of week [ms]
Column 18: velN     # NED north velocity [cm/s]
Column 19: velE     # NED east velocity [cm/s]
Column 20: velD     # NED down velocity [cm/s]
Column 21: speed    # Speed (3-D) [cm/s]
Column 22: gSpeed   # Ground Speed (2-D) [cm/s]
Column 23: heading  # Heading of motion 2-D [deg / 1e-5]
Column 24: sAcc     # Speed Accuracy Estimate [cm/s]
Column 25: cAcc     # Course / Heading Accuracy Estimate [deg / 1e-5]
Column 26: 1 if IMU from Pozyx, 0 if Xsens
Column 27: Pozyx sequence number for position
Column 28: Pozyx position X
Column 29: Pozyx position Y
Column 30: Pozyx position Z
Column 31: Pozyx ranges to anchors (more details below)
Column 32: IMU identifier
If more IMUs:
Column 33: 2. IMU sequence number,
Column 34: 2. IMU identifier,
Column 35: 2. IMU angular velocity X, # rad/s
Column 36: 2. IMU angular velocity Y,
Column 37: 2. IMU angular velocity Z,
Column 38: 2. IMU orientation.x, # always quaternion
Column 39: 2. IMU orientation.y,
Column 40: 2. IMU orientation.z,
Column 41: 2. IMU orientation.w,
Column 42: 2. IMU linear acceleration X, # m/s^2
Column 43: 2. IMU linear acceleration Y,
Column 44: 2. IMU linear acceleration Z
Column 45: 3. IMU sequence number,
Column 46: 3. IMU identifier,
...

All fields NaN if no value

Example of ranges:
s=uwb,t=6010,tu=30014,ts=1470123704193,d=1779,RSS=-83|s=uwb,t=601b,tu=30028,ts=1470123704193,d=1404,RSS=-84|s=uwb,t=601c,tu=30044,ts=1470123704193,d=736,RSS=-79|
The anchors are separated with | -characters, and fields for a single anchor are separated with commas.
To get the data to same format as spoonphone, the data file needs to be run through parse_data.py to separate ranges and other data.

Usage: python parse_data.py ORIGINAL [NORANGES-OUTPUT [RANGES-OUTPUT]]
Where ORIGINAL is the logfile from Google Drive, and outputs are optional filenames for output files.
Requires Python to be installed. (Might be required to be called "python parse_data.py ORIGINAL...")

The ranges in RANGES-OUTPUT are in the format:
s=uwb,t=6010,tu=30014,ts=1470123704193,d=1779,RSS=-83
s=uwb,t=601b,tu=30028,ts=1470123704193,d=1404,RSS=-84
s=uwb,t=601c,tu=30044,ts=1470123704193,d=736,RSS=-79
...

The t-parameter is the idetifier of the anchor (hex), tu is the timestamp from pozyx in milliseconds, ts is the timestamp from logger,
same for all ranges at the same round. d is the distance in millimeters, and RSS is the received signal strength.

This format can be given to parse_spoondata.m scipt, and the output from that can be given to spoon2uwb.m, which outputs the arguments
needed for uwb_filter.m.

If both Pozyx and Xsens connected, they will both generate imu data (if setting is on from Pozyx driver).
File can be loaded to MATLAB after parse_data.py with variable = load('filename')-command.

Example for angular velocity mean values:

function angmeans(testdata)
  nanmean(testdata(:, 7:9).*(180/pi).*3600)
end

Timestamps can be converted to time strings with:

TS = data(1, 2);
datestr(datevec(TS/60/60/24) + [1970 0 1 0 0 0])

