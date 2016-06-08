Format of ASCII data files:
Column 1: GPS sequence number,
Column 2: Timestamp (seconds since epoch),
Column 3: IMU sequence number, # IMU data could be either from Xsens or Pozyx
Column 4: GPS latitude,
Column 5: GPS longitude,
Column 6: GPS altitude,
Column 7: IMU angular velocity X, # rad/s
Column 8: IMU Angular velocity Y,
Column 9: IMU Angular velocity Z,
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

All fields NaN if no value

If both Pozyx and Xsens connected, they will both generate imu data. They also might be a bit out of order regarding the timestamp.

File can be loaded to MATLAB with variable = load('filename')-command.

Example for angular velocity mean values:

function angmeans(testdata)
  nanmean(testdata(:, 7:9).*(180/pi).*3600./(sum(~isnan(testdata(:, 7)))/100))
end

Timestamps can be converted to time strings with:

TS = data(1, 2);
datestr(datevec(TS/60/60/24) + [1970 0 1 0 0 0])

