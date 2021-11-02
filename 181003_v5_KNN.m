% code for OpenKin datalogger's and moticon insole's data processing-- 

%% Code lines which may need some modifications for different tests 

% if the same code is being used for different walking/running tests 
% then the following lines may need some modifications-- 

% Line no: 30      ->> name or (path+name) of the datalogger file
% Line no: 43      ->> index of datalogger required test data.   
% Line no: 66-67   ->> INS frame defination  
% Line no: 108-109 ->> recommended 
% Line no: 217-233 ->> as per device's orientation on human back 
% Line no: 247     ->> name or (path+name) of the moticon's data file 
% Line no: 262-287 ->> left and right insoles sync points 
% Line no: 590-605 ->> to see if the step segmentation using insoles is correct or not  
% NOTE: If any chnages in line no's (add/delete) are to be made before line
% no. 605 then the above list should be modified.   

%% Load datalogger's data 

% This section parses the complete dataset collected by INS datalogger and
% the foot IMU data (foot IMU is present only in few tests).

% The INS data is logged at the rate of 400Hz. Each row in the data file
% contains the data of all sensors at one time instance.  

% Therefore, the sensor data in two consecutive rows are separated by the
% time of 0.0025 sec.  

walkdata = cell_array_parser_vn200_mod('2018-10-13-07-58-14.txt', 1);

%The purpose of foot IMU data was to achieve better sync with moticon's
%foot insole data but it doesn't help much so the foot imu data is ignored
%in later sections.  

%% INS-data to matrices

% 'ind' is range, of data indices of INS datalogger, in which the walking/
% running test is perfrmed. If the data file contains the logged data from
% ealier time then the differnece between those two times can be very large
% so it better to exclude the older data from processing    

ind = 1:size(walkdata,2); % range of indeces of test performed. 
ts = arrayfun(@(c) c.time.ros_stamp, walkdata(ind)); % ROS timestamp 

% imu data seq. 0->65535->0->65535
imuseq = arrayfun(@(c) c.time.imu_seq, walkdata(ind)); 

times = arrayfun(@(c) c.time.gpstime, walkdata(ind)); %GPS ticks
plot(times);
%ID of row, 'ins' if row contains the data from datalogger or '/base_imu'
%if it is from foot imu.  
ids = arrayfun(@(c) c.imu.id, walkdata(ind)); 

insind = find(contains(ids(1,:),{'ins'})); % indices of INS data

%indices of foot IMU data (in this test foot imu was not present)
mtiind = find(contains(ids(1,:),{'/base_imu'}));

% Acceleration data from data logger at human back and foot IMU. 
acc = cell2mat( arrayfun(@(c) c.imu.acc(:), walkdata(ind), 'UniformOutput', 0) ); 
acc_ins=acc(:,insind); % Acceleration data from data logger at human back. 

% This test was performed using new INS orientation in device.
%Frame definations.
% INS frame axis in new device (when device is on human's back), y:
% forward, z upward, x: towards right hand.  
% Anatomical frame: for human movement, X in forward, Y in human's right 
% hand's direction and Z in vertically upwards direction. 

% Rotation quaternion (in XYZ order)
quat = cell2mat( arrayfun(@(c) c.imu.quat(:), walkdata(ind), 'UniformOutput', 0) );
for k=1:length(quat)
    %quaternion to euler angles (by using matlab's aerospace toolbox)
    [yaw(k), pitch(k), roll(k)] = quat2angle( [quat(4,k) quat(1,k) quat(2,k) quat(3,k)]); 
    
    % quaternion to euler angles (without using matlab's toolbox)
    % [yaw(k), pitch(k), roll(k)] = quat2euler([quat1(k) quat2(k) quat3(k) quat4(k)]); 
end

% Rotation angles obtained from INS datalogger (around sensor frame axis)
roll_ins = roll(insind);
pitch_ins = pitch(insind);
yaw_ins =  yaw(insind); 

% unwrap yaw angle to avoid jumps at pi and -pi. 
yaw_unwrap=unwrap(yaw_ins); 

% extract yaw angle oscillations
yaw_osc= highpass(yaw_unwrap, 0.005,'StopbandAttenuation',30,'Steepness',0.7); 

v_d= cell2mat(arrayfun(@(c) c.gps.v_d(:), walkdata(ind), 'UniformOutput', 0));% vertical velocity in downwards direction 
v_e= cell2mat(arrayfun(@(c) c.gps.v_e(:), walkdata(ind), 'UniformOutput', 0));
v_n= cell2mat(arrayfun(@(c) c.gps.v_n(:), walkdata(ind), 'UniformOutput', 0));

V_d = -v_d(insind);%vertical velocity in upwards direction from INS/GPS (sensor fusion) 
v_e = v_e(insind);%velocity in geographical east direction from INS/GPS (sensor fusion) 
v_n = v_n(insind);%velocity in geographical north direction from INSGPS (sensor fusion) 

ang_vel= cell2mat(arrayfun(@(c) c.imu.ang_vel(:), walkdata(ind), 'UniformOutput', 0));
ang_vel_ins =  ang_vel(:,insind);%angular velocity from IMU in INS datalogger 

gpsvel_n= cell2mat(arrayfun(@(c) c.gps.raw_v_n(:), walkdata(ind), 'UniformOutput', 0));
gpsvel_e= cell2mat(arrayfun(@(c) c.gps.raw_v_e(:), walkdata(ind), 'UniformOutput', 0));
gpsvel_n= gpsvel_n(:,insind);%velocity in geographical north direction by only GPS 
gpsvel_e= gpsvel_e(:,insind);%velocity in geographical east direction by only GPS 

%After this section is executed, it is better to delete variable 'walkdata'
%from matlab workspace for fast processing in sections below. 

%% Plot yaw angle? 
plot(rad2deg(yaw_ins))
hold on
plot(rad2deg(yaw_unwrap))
plot(rad2deg(yaw_osc))

%% To Generate 'custom_eul2rotm.m' file, if file already available then it is not needed to execute this section

% rotation matrix for transforming acceleration vectors in
% three-dimensional space from frame-A to frame-B.

% let's, first consider orientation X,Y,Z as axis of frame-B and
% x,y,z as of frame-A.

syms psi theta phi R_i1_B R_i2_i1 R_A_i2 R_A_B R_B_A 
% psi (yaw) around Z axis, theta (pitch) around Y axis, and phi (roll)
% around X axis  

% R_i1_b is rotation matrix from frame-B to first intermediate frame. 
    
% frame-B to first intermediate frame (right hand rotation around Z axis)
% X',Y',Z 
R_i1_B = [cos(psi)  sin(psi) 0 ;...
          -sin(psi) cos(psi) 0 ;...
             0         0     1];
         
% first intermediate frame to second intermediate frame (right hand
% rotation around Y axis) 
%i.e. X'',Y',Z'
R_i2_i1 = [cos(theta) 0 -sin(theta);...
              0       1       0   ;...
           sin(theta) 0 cos(theta)];
       
% second intermediate frame to a-frame (right hand rotation around x axis)
% X'',Y'',Z'' or x,y,z
R_A_i2 = [1    0         0     ;...
          0 cos(phi)  sin(phi) ;...
          0 -sin(phi) cos(phi)]; 
R_A_B = R_A_i2 * R_i2_i1 * R_i1_B; %frame-B to frame-A

% inverse of Rotation matrix is same as tranpose matrix
R_B_A = R_A_B.'; % frame-A to frame-B

%save symbolic function to the matlab function in .m file
matlabFunction(R_B_A,'File','custom_eul2rotm','Optimize',false,'Vars',[psi, theta, phi]);

% so, this function is good to use with yaw, pitch, roll to convert 
% accelerations from INS to geographical frame meaning that INS frame
% is frame-A and geographical frame is frame-B.    

%% Transform accelerations from INS (sensor) frame to geographical frame. 

acc_geo=zeros(3,length(insind));
for k=1:length(insind)
    %accelerations in geographical frame (n,e,downwards)
    %The order of the input angles should be the yaw, pitch and roll. 
    acc_geo(:,k) = custom_eul2rotm(yaw(k), pitch_ins(k), roll_ins(k)) * acc_ins(:,k); 
    %k
end

%% Transform accelerations from INS (sensor) frame to geographical frame (using robotics toolbox). 

% %alternate, by using inbuilt robotics toolbox.  
% acc_geo1=zeros(3,length(insind));
% for k=1:length(insind)
%     acc_geo1(:,k) =  eul2rotm([yaw(k), pitch_ins(k), roll_ins(k)],'ZYX')*acc_ins(:,k);
%     %k
% end

%% Ground track and velocity calculations 

GrndTrack= zeros(1,length(insind)); 
v_lon= zeros(1,length(insind));
v_lat= zeros(1,length(insind));
V_gps_lon= zeros(1,length(insind));

for k=1:length(insind)    
    % angle between north and forward direction. *not very accurate when
    % person is standing or slow walking (<1.5 m/s).  
    GrndTrack(k)=rad2deg(atan2(v_e(k),v_n(k)));
    % INS/GPS (sensor fusion) horizontal speed (forward direction of movement)
    v_lon(k) = sqrt(v_n(k)^2 + v_e(k)^2); 
    % Lateral velocity (in direction of right hand) constraint V_y = 0 (straight ine running?)
    v_lat(k) = v_e(k)*cosd(GrndTrack(k)) - v_n(k)*sind(GrndTrack(k));
    % GPS horizontal speed (only GPS), forward direction of movement
    V_gps_lon(k) = sqrt(gpsvel_n(k)^2 + gpsvel_e(k)^2);   
%     k
end

%% acceleration calculations in anatomical frame

acc_anatomical = zeros(3,length(insind));
for k=1:length(insind)
    % acc. in x direction of anatomical frame (i.e. forward or londitudinal)
    acc_anatomical(1,k) =  acc_geo(1,k)*cosd(GrndTrack(k)) + acc_geo(2,k)*sind(GrndTrack(k)); 
    % acceleration in lateral direction. 
    acc_anatomical(2,k) =  acc_geo(2,k)*cosd(GrndTrack(k)) - acc_geo(1,k)*sind(GrndTrack(k));
    %pure acceleration (without gravity bias) in vertically upward dir.
    acc_anatomical(3,k) =  -acc_geo(3,k)-9.81;
end

%% plot anatomical accelerations? 

plot(acc_anatomical(3,:)) %upward direction
hold on
plot(acc_anatomical(1,:)) % forward or londitudinal direction 

%% Euler with reference to Anatomical frame
% Since human body is also oscillating as INS datalogger, when walking/
% running. (assuming device is tightly attached to the body).    

% % So, body-frame (anatomical frame), x axis in forward and y axis in
% human right hand's direction and z in upward direction. This frame
% oscillates when person walks/runs around X,Y,Z by means of roll_anatomical,
% roll_anatomical, yaw_anatomical.

%These equation depends on the orientation of the sensor frame with the
%anatomical (body) frame 
roll_anatomical=  pitch_ins;
pitch_anatomical= roll_ins;
yaw_anatomical=   yaw_ins;

% angular velocities are yet to be tranformed/ calculated in geo and anatomical frame. 

%% Atomic/ TIA (or UTC) time conversion 
%Currently, 
% UTC Epoch - GPS Epoch= (315964800 - 18) % 18, leap second correction
% Also, GPS & UTC Epoch (seconds) are in order of 10 digits, until next 2 centuries. 

% variable 'times'/10e8 -> GPS time in sec (or GPS epoch) 

% times2 = arrayfun(@(x) x/10e8 + 315964800 - 18, times);% gps ticks to UTC time conversion.
times2 = arrayfun(@(x) x/10e8 + 315964800 + 19, times);% gps ticks to atomic time (TAI) conversion.
times2ins=times2(insind);

%% load moticon insoles data

raw = importdata('dharmendra_181003.txt', '\t', 2);
raw = raw.data;
% fill missing values with approximation
raw = fillmissing(raw, 'spline', 1,'EndValues','nearest');

%% plot to sync time between moticon and datalogger

figure;
plot(raw(:,1), raw(:,2)+raw(:,3)+raw(:,13)+raw(:,14));
hold on;
plot(raw(:,1), raw(:,2+19)+raw(:,3+19)+raw(:,13+19)+raw(:,14+19));

figure;
plot(times2ins(1,:), acc_ins(3,:));

%% time sync with two points for each foot at the time of jump landing 
% sync points for each foot 

%lsync = [left_peak_foot_pressure_time  coresponding_peak_vertical_acc_TIA_time; ...]
%These points needs to be changed if GPS/UTC time is used for sync. 
%TIA is better since it is not changed with leap sec. addition as UTC. 


lsync = [107.58  37+1.538553314535636e+09; ...
         533.00  37+1.538553739865062e+09; ...
         663.09  37+1.538553869958137e+09; ...
         951.44  37+1.538554158354394e+09];

rsync = [107.54  37+1.538553314535636e+09; ...
         532.98  37+1.538553739865062e+09; ...
         663.06  37+1.538553869958137e+09; ...
         951.44  37+1.538554158354394e+09];

ltimes = zeros(length(raw),1);
rtimes = zeros(length(raw),1);

% calculate timestamps for each moticon sample with piecewise linear
% approximation

if (length(lsync) > 1)
    i = 2;
    for k = 1:length(raw)
        rtime = raw(k,1);
        while (rtime > lsync(i) && length(lsync) > i)
            i = i+1;
        end
        ltimes(k) = ((rtime - lsync(i-1, 1)) * (lsync(i, 2) - lsync(i-1, 2)) / (lsync(i, 1) - lsync(i-1, 1))) + lsync(i-1, 2);
    end
end
if (length(rsync) > 1)
    i = 2;
    for k = 1:length(raw)
        rtime = raw(k,1);
        while (rtime > rsync(i) && length(rsync) > i)
            i = i+1;
        end
        rtimes(k) = ((rtime - rsync(i-1, 1)) * (rsync(i, 2) - rsync(i-1, 2)) / (rsync(i, 1) - rsync(i-1, 1))) + rsync(i-1, 2);
    end
end

%% various plots for moticon steps (-> uncomment)

% %% plot steps detected by Moticon software
% 
% % figure;
% % plot(times2(1,:), -acc(3,:))
% % hold on;
% % 
% % for i = 1:length(leftsteps)
% %     stime = ((leftsteps(i,2) - lp1rawtime) * (lp2realtime - lp1realtime) / (lp2rawtime - lp1rawtime)) + lp1realtime;
% %     plot([stime stime], [-20, 40], 'r');
% % end
% % 
% % for i = 1:length(leftsteps)
% %     stime = ((leftsteps(i,3) - lp1rawtime) * (lp2realtime - lp1realtime) / (lp2rawtime - lp1rawtime)) + lp1realtime;
% %     plot([stime stime], [-20, 40], 'b');
% % end
% % 
% % for i = 1:length(rightsteps)
% %     stime = ((rightsteps(i,2) - rp1rawtime) * (rp2realtime - rp1realtime) / (rp2rawtime - rp1rawtime)) + rp1realtime;
% %     plot([stime stime], [-20, 40], 'm');
% % end
% % 
% % for i = 1:length(rightsteps)
% %     stime = ((rightsteps(i,3) - rp1rawtime) * (rp2realtime - rp1realtime) / (rp2rawtime - rp1rawtime)) + rp1realtime;
% %     plot([stime stime], [-20, 40], 'c');
% % end
% 
% %% plot total forces
% 
% % figure;
% % plot(times2(1,:), -acc(3,:), 'b')
% % hold on;
% % 
% % % left total col. 18
% % plot(ltimes, raw(:,18)/100, 'r')
% % 
% % % right total col. 37
% % plot(rtimes, raw(:,37)/100, 'g')
% 
% %% add moticon steps to figure
% % for i = 1:length(leftsteps)
% %     stime = ((leftsteps(i,2) - lp1rawtime) * (lp2realtime - lp1realtime) / (lp2rawtime - lp1rawtime)) + lp1realtime;
% %     plot([stime stime], [-20, 40], 'r');
% % end
% % 
% % for i = 1:length(leftsteps)
% %     stime = ((leftsteps(i,3) - lp1rawtime) * (lp2realtime - lp1realtime) / (lp2rawtime - lp1rawtime)) + lp1realtime;
% %     plot([stime stime], [-20, 40], 'b');
% % end
% % 
% % for i = 1:length(rightsteps)
% %     stime = ((rightsteps(i,2) - rp1rawtime) * (rp2realtime - rp1realtime) / (rp2rawtime - rp1rawtime)) + rp1realtime;
% %     plot([stime stime], [-20, 40], 'm');
% % end
% % 
% % for i = 1:length(rightsteps)
% %     stime = ((rightsteps(i,3) - rp1rawtime) * (rp2realtime - rp1realtime) / (rp2rawtime - rp1rawtime)) + rp1realtime;
% %     plot([stime stime], [-20, 40], 'c');
% % end
% 
% %% plot individual sensor values with moticon steps
% 
% % figure;
% % plot(times2(1,:), -acc(3,:), 'b')
% % hold on;
% % 
% % % left toes
% % plot(ltimes, raw(:,2), 'r')
% % plot(ltimes, raw(:,3), 'r--')
% % 
% % % left heels
% % plot(ltimes, raw(:,13), 'g')
% % plot(ltimes, raw(:,14), 'g--')
% % 
% % for i = 1:length(leftsteps)
% %     stime = ((leftsteps(i,2) - lp1rawtime) * (lp2realtime - lp1realtime) / (lp2rawtime - lp1rawtime)) + lp1realtime;
% %     plot([stime stime], [-20, 40], 'r');
% % end
% % 
% % for i = 1:length(leftsteps)
% %     stime = ((leftsteps(i,3) - lp1rawtime) * (lp2realtime - lp1realtime) / (lp2rawtime - lp1rawtime)) + lp1realtime;
% %     plot([stime stime], [-20, 40], 'b');
% % end
% % 
% % 
% % % right toes
% % plot(rtimes, raw(:,2+19)-8, 'r')
% % plot(rtimes, raw(:,3+19)-8, 'r--')
% % 
% % % right heels
% % plot(rtimes, raw(:,13+19)-8, 'g')
% % plot(rtimes, raw(:,14+19)-8, 'g--')
% % 
% % for i = 1:length(rightsteps)
% %     stime = ((rightsteps(i,2) - rp1rawtime) * (rp2realtime - rp1realtime) / (rp2rawtime - rp1rawtime)) + rp1realtime;
% %     plot([stime stime], [-20, 40], 'm');
% % end
% % 
% % for i = 1:length(rightsteps)
% %     stime = ((rightsteps(i,3) - rp1rawtime) * (rp2realtime - rp1realtime) / (rp2rawtime - rp1rawtime)) + rp1realtime;
% %     plot([stime stime], [-20, 40], 'c');
% % end

%% Detect ground contact time from raw pressures

% to find out indices of all heelstrikes and toeoffs by 
% left and right foot in moticon's data  

% heel sum pressures
lheel = raw(:,13)+raw(:,14);
rheel = raw(:,13+19)+raw(:,14+19);

% % toe sum pressures
% ltoe = raw(:,2)+raw(:,3);
% rtoe = raw(:,2+19)+raw(:,3+19);
ltoe = raw(:,18);
rtoe = raw(:,37);

% plot(ltoe)
% hold on 
% plot(rtoe)

% step state, 0 = unknown, 1 = heel stoke, 2 = toe off
lstate = 0;rstate = 0;
% heelstrikes
lheelcount = 0;lheelstrikes = 0;
rheelcount = 0;rheelstrikes = 0;
% toe offs
ltoecount = 0;ltoeoffs = 0;
rtoecount = 0;rtoeoffs = 0;

% Iterate over raw data
for i = 2:length(raw)-3
    % Detect heelstrike left
    if (lstate ~= 1 && (lheel(i+1) >= 1 && lheel(i+1)-lheel(i) > 1 && lheel(i+2)-lheel(i+1) > 2.5 || lheel(i+1)-lheel(i) > 3))
%     if (lstate ~= 1 && (lheel(i-1) >= 30 && lheel(i+1)-lheel(i) >= 0 && ...
%             lheel(i+2) - lheel(i+1) >= 0 && lheel(i+3) - lheel(i+2) >= 0 && lheel(i+3) - lheel(i) >= 100))
        lstate = 1;
        lheelcount = lheelcount +1;
        lheelstrikes(lheelcount) = i;
        continue;
    end
    % Detect toe off left
    if (lstate == 1 && (ltoe(i-1)> 50 && ltoe(i-1) - ltoe(i) >= 0 && ltoe(i) - ltoe(i+1)>= 0 && ltoe(i+1) <= 50))
        lstate = 2;
        ltoecount = ltoecount +1;
        ltoeoffs(ltoecount) = i+1;
        continue;
    end 
end
% Iterate over raw data
for i = 2:length(raw)-3
    % Detect heelstrike right
     if (rstate ~= 1 && (rheel(i+1) >= 1 && rheel(i+1)-rheel(i) > 1 && rheel(i+2)-rheel(i+1) > 2.5 || rheel(i+1)-rheel(i) > 3))
%     if (rstate ~= 1 && (rheel(i-1) >= 30 && rheel(i+1) - rheel(i) >= 0 && ...
%             rheel(i+2) - rheel(i+1) >= 0 && rheel(i+3) - rheel(i+2) >= 0 && rheel(i+3) - rheel(i) >= 100))   
        rstate = 1;
        rheelcount = rheelcount +1;
        rheelstrikes(rheelcount) = i;
        continue;
     end
    
    % Detect toe off right
%     if (rstate == 1 && rheel(i+1) < 3.75 && rtoe(i+1) <= 4 && rtoe(i+2) <= 6 && rtoe(i) < rtoe(i-1)-0.3 && rtoe(i+1) < rtoe(i-1)-0.3 && abs(rtoe(i+1)-rtoe(i+2)) < 3 && (rtoe(i)-rtoe(i+1)) > (rtoe(i+1)-rtoe(i+2))*1.7 && ((rtoe(i)-rtoe(i+1)) >= 1 || (rtoe(i-1)-rtoe(i)) >= 2))
    if (rstate == 1 && (rtoe(i-1)> 50 && rtoe(i-1) - rtoe(i) >= 0 && rtoe(i) - rtoe(i+1)>= 0 && rtoe(i+1) <= 50 && rtoe(i+3) <= 40))
        rstate = 2;
        rtoecount = rtoecount +1;
        rtoeoffs(rtoecount) = i+1;
        continue;
    end    
end

%% Find ground contact times of left foot steps and right foot steps 

lsteps = []; % left foot heelstrike time, left foot toe-off time and left gct
t = 1;  % gct-> ground contact time 
for i = 1:length(lheelstrikes)
    while(t <= length(ltoeoffs))
        %raw(ltoeoffs(t),1)
        %raw(lheelstrikes(i),1)
        gct = raw(ltoeoffs(t),1) - raw(lheelstrikes(i),1);
        if (gct > 0 && gct < 2)
            stime = ltimes(ltoeoffs(t));
            lsteps = [lsteps; [stime-gct, stime, gct]];
            t = t+1;
            break;
        elseif (gct >= 2)
            break;
        elseif (gct <= 0)
            t = t+1;
        end
        %pause
    end
end

% right foot
rsteps = [];% right foot heelstrike time, right foot toe-off time and right gct
t = 1; 
for i = 1:length(rheelstrikes)
    while(t <= length(rtoeoffs))
        %raw(rtoeoffs(t),1)
        %raw(rheelstrikes(i),1)
        gct = raw(rtoeoffs(t),1) - raw(rheelstrikes(i),1);
        if (gct > 0 && gct < 2)
            stime = rtimes(rtoeoffs(t));
            rsteps = [rsteps; [stime-gct, stime, gct]];
            t = t+1;
            break;
        elseif (gct >= 2)
            break;
        elseif (gct <= 0)
            t = t+1;
        end
        %pause
    end
end

%% plot ground contact times

% to see if the steps segmentation is done perfectly or not. It can be seen
% by the plots of contact time of left foot and right foot. If it not
% correct for some steps have higher contact time (peaks) than
% nearby steps, which can be seen in plots.
%(sync jumps have higher contact time ~1s (peaks), so it should not be 
% considered as step segmentation error)    

% if it is not correct then the algo should be tweaked in lines (456, 465, 479, 488) 
% as per the slope of force curve by intuition (by increasing those differece no's). 

% figure;
% plot(lsteps(:, 1), lsteps(:, 3));
% hold on;
% plot(rsteps(:, 1), rsteps(:, 3));

figure;
plot(lsteps(:, 3));
hold on;
plot(rsteps(:, 3));

%% code to lable the data

lforce=raw(:,18)'; %left foot vertical force 
rforce=raw(:,37)'; %right foot vertical force 

% find index of foot heel strike and toe_off in INS data (TIA time). 
% lsteps contains heel strike, toe-off and their time difference 
% for left foot and same for rsteps.  

for i=1:length(lsteps)
    [tempLsVal,tempLsInd]= min(abs(times2ins - lsteps(i,1)));
    [tempLeVal,tempLeInd]= min(abs(times2ins - lsteps(i,2)));
    %technically 1/2*Fs should work instead of 1 assuming INS data is
    %present at 1/Fs time points 
    lstepinds(i).no= i;
    if (tempLsVal < 1 && tempLeVal < 1)%1s
        lstepinds(i).start= tempLsInd;
        lstepinds(i).end= tempLeInd;
    end
    %if length(lsteps) is less than length(lstepinds) then datalogger was
    % turned OFF before moticon (given, if insoles were turned ON after the
    % datalogger) **     
end
for i=1:length(rsteps)
    [tempRsVal,tempRsInd]= min(abs(times2ins - rsteps(i,1)));
    [tempReVal,tempReInd]= min(abs(times2ins - rsteps(i,2)));
    %technically 1/2*Fs should work instead of 1 assuming INS data is
    %present at 1/Fs time points 
    rstepinds(i).no= i;
    if ( tempRsVal < 1 && tempReVal < 1)
        rstepinds(i).start = tempRsInd;
        rstepinds(i).end = tempReInd;
    end
end

for i=1:length(lstepinds)
    if isempty(lstepinds(i).start)
        lstepinds(i).start = 0;
    end
    if isempty(lstepinds(i).end)
        lstepinds(i).end = 0;
    end
end
for i=1:length(rstepinds)
     if isempty(rstepinds(i).start)
        rstepinds(i).start = 0;
    end
    if isempty(rstepinds(i).end)
        rstepinds(i).end = 0;
    end
end

% initialise lables for both foot
lstepLable=zeros(1,length(insind));% if 1 then left foot is on the ground 
rstepLable=zeros(1,length(insind));% if 1 then right foot is on the ground 
% put lable=1 if the timepoint is during foot is contact with the ground. 
for i=1:length(lstepinds)
    if(lstepinds(i).start ~=0 && lstepinds(i).end ~=0)
        lstepLable(lstepinds(i).start:lstepinds(i).end)=ones(1,lstepinds(i).end-lstepinds(i).start+1);
    end
end
for i=1:length(rstepinds) 
        if(rstepinds(i).start ~=0 && rstepinds(i).end ~=0)
            rstepLable(rstepinds(i).start:rstepinds(i).end)=ones(1,rstepinds(i).end-rstepinds(i).start+1);
        end
end

doubleSupportlable=lstepLable & rstepLable; % 1, if both foot on the ground 
flightLable=not(lstepLable) & not(rstepLable); %1, if both feet in the air

% force interpolation (50Hz/100Hz --> 400Hz) to get foot force on instance
% of TIA time points of INS datalogger 
lforce_intrp=interp1(ltimes,lforce,times2ins);
rforce_intrp=interp1(rtimes,rforce,times2ins);

plot(ltimes,lforce, 'o')
hold on 
plot(times2ins,lforce_intrp,'*')

%% To select the range of velocity for further processing

%plot the figure and put two cursors for suitable data window and export
%points to worspace.    

figure
plot(times2ins, v_lon)
hold on 
plot(times2ins, acc_anatomical(3,:))

pause(0.00001);
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);

%% To extract various index for selected data range using curser points 

timepoints=size(cursor_info,2)
%%index for selected run time windows
for i = 1:timepoints/2
dataset_inds(i).rangeseq=i;
dataset_inds(i).windowStartInds=cursor_info(timepoints-2*(i-1)-0).DataIndex;
dataset_inds(i).windowEndInds=cursor_info(timepoints-2*(i-1)-1).DataIndex;
dataset_inds(i).rangeWindow_timepoints=dataset_inds(i).windowEndInds-dataset_inds(i).windowStartInds;
dataset_inds(i).windowStartTimes=cursor_info(timepoints-2*(i-1)-0).Position(1);
dataset_inds(i).windowEndTimes=cursor_info(timepoints-2*(i-1)-1).Position(1);
[~,dataset_inds(i).ltimesStartInds] = min(abs(ltimes- dataset_inds(i).windowStartTimes));
[~,dataset_inds(i).ltimesEndInds] = min(abs(ltimes- dataset_inds(i).windowEndTimes));
[~,dataset_inds(i).rtimesStartInds] = min(abs(rtimes- dataset_inds(i).windowStartTimes));
[~,dataset_inds(i).rtimesEndInds] = min(abs(rtimes- dataset_inds(i).windowEndTimes));
[~,dataset_inds(i).rstepStartInds] = min(abs(rsteps(:,1)- dataset_inds(i).windowStartTimes));
[~,dataset_inds(i).rstepEndInds] = min(abs(rsteps(:,1)- dataset_inds(i).windowEndTimes));
[~,dataset_inds(i).lstepStartInds] = min(abs(lsteps(:,1)- dataset_inds(i).windowStartTimes));
[~,dataset_inds(i).lstepEndInds] = min(abs(lsteps(:,1)-dataset_inds(i).windowEndTimes));
end

%% Run Once 

input_data={};
 target_data={};
 GRFL_strides={};
 GRFR_strides={};
 dwc=1; % data window count
 
%% Velocity Segmentation and time shifts for choosen range 

% % % % %range using curser points 
% windowStart = dataset_inds(1).windowStartInds ;% 
% windowEnd = dataset_inds(1).windowEndInds;

% % walking and then jogging 
%windowStart = 135305;
%windowEnd = 203301; 

% %walking + fast running   
%windowStart = 218000;
%windowEnd = 231000;

% % fast running + then some walking  
%windowStart = 244000;
%windowEnd = 263644;

% % % % walking 
windowStart = 272070;
windowEnd = 376847;

%% dataset for LSTM 

A_data = [lstepLable.' lstepLable.' rstepLable.' acc_ins.' ang_vel_ins.' ...
         quat.' v_lon.' V_d.' lforce_intrp.' rforce_intrp.'];
A_data1 = A_data(135305:203301, :);
A_data2 = A_data(218000:231000, :);
A_data3 = A_data(244000:263644, :);
A_data4 = A_data(272070:376847, :);

All_data = [A_data1; A_data2; A_data3; A_data4];
csvwrite('181003_testdata.csv',All_data)

%%

% %%8steps
% windowStart =270000;
% windowEnd = 272800;
% % %%12 steps
% windowStart =245000;
% windowEnd = 247000;

GrndTrack_unwrap= unwrap(GrndTrack);
GrndTrack_osc = highpass(GrndTrack_unwrap, 0.005,'StopbandAttenuation',30,'Steepness',0.7);

Fs = 400; 
V_d = highpass(V_d, 0.005,'StopbandAttenuation',30,'Steepness',0.7);
vertVel = V_d(1, windowStart:windowEnd);
vertVel_d= vertVel;

% figure
% plot(vertVel_d,':*')
% hold on

vertical_oscillations=[];
vertical_oscillations=[vertical_oscillations, cumtrapz(V_d./Fs)]; % vertical distance for complete window 
vertical_oscillations = highpass(vertical_oscillations, 0.005,'StopbandAttenuation',30,'Steepness',0.7);
vertical_oscillations=vertical_oscillations*100; % in cm 
vertical_oscillations_d= vertical_oscillations(windowStart:windowEnd);

maxV = max(vertVel_d);
thresh = 0.05 * maxV;
a = false;b = false;
front = 0;back = 0;
while (true) 
     if (abs(vertVel_d(1, 1)) > thresh || vertVel_d(1, 1) < vertVel_d(1, 2))%
        vertVel_d = vertVel_d(1, 3:end);
        front = front + 2;
        a = false;
     else
            if (GrndTrack_osc(windowStart + front + Fs/8)<0)
                a = true;
            else
                vertVel_d = vertVel_d(1, 50:end);
                front = front + 49;
            end
     end
    if (abs(vertVel_d(1, end)) > thresh || vertVel_d(1, end-1) < vertVel_d(1, end))%
        vertVel_d = vertVel_d(1, 1:end - 2);
        %vertVel_d = detrend(vertVel_d);
        back = back + 2;
        a = false;
        b = false;
    else
        b = true;
    end
    if (a && b)
        break;
    end
end
% plot(vertVel_d,':*')

        velsteppoints = [];
        if (vertVel_d(1, 1) < 0)%
            intersections = 1;
            velsteppoints = 1;
        else
            intersections = 0;
        end
        count = 0;
        points = [];
        for i = 1:length(vertVel_d) - 1
            if (vertVel_d(1, i) < 0 && vertVel_d(1, i + 1) > 0)
                intersections = intersections + 1;
                points = [points, i];
            end
            if (vertVel_d(1, i) > 0 && vertVel_d(1, i + 1) < 0  && vertical_oscillations_d(front + i) > 0.5)
                intersections = intersections + 1;
                points = [points, i];
                velsteppoints = [velsteppoints, i];%
                count = count + 1;%
            end
        end
        velstepCount = length(velsteppoints) - 1;
        disp(['Steps missed during segmentation: ', num2str(2*count-intersections-1)]);  
       
                
        e = datenum('01-jan-1970 00:00:00');
        datestr(e);
        Itime = datestr(e + (times2ins(windowStart + front)-37)/86400,'mmmm dd, yyyy HH:MM:SS.FFF AM');% UTC time for first point for first step
        Itime
        % start and end index of a step (when calculated according to the vertical velocity)
        velstepinds = struct([]);
        for i = 1:velstepCount
             velstepinds(i).start=windowStart + front + velsteppoints(i);
             velstepinds(i).end=windowStart + front + velsteppoints(i+1)-1;
        end
   
         v_lon_window = v_lon(1, windowStart:windowEnd);   
         velstepDuration = []; speed=[]; vel_lon_diff=[];vel_lon_steps=[];
         velstepLength = []; cadence = []; vertical_oscillations_window=[]; 
         vertical_oscillations_segmented=[]; vertical_oscillations_amp=[];
         vertOscillation_vel_err=[]; % error over a step

         vertical_oscillations_window=[vertical_oscillations_window, cumtrapz(vertVel./Fs)]; % vertical distance for complete selected window (with front and back) 
         vertical_oscillations_window=vertical_oscillations_window.*100;
         vertical_oscillations_segmented = vertical_oscillations_window(front:end-back-1); % without front and back
         v_dist_shift= mean(vertical_oscillations_segmented); %shift for the mid point of oscillations
         vertical_oscillations_segmented= vertical_oscillations_segmented-v_dist_shift; % shifted vertical displacement without front and back
        
         for i = 1:velstepCount
            velstepDuration = [velstepDuration, (front + velsteppoints(i + 1) - (front + velsteppoints(i)))/Fs];
            cadence = [cadence, 1/velstepDuration(i)];
            stepVelsLon = v_lon_window(front + velsteppoints(i):(front + velsteppoints(i + 1)-1));
%           stepVelsLon = v_lon(1,(velstepinds(i).start):(velstepinds(i).end));
            speed(i)=mean(stepVelsLon);
            vel_lon_diff(i)=max(stepVelsLon)-min(stepVelsLon);
            velstepLength = [velstepLength, trapz(stepVelsLon./Fs)];% longitudinal displacement in a step
            stepDispDown = vertical_oscillations_window((front + velsteppoints(i)):(front + velsteppoints(i+1)-1));
            stepDispDown_p_p= max(stepDispDown) - min(stepDispDown); 
            vertical_oscillations_amp = [vertical_oscillations_amp,stepDispDown_p_p]; % peak to peak    
            vertOscillation_vel_err = [vertOscillation_vel_err,mean(vertical_oscillations_segmented(velsteppoints(i):(velsteppoints(i + 1)-1)))];
         end  
         for i = 1:velstepCount
             contactTimeL(i)=sum(lstepLable(velstepinds(i).start:velstepinds(i).end)==1)/Fs;
             contactTimeR(i)=sum(rstepLable(velstepinds(i).start:velstepinds(i).end)==1)/Fs;
             flightTime(i)=sum(flightLable(velstepinds(i).start:velstepinds(i).end)==1)/Fs;
             doubleSupportTime(i)=sum(doubleSupportlable(velstepinds(i).start:velstepinds(i).end)==1)/Fs;
         end
            
% Stride parameters for velocity segmenattion, left foot stride and right foot stride and their comparision 
    
        %start and end points for each stride in window (when calculated according to the vertical velocity)       
        % one stride is consist of two steps
        vel_stride_points=[];
        for j=1:ceil(length(velsteppoints)/2)
            vel_stride_points=[vel_stride_points velsteppoints(2*j-1)];
        end
        velStrideCount = length(vel_stride_points) - 1;
        
        % start and end index of the strides (calculated using vertical velocity)
        velStrideinds = struct([]);
        for j = 1:velStrideCount
             velStrideinds(j).start=windowStart + front + vel_stride_points(j);
             velStrideinds(j).end=windowStart + front + vel_stride_points(j+1)-1;
        end 
        
        % To calculate stride parameters in a velocity stride
        velStrideDuration = [];  velStrideLength = []; velStridecadence=[];
        velStrideSpeed=[]; fVel_pp=[];fVel_std=[];
        velstrideDispDown = []; velstrideDispDown_p_p = [];
        vertOscillation_velstride_err= []; 
        velStride_vertDisp_pp = []; velStride_vertDisp_std = [];
         V_acc_pp=[]; F_acc_pp=[]; vVel_pp=[]; V_acc_std=[]; F_acc_std=[]; vVel_std=[];
         yaw_pp=[]; roll_pp=[]; pitch_pp=[]; yaw_std=[]; roll_std=[]; pitch_std=[];
         wx_pp=[]; wy_pp=[]; wz_pp=[]; wx_std=[]; wy_std=[]; wz_std=[];
         GrndTrack_pp=[]; GrndTrack_std=[];
         
         GCTL_stride=[];GCTR_stride=[];%GCT in stride
         FT_stride=[];%flight time 
         DST_stride=[];% double suppport time 
         TD_L=[];TO_L=[];footTimeL=[];TD_R=[];TO_R=[];footTimeR=[];
         GRFR_stride={}; GRFL_stride={}; GRFR_area=[]; GRFL_area=[]; GRFR_max=[]; GRFL_max=[];
                
        for i = 1:velStrideCount
            i
            velStrideDuration(i) = (windowStart + front + vel_stride_points(i + 1) - (windowStart + front + vel_stride_points(i)))/Fs;
            velStridecadence(i) = 1/velStrideDuration(i);
           
            velStrideSpeed(i)  = mean(v_lon(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i + 1)-1));
            fVel_pp(i)         = peak2peak(v_lon(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i + 1)-1));
            fVel_std(i)        = std(v_lon(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i + 1)-1));
            velStrideLength(i) = trapz(v_lon(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i + 1)-1)./Fs);% stride legth in direction of movement (typically 2* velsteplength)
            
            velStride_vertDisp_pp(i) =  peak2peak(vertical_oscillations_window((front + vel_stride_points(i)):(front + vel_stride_points(i+1)-1))); % peak to peak  
            velStride_vertDisp_std(i) = std(vertical_oscillations_window((front + vel_stride_points(i)):(front + vel_stride_points(i+1)-1))); %std
            
            vertOscillation_velstride_err(i) = mean(vertical_oscillations_segmented(vel_stride_points(i):(vel_stride_points(i + 1)-1))); 
            
            V_acc_pp(i) =     peak2peak(acc_anatomical(3,(windowStart + front + vel_stride_points(i)): (windowStart + front + vel_stride_points(i+1)-1)));
            F_acc_pp(i) =     peak2peak(acc_anatomical(1,(windowStart + front + vel_stride_points(i)): (windowStart + front + vel_stride_points(i+1)-1)));
            vVel_pp(i) =      peak2peak(V_d(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
            GrndTrack_pp(i) = peak2peak(GrndTrack_unwrap(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
           
            
%             yaw_pp(i) =       peak2peak(yaw_unwrap(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
%             roll_pp(i) =      peak2peak(roll(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
%             pitch_pp(i) =     peak2peak(pitch(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
%             wx_pp(i) =        peak2peak(ang_vel_ins(1, windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
%             wy_pp(i) =        peak2peak(ang_vel_ins(2, windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
%             wz_pp(i) =        peak2peak(ang_vel_ins(3, windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));

            yaw_pp(i) =       max(yaw_unwrap(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1))-...
                              min(yaw_unwrap(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
                          
            roll_pp(i) =      max(roll(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1))-...
                              min(roll(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
            
            pitch_pp(i) =     max(pitch(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1))-...
                              min(pitch(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
                          
            wx_pp(i) =        max(ang_vel_ins(1, windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1))-...
                              min(ang_vel_ins(1,windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
                          
            wy_pp(i) =        max(ang_vel_ins(2, windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1))-...
                              min(ang_vel_ins(2,windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
                          
            wz_pp(i) =        max(ang_vel_ins(3, windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1))-...
                              min(ang_vel_ins(3, windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
 

            V_acc_std(i) =     std(acc_anatomical(3,(windowStart + front + vel_stride_points(i)): (windowStart + front + vel_stride_points(i+1)-1)));
            F_acc_std(i) =     std(acc_anatomical(1,(windowStart + front + vel_stride_points(i)): (windowStart + front + vel_stride_points(i+1)-1)));
            vVel_std(i) =      std(V_d(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
            GrndTrack_std(i) = std(GrndTrack_unwrap(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
            yaw_std(i) =       std(yaw_unwrap(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
            roll_std(i) =      std(roll(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
            pitch_std(i) =     std(pitch(windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
            wx_std(i) =        std(ang_vel_ins(1, windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
            wy_std(i) =        std(ang_vel_ins(2, windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));
            wz_std(i) =        std(ang_vel_ins(3, windowStart + front + vel_stride_points(i) : windowStart + front + vel_stride_points(i+1)-1));

        end
      
         for i = 1:(velStrideCount)
                 i
                 GCTL_stride(i)=sum(lstepLable(velStrideinds(i).start:velStrideinds(i).end)==1)/Fs;
                 GCTR_stride(i)=sum(rstepLable(velStrideinds(i).start:velStrideinds(i).end)==1)/Fs;
                 FT_stride(i)=sum(flightLable(velStrideinds(i).start:velStrideinds(i).end)==1)/Fs;
                 DST_stride(i)=sum(doubleSupportlable(velStrideinds(i).start:velStrideinds(i).end)==1)/Fs;

                     footTimeL = find((lstepLable(velStrideinds(i).start : velStrideinds(i).end)==1));
                     TD_L(i) = min(footTimeL)/Fs; 
                     TO_L(i) = max(footTimeL)/Fs;%

                     %%%%%%%%%%%%%%%%%%%%******************************
                     footTimeR = find((rstepLable(velStrideinds(i).start:velStrideinds(i).end)==0));%%%%%%%%
                     TO_R(i)= min(footTimeR)/Fs;
                     TD_R(i)= max(footTimeR)/Fs;%

                     if(TO_R(i)*Fs==1)
                         footTimeR = find((rstepLable(velStrideinds(i).start:velStrideinds(i).end)==1));%%%%%%%%
                         TD_R(i)= min(footTimeR)/Fs;
                         TO_R(i)= max(footTimeR)/Fs;%
                     end
                 GRFL_stride{i} = lforce_intrp((windowStart + front + vel_stride_points(i)) : (windowStart + front + vel_stride_points(i+1)-1));
                 GRFR_stride{i} = rforce_intrp((windowStart + front + vel_stride_points(i)) : (windowStart + front + vel_stride_points(i+1)-1));

                 GRFL_area= [GRFL_area, trapz(GRFL_stride{i} ./Fs)];% 
                 GRFR_area= [GRFR_area, trapz(GRFR_stride{i} ./Fs)];% 
                 GRFL_max = [GRFL_max,  max(GRFL_stride{i})];%       
                 GRFR_max = [GRFR_max,  max(GRFR_stride{i})];%    
         end
      
% combine input / target data cells 
         input_data{dwc} =[
                 velStrideSpeed; ...
                 velStrideDuration;...
                 velStrideLength;...
                 F_acc_pp; ... 
                 V_acc_pp; ...
                 fVel_pp; ...
                 vVel_pp;...
                 velStride_vertDisp_pp;...
                 GrndTrack_pp;...
                 yaw_pp;...
                 roll_pp;...
                 pitch_pp;...
                 wx_pp;...
                 wy_pp;...
                 wz_pp;...
                 
                 F_acc_std; ... 
                 V_acc_std; ...
                 fVel_std; ...
                 vVel_std;...
                 velStride_vertDisp_std;...
                 GrndTrack_std;...
                 yaw_std;...
                 roll_std;...
                 pitch_std;...
                 wx_std;...
                 wy_std;...
                 wz_std
                 ];
                       
         target_data{1,dwc}=[ ...
                 TD_L; ...        % touch down leftfoot 
                 TO_L; ...               % toeoff leftfoot
                 GCTL_stride; ...        % GCTLF 
                 GRFL_max; ...           % GRFLF max 
                 GRFL_area; ...           % Area under GRFLF curve
                 
                 TD_R; ...               % touch down rightfoot 
                 TO_R; ...               % toeoff leftfoot 
                 GCTR_stride; ...        % GCTRF
                 GRFR_max; ...           % GRFRF max 
                 GRFR_area; ...           % Area under GRFRF curve 
                 
                 FT_stride; ...  % flight time
                 DST_stride]; % Double Support   
             
         GRFL_strides{dwc} = GRFL_stride;
         GRFR_strides{dwc} = GRFR_stride;
         dwc=dwc+1;
             
% To verify the velociry segmentation (** left foot strike before right foot**) 
         
        figure
        scatter3(velStrideSpeed,velStridecadence*60,TD_L,'filled','DisplayName','TDL (s)')
        hold on 
        scatter3(velStrideSpeed,velStridecadence*60,TO_L,'filled','DisplayName','TOL (s)')
        scatter3(velStrideSpeed,velStridecadence*60,GCTL_stride,'filled','DisplayName','GCTL (s)')
        scatter3(velStrideSpeed,velStridecadence*60,velStrideDuration,'filled','DisplayName','Stride duration (s)')
        xlabel('speed, m/s')
        ylabel('cadence /min')
        legend 
        
        figure
        scatter3(velStrideSpeed,velStridecadence*60,TD_R,'filled','DisplayName','TDR (s)')
        hold on 
        scatter3(velStrideSpeed,velStridecadence*60,TO_R,'filled','DisplayName','TOR (s)')
        scatter3(velStrideSpeed,velStridecadence*60,GCTR_stride,'filled','DisplayName','GCTR (s)')
        scatter3(velStrideSpeed,velStridecadence*60,velStrideDuration,'filled','DisplayName','Stride duration (s)')
        xlabel('speed, m/s')
        ylabel('cadence /min')
        legend 
                           
         %% All input and targte train and test data 
         
          train_input_data= [input_data{1} input_data{2} input_data{4}];
          train_target_data= [target_data{1} target_data{2} target_data{4}];
          train_GRFL_stride = [GRFL_strides{1} GRFL_strides{2} GRFL_strides{4}];
          train_GRFR_stride = [GRFR_strides{1} GRFR_strides{2} GRFR_strides{4}];
%          
          test_input_data= [input_data{3}];
          test_target_data= [target_data{3}];
          test_GRFL_stride = [GRFL_strides{3}];
          test_GRFR_stride = [GRFR_strides{3}];
         
         test_input_data_s2= [input_data{1} input_data{2} input_data{3} input_data{4}];
         test_target_data_s2= [target_data{1} target_data{2} target_data{3} target_data{4}];
         test_GRFL_stride_s2 = [GRFL_strides{1} GRFL_strides{2} GRFL_strides{3} GRFL_strides{4}];
         test_GRFR_stride_s2 = [GRFR_strides{1} GRFR_strides{2} GRFR_strides{3} GRFR_strides{4}];
         save('181003_KNN.mat','test_input_data_s2','test_target_data_s2','test_GRFL_stride_s2','test_GRFR_stride_s2')
         
         %% correlation matrix for feature selection (among inputs and input/outputs)

        figure
      % matrix and axis lables for all input features  
        train_feature_table = [train_input_data];    
        axislabels = {'speed','stride duration','stride length','forward acceleration(p-p)','vertical acceleration(p-p)','speed (p-p)',...
            'vertical velocity(p-p)','vertical displacement(p-p)','ground track(p-p)','yaw(p-p)','roll(p-p)','pitch(p-p)','wx(p-p)','wy(p-p)','wz(p-p)',...
            'forward acceleration SD','vertical acceleration SD','speed SD',...
            'vertical velocity SD','vertical displacement SD','ground track SD','yaw SD','roll SD','pitch SD','wx SD','wy SD','wz SD'};
        
        % matrix and axis lables for selected input features  
        train_feature_table = [train_input_data([1 2 4 5 6 8 9 18],:)];   
        axislabels = {'speed','stride duration','forward acceleration(p-p)','vertical acceleration(p-p)','speed (p-p)',...
            'vertical displacement(p-p)','ground track(p-p)','speed SD'};

        % matrix and axis lables for selected input and output (combined) features  
        train_feature_table = [train_input_data([1 2 4 5 6 8 9 18],:); train_target_data];   
        axislabels = {'speed','stride duration','forward acceleration(p-p)','vertical acceleration(p-p)','speed (p-p)',...
            'vertical displacement(p-p)','ground track(p-p)','speed SD','TDL','TOL','GCTL','vGRFL peak','impulse left-foot',...
             'TDR','TOR','GCTR','vGRFR peak','impulse right-foot','flight time','double support time'};
        
        [R,PValue] = corr(train_feature_table.'); % Spearman
        R =  abs(R); NC=10;
        newmap = (jet(NC));    
        colormap(newmap);      
        im = imagesc(R);

        h = colorbar(); cl = caxis;
        set(h,'Direction','reverse','FontSize',12,'Box','off','FontWeight','bold',...
            'TickDirection', 'out', 'ytick',linspace(cl(1),cl(2),NC+1),...
            'TickLabels',{'0','0.1','0.2','0.3','0.4','0.5','0.6','0.7','0.8','0.9','1'})
                                          
        xticks = linspace(1, size(R, 2), numel(axislabels));
        yticks = linspace(1, size(R, 2), numel(axislabels));
        set(gca,'box','off','color','none', 'TickDir','out', 'XTick', xticks, 'YTick', yticks, 'XTickLabel', axislabels, ...
            'YTickLabel', axislabels, 'XTickLabelRotation', 90);
        pbaspect([1 1 1])
        
        xlims = get(gca,'XLim'); ylims = get(gca,'YLim');
        hold on
        for i=ylims(1):ylims(2)
            plot(xlims, [i i], 'w' ,'LineWidth',1)
        end
        for i=xlims(1):xlims(2)
            plot([i i], ylims,'w' ,'LineWidth',1)
        end
        grid on
        
        
%         % To color yticks of all input corr colormap
%         ticklabels = get(gca,'YTickLabel');
%         ticklabels_new = cell(size(ticklabels));
%         for i = 1:length(ticklabels)
%             ticklabels_new{i} = ['\color{black} ' ticklabels{i}];
%             if i == 1 || i == 2 || i == 4 || i == 5 || i == 6 || i == 9 || i == 8 || i == 18
%                 ticklabels_new{i} = ['\color{darkGreen} \bf' ticklabels{i}];
%             end
%         end 

%         % set the tick labels
%         set(gca, 'YTickLabel', ticklabels_new, 'XTickLabel', ticklabels_new);

        %To bold selected yticks of all inputoutput corr colormap
        ticklabels = get(gca,'YTickLabel');
        ticklabels_new = cell(size(ticklabels));
        for i = 1:length(ticklabels)
            ticklabels_new{i} = ['\color{black}' ticklabels{i}];
            if i ~= 1 && i ~= 2 && i ~= 3 && i ~= 4 && i ~= 5 && i ~= 6 && i ~= 7 && i ~= 8
                ticklabels_new{i} = ['\color{black} \bf' ticklabels{i}];
            end
            if i == 5 || i == 7
                ticklabels_new{i} = ['\color{gray} ' ticklabels{i}];
            end
        end 
        % set the tick labels
        set(gca, 'YTickLabel', ticklabels_new, 'XTickLabel', ticklabels_new);

         %% filter input features (keep only 6 relavent features) ( No need for 27 or PCA 
         
         train_input_data = train_input_data([1 2 4 5 8 18],:);
         test_input_data  = test_input_data([1 2 4 5 8 18],:);
         
         %% training bagged ensemble  
         
         train_input_target=[train_input_data; train_target_data(1,:)];
         [trainedModel_TD_L, validationRMSE_TD_L] = trainRegressionModel_EBT_6(train_input_target)

         train_input_target=[train_input_data; train_target_data(2,:)];
         [trainedModel_TO_L, validationRMSE_TO_L] = trainRegressionModel_EBT_6(train_input_target)
         
         train_input_target=[train_input_data; train_target_data(3,:)];
         [trainedModel_GCTL, validationRMSE_GCTL] = trainRegressionModel_EBT_6(train_input_target)
         
         train_input_target=[train_input_data; train_target_data(4,:)];
         [trainedModel_GRFL_max, validationRMSE_GRFL_max] = trainRegressionModel_EBT_6(train_input_target)
         
         train_input_target=[train_input_data; train_target_data(5,:)];
         [trainedModel_GRFL_area, validationRMSE_GRFL_area] = trainRegressionModel_EBT_6(train_input_target)
         
         train_input_target=[train_input_data; train_target_data(6,:)];
         [trainedModel_TD_R, validationRMSE_TD_R] = trainRegressionModel_EBT_6(train_input_target)
         
         train_input_target=[train_input_data; train_target_data(7,:)];
         [trainedModel_TO_R, validationRMSE_TO_R] = trainRegressionModel_EBT_6(train_input_target)
         
         train_input_target=[train_input_data; train_target_data(8,:)];
         [trainedModel_GCTR, validationRMSE_GCTR] = trainRegressionModel_EBT_6(train_input_target)
         
         train_input_target=[train_input_data; train_target_data(9,:)];
         [trainedModel_GRFR_max, validationRMSE_GRFR_max] = trainRegressionModel_EBT_6(train_input_target)
         
         train_input_target=[train_input_data; train_target_data(10,:)];
         [trainedModel_GRFR_area, validationRMSE_GRFR_area] = trainRegressionModel_EBT_6(train_input_target)
                  
         train_input_target=[train_input_data; train_target_data(11,:)];
         [trainedModel_FT_stride, validationRMSE_FT_stride] = trainRegressionModel_EBT_6(train_input_target)
         
         train_input_target=[train_input_data; train_target_data(12,:)];
         [trainedModel_DST_stride, validationRMSE_DST_stride] = trainRegressionModel_EBT_6(train_input_target)
         
         %% foot feature (12) predictions train and test  
         
         %traing moticon data predictions 
         train_target_pred=[];
         train_target_pred(1,:)=trainedModel_TD_L.predictFcn(train_input_data); 
         train_target_pred(2,:)=trainedModel_TO_L.predictFcn(train_input_data); 
         train_target_pred(3,:)=trainedModel_GCTL.predictFcn(train_input_data); 
         train_target_pred(4,:)=trainedModel_GRFL_max.predictFcn(train_input_data); 
         train_target_pred(5,:)=trainedModel_GRFL_area.predictFcn(train_input_data); 
         train_target_pred(6,:)=trainedModel_TD_R.predictFcn(train_input_data); 
         train_target_pred(7,:)=trainedModel_TO_R.predictFcn(train_input_data); 
         train_target_pred(8,:)=trainedModel_GCTR.predictFcn(train_input_data); 
         train_target_pred(9,:)=trainedModel_GRFR_max.predictFcn(train_input_data); 
         train_target_pred(10,:)=trainedModel_GRFR_area.predictFcn(train_input_data); 
         train_target_pred(11,:)=trainedModel_FT_stride.predictFcn(train_input_data); 
         train_target_pred(12,:)=trainedModel_DST_stride.predictFcn(train_input_data); 
         train_NRMSE=[]; train_RMSE=[];
         for i=1:12
                 train_RMSE = [train_RMSE sqrt(mean((double(train_target_pred(i,:)) - double(train_target_data(i,:))).^2))];
                 train_NRMSE = [train_NRMSE sqrt(mean((double(train_target_pred(i,:)) - double(train_target_data(i,:))).^2)).*100 ...
                                   /max(train_target_data(i,:))];
         end
         
         %test moticon data predictions 
         test_target_pred=[];
         test_target_pred(1,:)=trainedModel_TD_L.predictFcn(test_input_data); 
         test_target_pred(2,:)=trainedModel_TO_L.predictFcn(test_input_data); 
         test_target_pred(3,:)=trainedModel_GCTL.predictFcn(test_input_data); 
         test_target_pred(4,:)=trainedModel_GRFL_max.predictFcn(test_input_data); 
         test_target_pred(5,:)=trainedModel_GRFL_area.predictFcn(test_input_data); 
         test_target_pred(6,:)=trainedModel_TD_R.predictFcn(test_input_data); 
         test_target_pred(7,:)=trainedModel_TO_R.predictFcn(test_input_data); 
         test_target_pred(8,:)=trainedModel_GCTR.predictFcn(test_input_data); 
         test_target_pred(9,:)=trainedModel_GRFR_max.predictFcn(test_input_data); 
         test_target_pred(10,:)=trainedModel_GRFR_area.predictFcn(test_input_data); 
         test_target_pred(11,:)=trainedModel_FT_stride.predictFcn(test_input_data); 
         test_target_pred(12,:)=trainedModel_DST_stride.predictFcn(test_input_data); 
         test_NRMSE=[]; test_RMSE=[];
         for i=1:12
             test_RMSE = [test_RMSE sqrt(mean((double(test_target_data(i,:)) - double(test_target_pred(i,:))).^2))];
             test_NRMSE = [test_NRMSE sqrt(mean((double(test_target_data(i,:)) - double(test_target_pred(i,:))).^2)).*100 ...
                               / max(test_target_data(i,:))];
         end  
         
%          train_NRMSE_FS27 = train_NRMSE
%          test_NRMSE_FS27  = test_NRMSE
         
%            train_NRMSE_FS6 = train_NRMSE
%            test_NRMSE_FS6  = test_NRMSE

%          train_NRMSE_PCA = train_NRMSE
%          test_NRMSE_PCA  = test_NRMSE
         
         %% select regresssion NRMSE's and plot Bar graphs 
         
         train_NRMSE = [train_NRMSE_FS6; train_NRMSE_PCA; train_NRMSE_FS27];
         test_NRMSE = [test_NRMSE_FS6; test_NRMSE_PCA; test_NRMSE_FS27];
         save('NRMSE_train_test.mat','train_NRMSE','test_NRMSE')
         
         train_NRMSE = train_NRMSE(:,[1 2 3 4 5 6 8 9 10 11 12]);
         test_NRMSE  = test_NRMSE(:,[1 2 3 4 5 6 8 9 10 11 12]);
         
         str_bar = {'TDL','TOL','GCTL','vGRFL\_peak','Impulse\_L',...
                        'TDR','GCTR','vGRFR\_peak','Impulse\_R','flight time', 'double support'};
         
         figure 
         pause(0.00001);
         frame_h = get(handle(gcf),'JavaFrame');
         set(frame_h,'Maximized',1);
         hB1 = bar(test_NRMSE.',0.9,'grouped','FaceAlpha',0.3,'EdgeColor','none');
         hT1=[];              
         for i=1:length(hB1) 
            if i==1
                i=1
                hT1=[hT1 text(hB1(i).XData+hB1(i).XOffset,hB1(i).YData,num2str(hB1(i).YData.','%.1f'), ...
                            'FontWeight','bold','VerticalAlignment','bottom','horizontalalign','right')];
            elseif i==2
                hT1=[hT1 text(hB1(i).XData+hB1(i).XOffset,hB1(i).YData,num2str(hB1(i).YData.','%.1f'), ...
                            'FontWeight','bold','VerticalAlignment','bottom','horizontalalign','center')];
            else
                 hT1=[hT1 text(hB1(i).XData+hB1(i).XOffset,hB1(i).YData,num2str(hB1(i).YData.','%.1f'), ...
                            'FontWeight','bold','VerticalAlignment','bottom','horizontalalign','left')];
            end
         end 
         hB1(1).FaceColor = 'g'; hB1(2).FaceColor = 'r'; hB1(3).FaceColor = 'b';

         hold on 
         hB2=bar(train_NRMSE.',0.9,'grouped','FaceAlpha',0.8,'EdgeColor','none');
         hT2=[];      
         pause(2)
         for i=1:length(hB2) 
            hT2=[hT2 text(hB2(i).XData+hB2(i).XOffset,hB2(i).YData,num2str(hB2(i).YData.','%.1f'), ...
                                  'VerticalAlignment','bottom','horizontalalign','center')];
         end 
         hB2(1).FaceColor = 'g'; hB2(2).FaceColor = 'r'; hB2(3).FaceColor = 'b';
         
         set(gca,'box','off','fontsize',16,'TickDir','out','FontSize',12,'XTickLabel',str_bar,'XTickLabelRotation',22.5) 
         barLegend = legend({'test error, 6 features','test error, PCA','test error, all features','train error, 6 features',...
             'train error, PCA','train error, all features'},'location','best','box','off','Position',[0.29 0.7 0.1 0.1]);
         barLegend.NumColumns=2;
         box off
         ylabel('NRMSE (%)')
         xlabel('target parameters')

         %% 3D stride plots and surf 
         
        % normalise GRFL strides train 
        P=50;
        train_GRFLs=zeros(length(train_GRFL_stride),P); 
        train_GRFL_stride_foot={}
         for i=1:length(train_GRFL_stride)
            i
            temp = train_GRFL_stride{i}(train_target_data(1,i)*Fs : train_target_data(2,i)*Fs);
            train_GRFL_stride_foot{i}= temp ;
         end
        for i=1:length(train_GRFL_stride)
            seq1= 1:1:length(train_GRFL_stride_foot{i});
            seq2= 1:length(train_GRFL_stride_foot{i})/P:length(train_GRFL_stride_foot{i});
            train_GRFLs(i,:) = interp1(seq1,train_GRFL_stride_foot{i},seq2,'linear');
        end
        
        %surf 
        figure
        a = train_input_data(1,:);
        [X,Y] = meshgrid(1:1:50,a);
        surf(X,Y,train_GRFLs)
        az = -45;el = 11;
        view(az, el);
        ax=gca;
        set(ax,'XTickLabels',{'0','0.2','0.4','0.6','0.8','1'})        
        ylabel('speed, m/s')
        zlabel('vGRFL, N')
        h = colorbar(); cl = caxis;
        set(h,'FontSize',12,'Box','off','FontWeight','bold',...
            'TickDirection', 'out', 'ytick',round(linspace(0,1800,10)))
        pbaspect([1 1 1])
        xlim([0 50])
        
        % normalise GRFL strides test 
        P=50;
        test_GRFLs=zeros(length(test_GRFL_stride),P); 
        test_GRFL_stride_foot={}
         for i=1:length(test_GRFL_stride)
            i
            temp = test_GRFL_stride{i}(test_target_data(1,i)*Fs : test_target_data(2,i)*Fs);
            test_GRFL_stride_foot{i}= temp ;
         end
        for i=1:length(test_GRFL_stride)
            seq1= 1:1:length(test_GRFL_stride_foot{i});
            seq2= 1:length(test_GRFL_stride_foot{i})/P:length(test_GRFL_stride_foot{i});
            test_GRFLs(i,:) = interp1(seq1,test_GRFL_stride_foot{i},seq2,'linear');
        end
        
        %surf test
        figure
        a = test_input_data(1,:);
        [X,Y] = meshgrid(1:1:50,a);
        surf(X,Y,test_GRFLs)
        az = -45;el = 11;
        view(az, el);
        ax=gca;
        set(ax,'XTickLabels',{'0','0.2','0.4','0.6','0.8','1'})        
        ylabel('speed, m/s')
        zlabel('vGRFL, N')
        h = colorbar(); cl = caxis;
        set(h,'FontSize',12,'Box','off','FontWeight','bold',...
            'TickDirection', 'out', 'ytick',round(linspace(0,1800,10)))
        pbaspect([1 1 1])
        xlim([0 50])
        
        %test data 2
        
        % normalise GRFL strides test 
        P=50;
        test2_GRFLs=zeros(length(test_GRFL_stride_s2),P); 
        test2_GRFL_stride_foot={}
         for i=1:length(test_GRFL_stride_s2)
            i
            temp = test_GRFL_stride_s2{i}(test_target_data_s2(1,i)*Fs : test_target_data_s2(2,i)*Fs);
            test2_GRFL_stride_foot{i}= temp ;
         end
        for i=1:length(test_GRFL_stride_s2)
            seq1= 1:1:length(test2_GRFL_stride_foot{i});
            seq2= 1:length(test2_GRFL_stride_foot{i})/P:length(test2_GRFL_stride_foot{i});
            test2_GRFLs(i,:) = interp1(seq1,test2_GRFL_stride_foot{i},seq2,'linear');
        end

        
        %surf test
        figure
        a = test_input_data_s2(1,:);
        [X,Y] = meshgrid(1:1:50,a);
        surf(X,Y,test2_GRFLs)
        az = -45;el = 11;
        view(az, el);
        ax=gca;
        set(ax,'XTickLabels',{'0','0.2','0.4','0.6','0.8','1'})        
        ylabel('speed, m/s')
        zlabel('vGRFL, N')
        h = colorbar(); cl = caxis;
        set(h,'FontSize',12,'Box','off','FontWeight','bold',...
            'TickDirection', 'out', 'ytick',round(linspace(0,1800,10)))
        pbaspect([1 1 1])
        xlim([0 50])
        
         %% Find peaks(foot landing) vs speed for training and test data
         
         % subplot 1 (traing data)
         GRF_peak_count =[];
         pks={};
         for i=1:length(train_GRFLs)
              [pks{i},locs,w,p] = findpeaks(train_GRFLs(i,:),'MinPeakDistance',10);
              GRF_peak_count(i)=length(pks{i});
         end
         GRF_peak_count(find(GRF_peak_count==3))=2;         
         indx_1peak = find(GRF_peak_count==1); % speed walk + running 
         indx_2peak = find(GRF_peak_count==2); % walking + speed walking 
         minvel_1peak = min(train_input_data(1,indx_1peak));
         maxvel_2peak = max(train_input_data(1,indx_2peak));  
         minvel_2peak = min(train_input_data(1,indx_2peak));  
         maxvel_peaks = max(train_input_data(1,:));
         
         subplot(1,3,1)
         hp1 = histogram(train_input_data(1,indx_1peak));
         hp1.BinEdges = [minvel_1peak maxvel_2peak 4 5 maxvel_peaks];
         text((hp1.BinEdges(1:end-1)+hp1.BinEdges(2:end))/2,hp1.Values,num2str(hp1.Values'),'vert','bottom','horiz','center'); 
         box off, hold on 
         hp2 = histogram(train_input_data(1,indx_2peak), 'EdgeColor', 'w' );
         hp2.BinEdges = [minvel_2peak:0.001:maxvel_2peak];
         hp2.NumBins = 8;
         text((hp2.BinEdges(1:end-1)+hp2.BinEdges(2:end))/2,hp2.Values,num2str(hp2.Values'),'vert','bottom','horiz','center'); 
         box off, ax=gca;
         set(ax,'box','off','TickDir','out','FontSize',12);       
         legend({'single GRF peak (full/mid/fore foot landing)',...
            'double GRF peak (rear foot landing)'},'box','off','Location','northoutside')
         box off
         ylabel('GRFL stride count'), xlabel('speed, m/s')
         title ('Training dataset, subject 1')   
        
         % subplot 2 (test data)
         GRF_peak_count =[];
         pks={};
         for i=1:length(test_GRFLs)
              [pks{i},locs,w,p] = findpeaks(test_GRFLs(i,:),'MinPeakDistance',10);
              GRF_peak_count(i)=length(pks{i});
         end
         GRF_peak_count(find(GRF_peak_count==3))=2;         
         indx_1peak = find(GRF_peak_count==1); % speed walk + running 
         indx_2peak = find(GRF_peak_count==2); % walking + speed walking 
         minvel_1peak = min(test_input_data(1,indx_1peak));
         maxvel_2peak = max(test_input_data(1,indx_2peak));  
         minvel_2peak = min(test_input_data(1,indx_2peak));  
         maxvel_peaks = max(test_input_data(1,:));
         
         subplot(1,3,2)
         hp1 = histogram(test_input_data(1,indx_1peak) );
         hp1.BinEdges = [minvel_1peak maxvel_2peak 4 5 maxvel_peaks];
         text((hp1.BinEdges(1:end-1)+hp1.BinEdges(2:end))/2,hp1.Values,num2str(hp1.Values'),'vert','bottom','horiz','center'); 
         box off, hold on 
         hp2 = histogram(test_input_data(1,indx_2peak), 'EdgeColor', 'w' );
         hp2.BinEdges = [minvel_2peak:0.001:maxvel_2peak];
         hp2.NumBins = 8;
         text((hp2.BinEdges(1:end-1)+hp2.BinEdges(2:end))/2,hp2.Values,num2str(hp2.Values'),'vert','bottom','horiz','center'); 
         box off, ax=gca;
         set(ax,'box','off','TickDir','out','FontSize',12);       
         legend({'single GRF peak (full/mid/fore foot landing)',...
            'double GRF peak (rear foot landing)'},'box','off','Location','northoutside')
         box off
         ylabel('GRFL stride count'), xlabel('speed, m/s')
         title ('Test dataset 1, subject 1')   
         
         
         %%
         
         train_featuresLF = [train_target_data(1:3,:); train_input_data(2,:)/3];
         test_featuresLF  = [test_target_pred(1:3,:);  test_input_data(2,:)/3];

         train_featuresRF= [train_target_data(8,:); train_input_data(2,:)];
         test_featuresRF= [test_target_pred(8,:); test_input_data(2,:)];
         
          %% plot train/test prediction resgression results 

         for r=1:12
              figure
              pause(2);
              frame_h = get(handle(gcf),'JavaFrame');
              set(frame_h,'Maximized',1);
             plot(train_target_data(r,:))
             hold on 
             plot(train_target_pred(r,:)) 
          end 

         % plot test pred data 
         
          for r=1:12
              figure
              pause(2);
              frame_h = get(handle(gcf),'JavaFrame');
              set(frame_h,'Maximized',1); 
             plot(test_target_data(r,:))
             hold on 
             plot(test_target_pred(r,:))
          end 
            
         train_RMSEs_rg=[];train_NRMSEs_rg=[];test_RMSEs_rg=[];test_NRMSEs_rg=[];
         for r=1:12
%             train_RMSEs_rg(r)  = sqrt(mean((train_target_pred(r,:) - train_target_data(r,:)).^2))
%             train_NRMSEs_rg(r) = sqrt(mean((train_target_pred(r,:) - train_target_data(r,:)).^2))/max(train_target_data(r,:))            
            test_RMSEs_rg(r)   = sqrt(mean((test_target_pred(r,:) - test_target_data(r,:)).^2));
            test_NRMSEs_rg(r)  = sqrt(mean((test_target_pred(r,:) - test_target_data(r,:)).^2))/max(test_target_data(r,:));
         end 
         test_RMSEs_rg
         test_NRMSEs_rg
          
%% NN
%             % left foot GRFL
%             predIndxNN_LF=[];
%             for i=1:length(test_featuresLF)
%                 predIndxNN_LF=[predIndxNN_LF, time_NN(test_featuresLF(:,i),train_featuresLF)];
%                 i
%             end
%             test_GRFL_stride_predNN={};
%             for i=1:length(test_featuresLF) 
%                 i
%                 seq1= 1:1:length(test_GRFL_stride{i});
%                 seq2= 1:1:length(train_GRFL_stride{predIndxNN_LF(i)});
%                 test_GRFL_stride_pred0{i}= train_GRFL_stride{predIndxNN_LF(i)};
%                 test_GRFL_stride_predNN{i} = interp1(seq2,test_GRFL_stride_pred0{i} ,seq1,'linear','extrap');
%             end
%             % right foot GRF
%             predIndxNN_RF=[];
%             for i=1:length(test_featuresRF)
%                 predIndxNN_RF=[predIndxNN_RF,time_NN(test_featuresRF(:,i),train_featuresRF)];
%                 i
%             end
%             test_GRFR_stride_predNN={};
%             for i=1:length(test_featuresRF) 
%                 i
%                 seq1= 1:1:length(test_GRFR_stride{i});
%                 seq2= 1:1:length(train_GRFR_stride{predIndxNN_RF(i)});
%                 test_GRFR_stride_pred0{i}= train_GRFR_stride{predIndxNN_RF(i)};
%                 test_GRFR_stride_predNN{i} = interp1(seq2,test_GRFR_stride_pred0{i} ,seq1,'linear','extrap');
%             end

%% KNN
            % left foot 
            predIndxKNN_LF=[];KNN=8;
            for i=1:length(test_featuresLF)
                predIndxKNN_LF=[predIndxKNN_LF; time_KNN(test_featuresLF(:,i),train_featuresLF,KNN)];
                i;
            end
            test_GRFL_stride_pred={};test_GRFL_stride_predKNN={};
            for i=1:length(test_featuresLF) 
                i
                for j=1:KNN  
                seq1= 1:1:length(test_GRFL_stride{i});
                seq2= 1:1:length(train_GRFL_stride{predIndxKNN_LF(i,j)});
                test_GRFL_stride_pred{i,j} = interp1(seq2,train_GRFL_stride{predIndxKNN_LF(i,j)},seq1,'linear','extrap');
                end
                
                test_GRFL_stride_predKNN{i} =test_GRFL_stride_pred{i,1};
                for j=2:KNN  
                test_GRFL_stride_predKNN{i} = test_GRFL_stride_predKNN{i}+ test_GRFL_stride_pred{i,j};
                end
                test_GRFL_stride_predKNN{i}= test_GRFL_stride_predKNN{i}./KNN; 
            end
            
            % right foot 
            predIndxKNN_RF=[];KNN=2;
            for i=1:length(test_featuresRF)
                predIndxKNN_RF=[predIndxKNN_RF; time_KNN(test_featuresRF(:,i),train_featuresRF,KNN)];
                i;
            end
            test_GRFR_stride_pred={};test_GRFR_stride_predKNN={};
            for i=1:length(test_featuresRF) 
                i
                for j=1:KNN  
                seq1= 1:1:length(test_GRFR_stride{i});
                seq2= 1:1:length(train_GRFR_stride{predIndxKNN_RF(i,j)});
                test_GRFR_stride_pred{i,j} = interp1(seq2,train_GRFR_stride{predIndxKNN_RF(i,j)},seq1,'linear','extrap');
                end
                
                test_GRFR_stride_predKNN{i} =test_GRFR_stride_pred{i,1};
                for j=2:KNN  
                test_GRFR_stride_predKNN{i} = test_GRFR_stride_predKNN{i}+ test_GRFR_stride_pred{i,j};
                end
                test_GRFR_stride_predKNN{i}= test_GRFR_stride_predKNN{i}./KNN; 
            end

        %% left foot NN/KNN pred
            
                test_GRFL_stride_mat = cell2mat(test_GRFL_stride);
                test_GRFL_stride_mat(isnan(test_GRFL_stride_mat))=0;
                test_GRFL_stride_pred_mat = cell2mat(test_GRFL_stride_predKNN); %%%%%%%%
                test_GRFL_stride_pred_mat(isnan(test_GRFL_stride_pred_mat))=0;
                
                plot(test_GRFL_stride_mat)
                hold on 
                plot(test_GRFL_stride_pred_mat)
                legend('true GRF','predicted GRF')
                
%                 RMSE = sqrt(mean((double(test_GRFL_stride_mat(1:41000)) - double(test_GRFL_stride_pred_mat(1:41000))).^2))/max(test_GRFL_stride_mat(1:41000))
%                 RMSE = sqrt(mean((double(test_GRFL_stride_mat(50000:80000)) - double(test_GRFL_stride_pred_mat(50000:80000))).^2))/max(test_GRFL_stride_mat(50000:80000))
%                 RMSE = sqrt(mean((double(test_GRFL_stride_mat(85000:end)) - double(test_GRFL_stride_pred_mat(85000:end))).^2))/max(test_GRFL_stride_mat(85000:end))
                RMSE = sqrt(mean((double(test_GRFL_stride_mat) - double(test_GRFL_stride_pred_mat)).^2))/max(test_GRFL_stride_mat)
               
        %% right foot NN/KNN pred     
            
                test_GRFR_stride_mat = cell2mat(test_GRFR_stride);
                test_GRFR_stride_mat(isnan(test_GRFR_stride_mat))=0;
                test_GRFR_stride_pred_mat = cell2mat(test_GRFR_stride_predKNN); %%%%%%%%%%%%%%%
                test_GRFR_stride_pred_mat(isnan(test_GRFR_stride_pred_mat))=0;
                
                figure
                plot(test_GRFR_stride_mat)
                hold on 
                plot(test_GRFR_stride_pred_mat)
                legend('true GRF','predicted GRF')
                
%               RMSE = sqrt(mean((double(test_GRFR_stride_mat(1:41000)) - double(test_GRFR_stride_pred_mat(1:41000))).^2))/max(test_GRFR_stride_mat(1:41000))
%               RMSE = sqrt(mean((double(test_GRFR_stride_mat(50000:80000)) - double(test_GRFR_stride_pred_mat(50000:80000))).^2))/max(test_GRFR_stride_mat(50000:80000))
%               RMSE = sqrt(mean((double(test_GRFR_stride_mat(85000:end)) - double(test_GRFR_stride_pred_mat(85000:end))).^2))/max(test_GRFR_stride_mat(85000:end))
                RMSE = sqrt(mean((double(test_GRFR_stride_mat) - double(test_GRFR_stride_pred_mat)).^2))/max(test_GRFR_stride_mat)
                               
        %% 04 strides (walking) vertical Velocity and foot force 
          x = (windowStart + front :1: windowEnd - back).*(1/Fs);
          fig8 = figure
            
          pause(0.00001);
          frame_h = get(handle(gcf),'JavaFrame');
          set(frame_h,'Maximized',1);
          
          hold on
          colorheight=3; % for ylim
        
%         %left
%         y=lstepLable(windowStart + front:windowEnd - back);
%         hl = area(x,y.*1.8);
%         hl.EdgeColor=[1 1 1];
%         hl.FaceColor = [0 1 1];
%         hl.FaceAlpha = 0.2;
%         %right
%         y=rstepLable(windowStart + front:windowEnd - back);
%         hr = area(x,y.*1.8);
%         hr.EdgeColor=[1 1 1];
%         hr.FaceColor = [1 0 0];
%         hr.FaceAlpha = 0.2;
          
        %only left
        y=xor(lstepLable(windowStart + front:windowEnd - back),doubleSupportlable(windowStart + front:windowEnd - back));
        hl = area(x,y.*colorheight);
        hl.EdgeColor=[1 1 1];
        hl.FaceColor = [0 1 0];
        hl.FaceAlpha = 0.2;
        
        %only righht
        y=xor(rstepLable(windowStart + front:windowEnd - back),doubleSupportlable(windowStart + front:windowEnd - back));
        hr = area(x,y.*colorheight);
        hr.EdgeColor=[1 1 1];
        hr.FaceColor = [1 0 0];
        hr.FaceAlpha = 0.2;
        
        %flight
        y=flightLable(windowStart + front:windowEnd - back);
        hf = area(x,y.*colorheight);
        hf.EdgeColor=[1 1 1];
        hf.FaceColor = [0.5 0.5 0.5];
        hf.FaceAlpha = 0.2;
        
        %double support 
        y=doubleSupportlable(windowStart + front:windowEnd - back);
        hd = area(x,y.*colorheight);
        hd.EdgeColor=[1 1 1];
        hd.FaceColor = [1 1 0];
        hd.FaceAlpha = 0.2;
        
        yyaxis left   
        ax = gca;
        ax.YColor = 'k';
        ylims = [-5, colorheight];
        ylim(ylims)
        hv=plot(x,vertVel_d,'Color',[0.5 0.8 0.8],'LineWidth',1); %velocity window has front and back removed
        set(gca,'ytick',-1:1:2)      
%       hgt=plot(x,roll_anatomical_osc(windowStart + front:windowEnd - back),'Color',[0.7410 0.4470 0],'LineWidth',1); 

        xlabel('time (s)');
        ylabel('vertical velocity, m/s')   
%       title('step segmentation using vertical velocity')
        
%       ylims = [-1.2, colorheight];
%       ylim(ylims) 
        xlims = [x(1) x(end)];
        xlim(xlims)
        
        ylims = [-5*0.70, colorheight];
        for i = 1:1:length(vel_stride_points)
           hs(i)=plot([(windowStart + front + vel_stride_points(i))/Fs (windowStart + front+ vel_stride_points(i))/Fs], ylims, 'b-.');
        end
        
        %set(gca,'fontsize',16);
        set(gca,'fontsize',16,'box','off','TickDir','out','xtick',ceil(xlims(1)):floor(xlims(2)));

        yyaxis right
        ax = gca;
        ax.YColor  = 'k';
        ylims = [-5, colorheight];
        ylim(ylims)
        ylabel('vGRF (kN)');
        set(gca,'ytick',0:1:2)
                
        hrf=plot(x,rforce_intrp(windowStart + front:windowEnd - back)./1000,'LineWidth',1,'Color','r','LineStyle','-');
        hlf=plot(x,lforce_intrp(windowStart + front:windowEnd - back)./1000,'LineWidth',1,'Color',[0 0.6 0],'LineStyle','-');
             
%         lgd=legend([hl hlf hr hrf hd hf hv hs(1)],{'left foot contact','left foot force (kN)','right foot contact','right foot force (kN)', ...
%             'double support','flight time','vertical velocity (m/s)','beginning of the stride'},'location','southeast','EdgeColor',[1 1 1],'FontSize',10);
%         lgd.NumColumns = 4;
%         legend boxoff
           
parameters=5;
s = cell(velStrideCount,parameters);
str = cell(1,parameters);

parameters_foot=12-3;
s_foot = cell(velStrideCount,parameters_foot);
str_foot = cell(1,parameters_foot);

for i=1:velStrideCount
    i
        str{1} = strcat('speed',"  ", num2str(round(velStrideSpeed(i),2)), ' m/s ,',"   ",'speed SD',"  ", num2str(round(fVel_std(i),2)),' m/s');
        str{2} = strcat('stride duration',"  ", num2str(round(velStrideDuration(i),2)), ' s');
        str{3} = strcat('vertical disp.(p-p)',"  ", num2str(round(velStride_vertDisp_pp(i),2)),' cm');
        str{4} = strcat('forward acc. (p-p)',"  ", num2str(round(F_acc_pp(i),2)),' m/s^2');
        str{5} = strcat('vertical acc. (p-p)',"  ", num2str(round(V_acc_pp(i),2)), ' m/s^2');
        s(i,:)=str;
        
        str_foot{1} = strcat('TDL',"  ", num2str(round(TD_L(i),2)), ' s',"          ", 'TOR',"  ", num2str(round(TO_R(i),2)), ' s');
        str_foot{2} = strcat('TOL',"  ", num2str(round(TO_L(i),2)), ' s',"          ", 'TDR',"  ", num2str(round(TD_R(i),2)), ' s');
        str_foot{3} = strcat('GCTL',"  ", num2str(round(GCTL_stride(i),2)), ' s',"        ", 'GCTR',"  ", num2str(round(GCTR_stride(i),2)), ' s');
        str_foot{4+2} = strcat('vGRFL\_peak',"  ", num2str(round(GRFL_max(i),2)),' N');
        str_foot{9-3+2} = strcat('Impulse\_L',"  ", num2str(round(GRFL_area(i),2)),' N-s');
%         str_foot{6} = strcat('TOR',"  ", num2str(round(TO_R(i),2)), ' s');
%         str_foot{7} = strcat('TDR',"  ", num2str(round(TD_R(i),2)), ' s');
%         str_foot{8} = strcat('GCTR',"  ", num2str(round(GCTR_stride(i),2)), ' s');
        str_foot{5+2} = strcat('vGRFR\_peak',"  ", num2str(round(GRFR_max(i),2)),' N');
        str_foot{10-3+2} = strcat('Impulse\_R ',"  ", num2str(round(GRFR_area(i),2)),' N-s');
        str_foot{11-3-4} = strcat('flight time',"  ", num2str(round(FT_stride(i),2)),' s');
        str_foot{12-3-4} = strcat('double support',"  ", num2str(round(DST_stride(i),2)),' s');
        s_foot(i,:)=str_foot;
end
 
        font=10;
        dim1 = [.13 .73 .098*2 .2];
         t = annotation('textbox',dim1,'String',s(1,:),'EdgeColor','none' );
        t.FontSize=font;
        
        dim2 = [.323 .73 .095*2 .2];
         t = annotation('textbox',dim2,'String',s(2,:),'EdgeColor','none');
        t.FontSize=font;
        
        dim3 = [.52 .73 .096*2 .2];
         t = annotation('textbox',dim3,'String',s(3,:),'EdgeColor','none');
        t.FontSize=font;
         
        dim4 = [.709 .73 .099*2 .2];
         t = annotation('textbox',dim4,'String',s(4,:),'EdgeColor','none' );
        t.FontSize=font;
        
        font=10;
        dim1f = [.13 .33 .098*2 .2];
         t = annotation('textbox',dim1f,'String',s_foot(1,:),'EdgeColor','none' );
        t.FontSize=font;
        
        dim2f = [.323 .33 .095*2 .2];
         t = annotation('textbox',dim2f,'String',s_foot(2,:),'EdgeColor','none');
        t.FontSize=font;
        
        dim3f = [.52 .33 .096*2 .2];
         t = annotation('textbox',dim3f,'String',s_foot(3,:),'EdgeColor','none');
        t.FontSize=font;
         
        dim4f = [.709 .33 .099*2 .2];
         t = annotation('textbox',dim4f,'String',s_foot(4,:),'EdgeColor','none' );
        t.FontSize=font;
     
        str_abbreviation= {'TDL: touch-down left,        TOL: toe-off left,        GCTL: ground contact time left',...
                           'TDR: touch-down right,     TOR: toe-off right,     GCTR: ground contact time right',...
                           'vGRFL\_peak: left foot vGRF peak,      Impulse\_L: vGRFL impulse',...
                           'vGRFR\_peak: right foot vGRF peak,   Impulse\_R: vGRFR impulse',...
            };
        dim9 = [.13 .05 .5 .2];
        t = annotation('textbox',dim9,'String',str_abbreviation,'EdgeColor','none');
        t.FontSize=font;
        
        xta = [.13 .13];
        yta = [.035 .123];     
        annotation('textarrow',xta,yta,'String',{Itime},'FontSize',12);
        
        
        for i = 1:length(vel_stride_points)-1%-4
           ylims = [-1.2*0.25, 0];
           plot([TD_L(i)+(windowStart + front + vel_stride_points(i))/Fs TD_L(i)+(windowStart + front+vel_stride_points(i))/Fs], ylims, 'g-.','LineWidth',1);
           plot([(windowStart + front + vel_stride_points(i))/Fs + TO_L(i) (windowStart + front+ vel_stride_points(i))/Fs+TO_L(i)], ylims, 'g-.','LineWidth',1);
           ylims = [-1.2*0.5, 0];
           plot([TO_R(i)+(windowStart + front + vel_stride_points(i))/Fs TO_R(i)+(windowStart + front+ vel_stride_points(i))/Fs], ylims, 'r-.','LineWidth',1);
           plot([(windowStart + front + vel_stride_points(i))/Fs + TD_R(i) (windowStart + front+ vel_stride_points(i))/Fs + TD_R(i)], ylims, 'r-.','LineWidth',1);
        end
        
        text([TD_L(1) + (windowStart + front + vel_stride_points(1))/Fs (windowStart+front + vel_stride_points(1))/Fs + TO_L(1)...
            TO_R(1) + (windowStart + front + vel_stride_points(1))/Fs  (windowStart+front + vel_stride_points(1))/Fs + TD_R(1)],...
            [-0.4, -0.4, -0.7 -0.7], {'TDL', 'TOL', 'TOR', 'TDR'},'HorizontalAlignment','center')
        
        
        lgd=legend([hl hr hs(1) hlf hrf hv hd hf],{'left foot contact only','right foot contact only','beginning/ end of strides','left foot vGRF ','right foot vGRF '...
            ,'vertical velocity','double support','flight time'},'location','southeast','EdgeColor',[1 1 1],'FontSize',10);
        lgd.NumColumns = 3;
        legend boxoff        

        %% print 04 strides foot force  
        
        print(gcf,'04stridesfootforce','-dpng')
   
        %% 06 strides (walking) vertical Velocity and foot force 
          x = (windowStart + front :1: windowEnd - back).*(1/Fs);
          fig8 = figure
            
          pause(0.00001);
          frame_h = get(handle(gcf),'JavaFrame');
          set(frame_h,'Maximized',1);
          
          hold on
          colorheight=3.8; % for ylim
        
%         %left
%         y=lstepLable(windowStart + front:windowEnd - back);
%         hl = area(x,y.*1.8);
%         hl.EdgeColor=[1 1 1];
%         hl.FaceColor = [0 1 1];
%         hl.FaceAlpha = 0.5;
%         %right
%         y=rstepLable(windowStart + front:windowEnd - back);
%         hr = area(x,y.*1.8);
%         hr.EdgeColor=[1 1 1];
%         hr.FaceColor = [1 0 0];
%         hr.FaceAlpha = 0.5;
          
        %only left
        y=xor(lstepLable(windowStart + front:windowEnd - back),doubleSupportlable(windowStart + front:windowEnd - back));
        hl = area(x,y.*colorheight);
        hl.EdgeColor=[1 1 1];
        hl.FaceColor = [0 1 0];
        hl.FaceAlpha = 0.2;
        
        %only righht
        y=xor(rstepLable(windowStart + front:windowEnd - back),doubleSupportlable(windowStart + front:windowEnd - back));
        hr = area(x,y.*colorheight);
        hr.EdgeColor=[1 1 1];
        hr.FaceColor = [1 0 0];
        hr.FaceAlpha = 0.2;
        
        %flight 
        
        y=flightLable(windowStart + front:windowEnd - back);
        hf = area(x,y.*colorheight);
        hf.EdgeColor=[1 1 1];
        hf.FaceColor = [0.5 0.5 0.5];
        hf.FaceAlpha = 0.2;
        
        %double support 
        y=doubleSupportlable(windowStart + front:windowEnd - back);
        hd = area(x,y.*colorheight);
        hd.EdgeColor=[1 1 1];
        hd.FaceColor = [1 1 0];
        hd.FaceAlpha = 0.2;
        
         yyaxis left   
         ax = gca;
         ax.YColor = 'k';
         ylims = [-6.5, colorheight];
        ylim(ylims)
        hv=plot(x,vertVel_d,'Color',[0.5 0.8 0.8],'LineWidth',1); %velocity window has front and back removed
        set(gca,'ytick',-1:1:2)
      
%         hgt=plot(x,roll_anatomical_osc(windowStart + front:windowEnd - back),'Color',[0.7410 0.4470 0],'LineWidth',1); 


        xlabel('time (s)');
        ylabel('vertical velocity, m/s')   
%       title('step segmentation using vertical velocity')
        
%         ylims = [-1.2, colorheight];
%         ylim(ylims) 
        xlims = [x(1) x(end)];
        xlim(xlims)
        
        ylims = [-6.5*0.65, colorheight];
        for i = 1:1:length(vel_stride_points)
           hs(i)=plot([(windowStart + front + vel_stride_points(i))/Fs (windowStart + front+ vel_stride_points(i))/Fs], ylims, 'b-.');
        end
        
        %set(gca,'fontsize',16);
        set(gca,'fontsize',16,'box','off','TickDir','out','xtick',ceil(xlims(1)):floor(xlims(2)));

        yyaxis right
        ax = gca;
        ax.YColor  = 'k';
        ylims = [-6.5, colorheight];
        ylim(ylims)
        ylabel('vGRF(kN)');
        set(gca,'ytick',0:1:2)
                
        hrf=plot(x,rforce_intrp(windowStart + front:windowEnd - back)./1000,'LineWidth',1,'Color','r','LineStyle','-');
        hlf=plot(x,lforce_intrp(windowStart + front:windowEnd - back)./1000,'LineWidth',1,'Color',[0 0.6 0],'LineStyle','-');
             
%         lgd=legend([hl hlf hr hrf hd hf hv hs(1)],{'left foot contact','left foot force (kN)','right foot contact','right foot force (kN)', ...
%             'double support','flight time','vertical velocity (m/s)','beginning of the stride'},'location','southeast','EdgeColor',[1 1 1],'FontSize',10);
%         lgd.NumColumns = 4;
%         legend boxoff
           
parameters = 6;
s = cell(velStrideCount,parameters);
str = cell(1,parameters);

parameters_foot=12-3;
s_foot = cell(velStrideCount,parameters_foot);
str_foot = cell(1,parameters_foot);

for i=1:velStrideCount
    i
        str{1} = strcat('speed',"  ", num2str(round(velStrideSpeed(i),2)), ' m/s ,');
        str{2} = strcat('speed SD',"  ", num2str(round(fVel_std(i),2)),' m/s');
        str{3} = strcat('stride duration',"  ", num2str(round(velStrideDuration(i),2)), ' s');
        str{4} = strcat('vertical disp.(p-p)',"  ", num2str(round(velStride_vertDisp_pp(i),2)),' cm');
        str{5} = strcat('forward acc.(p-p)',"  ", num2str(round(F_acc_pp(i),2)),' m/s^2');
        str{6} = strcat('vertical acc.(p-p)',"  ", num2str(round(V_acc_pp(i),2)), ' m/s^2');
        s(i,:)=str;
        
        str_foot{1} = strcat('TDL'," ", num2str(round(TD_L(i),2)), 's',"    ", 'TOR',"  ", num2str(round(TO_R(i),2)), 's');
        str_foot{2} = strcat('TOL'," ", num2str(round(TO_L(i),2)), 's',"    ", 'TDR',"  ", num2str(round(TD_R(i),2)), 's');
        str_foot{3} = strcat('GCTL'," ", num2str(round(GCTL_stride(i),2)), 's',"  ", 'GCTR',"  ", num2str(round(GCTR_stride(i),2)), 's');
        str_foot{4+2} = strcat('vGRFL\_peak',"  ", num2str(round(GRFL_max(i),2)),' N');
        str_foot{9-3+2} = strcat('Impulse\_L',"  ", num2str(round(GRFL_area(i),2)),' N-s');
%         str_foot{6} = strcat('TOR',"  ", num2str(round(TO_R(i),2)), ' s');
%         str_foot{7} = strcat('TDR',"  ", num2str(round(TD_R(i),2)), ' s');
%         str_foot{8} = strcat('GCTR',"  ", num2str(round(GCTR_stride(i),2)), ' s');
        str_foot{5+2} = strcat('vGRFR\_peak',"  ", num2str(round(GRFR_max(i),2)),' N');
        str_foot{10-3+2} = strcat('Impulse\_R ',"  ", num2str(round(GRFR_area(i),2)),' N-s');
        str_foot{11-3-4} = strcat('flight time',"  ", num2str(round(FT_stride(i),2)),' s');
        str_foot{12-3-4} = strcat('double support',"  ", num2str(round(DST_stride(i),2)),' s');
        s_foot(i,:)=str_foot;
        
end

        font=9;
        dim1 = [.127 .73 .098*2 .2];
         t = annotation('textbox',dim1,'String',s(1,:),'EdgeColor','none' );
        t.FontSize=font;
        dim2 = [.258 .73 .095*2 .2];
         t = annotation('textbox',dim2,'String',s(2,:),'EdgeColor','none');
        t.FontSize=font;
        dim3 = [.387 .73 .096*2 .2];
         t = annotation('textbox',dim3,'String',s(3,:),'EdgeColor','none');
        t.FontSize=font;
        dim4 = [.517 .73 .099*2 .2];
         t = annotation('textbox',dim4,'String',s(4,:),'EdgeColor','none' );
        t.FontSize=font;
        dim5 = [.646 .73 .095*2 .2];
         t = annotation('textbox',dim5,'String',s(5,:),'EdgeColor','none');
        t.FontSize=font;
        dim6 = [.773 .73 .095*2 .2];
         t = annotation('textbox',dim6,'String',s(6,:),'EdgeColor','none' );
        t.FontSize=font;
        
        dim1f = [.127 .33 .098*2 .2];
         t = annotation('textbox',dim1f,'String',s_foot(1,:),'EdgeColor','none' );
        t.FontSize=font;
        dim2f = [.258 .33 .095*2 .2];
         t = annotation('textbox',dim2f,'String',s_foot(2,:),'EdgeColor','none');
        t.FontSize=font;
        dim3f = [.387 .33 .096*2 .2];
         t = annotation('textbox',dim3f,'String',s_foot(3,:),'EdgeColor','none');
        t.FontSize=font;
        dim4f = [.517 .33 .099*2 .2];
         t = annotation('textbox',dim4f,'String',s_foot(4,:),'EdgeColor','none' );
        t.FontSize=font;
         dim5f = [.646 .33 .095*2 .2];
         t = annotation('textbox',dim5f,'String',s_foot(5,:),'EdgeColor','none');
        t.FontSize=font;
        dim6f = [.773 .33 .095*2 .2];
         t = annotation('textbox',dim6f,'String',s_foot(6,:),'EdgeColor','none' );
        t.FontSize=font;
     
        str_abbreviation= {'TDL: touch-down left,        TOL: toe-off left,        GCTL: ground contact time left',...
                           'TDR: touch-down right,     TOR: toe-off right,     GCTR: ground contact time right',...
                           'vGRFL\_peak: left foot vGRF peak,      Impulse\_L: vGRFL impulse',...
                           'vGRFR\_peak: right foot vGRF peak,   Impulse\_R: vGRFR impulse',...
            };
        dim9 = [.13 .05 .5 .2];
        t = annotation('textbox',dim9,'String',str_abbreviation,'EdgeColor','none');
        t.FontSize=font;
        
        xta = [.154 .13];
        yta = [.035 .129];     
        annotation('textarrow',xta,yta,'String',{Itime},'FontSize',12);
        
        for i = 1:length(vel_stride_points)-1%-6 % TD/TO vertical lines 
           ylims = [-1.2*0.25, 0];
           plot([TD_L(i)+(windowStart + front + vel_stride_points(i))/Fs TD_L(i)+(windowStart + front+vel_stride_points(i))/Fs], ylims, 'g-.','LineWidth',1);
           plot([(windowStart + front + vel_stride_points(i))/Fs + TO_L(i) (windowStart + front+ vel_stride_points(i))/Fs+TO_L(i)], ylims, 'g-.','LineWidth',1);
           ylims = [-1.2*0.5, 0];
           plot([TO_R(i)+(windowStart + front + vel_stride_points(i))/Fs TO_R(i)+(windowStart + front+ vel_stride_points(i))/Fs], ylims, 'r-.','LineWidth',1);
           plot([(windowStart + front + vel_stride_points(i))/Fs + TD_R(i) (windowStart + front+ vel_stride_points(i))/Fs + TD_R(i)], ylims, 'r-.','LineWidth',1);
        end
        
        text([TD_L(1)+(windowStart+front+vel_stride_points(1))/Fs (windowStart+front+vel_stride_points(1))/Fs+TO_L(1)...
            TO_R(1)+(windowStart+front+vel_stride_points(1))/Fs  (windowStart+front+vel_stride_points(1))/Fs+TD_R(1)],...
            [-0.4, -0.4, -0.7 -0.7], {'TDL', 'TOL', 'TOR', 'TDR'},'HorizontalAlignment','center')
        
        lgd=legend([hl hr hs(1) hlf hrf hv hd hf],{'left foot contact only','right foot contact only','beginning/ end of strides','left foot vGRF ','right foot vGRF '...
            ,'vertical velocity','double support','flight time'},'location','southeast','EdgeColor',[1 1 1],'FontSize',10);
        lgd.NumColumns = 3;
        legend boxoff        
       
        %% print 06 strides foot force  
        
        print(gcf,'06stridesfootforce','-dpng')
 