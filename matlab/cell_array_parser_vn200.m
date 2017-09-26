function data = cell_array_parser(fileID, vn200)
% creates a nested structure from a cell array
%fileID='2016-10-16-20-57-38.txt';
%fileID='2016-11-03-16-22-42.txt';
%fileID='/Users/heikki/Projektit/tut/dualimu.txt';
% load the data in form of cell array
%data_cell=importdata(fileID);

if ~exist('vn200', 'var')
    vn200 = false;
end

if (vn200)
    of = 8;
else
    of = 0;
end

tic

% read data to table, delimited by tab
T = readtable(fileID,'Delimiter','\t','ReadVariableNames',false);

if (~vn200)
    if (size(T,2) == 32+8)
        disp("Probably vn200-data? add true as second argument or press to continue anyway");
        pause;
    end
end

% most data to arrays
gps_seq = table2array(T(:,1));
gps_stamp = table2array(T(:,2));
imu_seq = table2array(T(:,3));
ang_vel1 = table2array(T(:,7));
ang_vel2 = table2array(T(:,8));
ang_vel3 = table2array(T(:,9));
quat1 = table2array(T(:,10));
quat2 = table2array(T(:,11));
quat3 = table2array(T(:,12));
quat4 = table2array(T(:,13));
acc1 = table2array(T(:,14));
acc2 = table2array(T(:,15));
acc3 = table2array(T(:,16));
pos1 = table2array(T(:,28+of));
pos2 = table2array(T(:,29+of));
pos3 = table2array(T(:,30+of));
imu_on = table2array(T(:,26+of));
id = table2array(T(:,32+of));

gps_itow = table2array(T(:,17));
pozyx_seq = table2array(T(:,27+of));
lat = table2array(T(:,4));
lon = table2array(T(:,5));
alt = table2array(T(:,6));
v_n = 1e-2.*table2array(T(:,18));
v_e = 1e-2.*table2array(T(:,19));
v_d = 1e-2.*table2array(T(:,20));
grnd_speed = 1e-2.*table2array(T(:,22));
hdg = 1e-5.*table2array(T(:,23));
cov_speed = 1e-2.*table2array(T(:,24));
cov_hdg = 1e-5.*table2array(T(:,25));

uwb = table2array(T(:,31+of));

if (vn200)
    gpstime = table2array(T(:,26));
    dtime = table2array(T(:,27));
    dtheta_x = table2array(T(:,28));
    dtheta_y = table2array(T(:,29));
    dtheta_z = table2array(T(:,30));
    dvel_x = table2array(T(:,31));
    dvel_y = table2array(T(:,32));
    dvel_z = table2array(T(:,33));
end

last = NaN;
for k=1:length(imu_seq)
    if (~isnan(last))
        imuseq = imu_seq(k);
        if (~isnan(imuseq))
            while (abs(imuseq-last) > 100)
                imuseq = imuseq + 65536;
            end
            imu_seq(k) = imuseq;
            last = imuseq;
        else
            imu_seq(k) = NaN;
        end
    else
        last = imu_seq(k);
        %imu_seq(k) = imu_seq(k);
    end
end

[xData, yData] = prepareCurveData( imu_seq(:), gps_stamp(:) );

% Set up fittype and options.
ft = fittype( 'poly1' );

% Fit model to data.
[mtifit, ~] = fit( xData, yData, ft );

mtits = imu_seq(:).*mtifit.p1+mtifit.p2-0.015;

for n=1:(size(T,2)-(32+of))/12

    tmp = table2array(T(:,[33+of+(n-1)*12,35+of+(n-1)*12:44+of+(n-1)*12]));
	%mtw(n).id2 = table2array(T(:,34+(n-1)*12));
	%mtw(n).seq = table2array(T(:,33+(n-1)*12));
	%mtw(n).ang_vel(1) = table2array(T(:,35+(n-1)*12));
	%mtw(n).ang_vel(2) = table2array(T(:,36+(n-1)*12));
	%mtw(n).ang_vel(3) = table2array(T(:,37+(n-1)*12));
	%mtw(n).quat(1) = table2array(T(:,38+(n-1)*12));
	%mtw(n).quat(2) = table2array(T(:,39+(n-1)*12));
	%mtw(n).quat(3) = table2array(T(:,40+(n-1)*12));
	%mtw(n).quat(4) = table2array(T(:,41+(n-1)*12));
	%mtw(n).acc(1) = table2array(T(:,42+(n-1)*12));
	%mtw(n).acc(2) = table2array(T(:,43+(n-1)*12));
	%mtw(n).acc(3) = table2array(T(:,44+(n-1)*12));
    mtw(:,:,n) = tmp;
    
    last = NaN;
    for k=1:size(mtw,1)
        if (~isnan(last))
            imuseq = mtw(k,1,n);
            if (~isnan(imuseq))
                while (abs(imuseq-last) > 100 && imuseq < last)
                    imuseq = imuseq + 65536;
                end
                mtw(k,1,n) = imuseq;
                last = imuseq;
            else
                mtw(k,1,n) = NaN;
            end
        else
            last = mtw(k,1,n);
            %mtw(k,1,n) = imu_seq(k);
        end
    end
    
    [xData, yData] = prepareCurveData( mtw(:,1,n), gps_stamp(:) );

    % Fit model to data.
    [mtwfit, ~] = fit( xData, yData, ft );
    
    mtwts(n,:) = mtw(:,1,n).*mtwfit.p1+mtwfit.p2;
end


uwbtags = containers.Map;

% number of rows in the table max(size(T))
for k=1:max(size(T))
    % place data to struct
    
    
    if (vn200)
        data(k).time.gpstime = gpstime(k);
        data(k).d.time = dtime(k);
        data(k).d.theta(1) = dtheta_x(k);
        data(k).d.theta(2) = dtheta_y(k);
        data(k).d.theta(3) = dtheta_z(k);
        data(k).d.vel(1) = dvel_x(k);
        data(k).d.vel(2) = dvel_y(k);
        data(k).d.vel(3) = dvel_z(k);
    end
    
    data(k).time.gps_seq = gps_seq(k);
    data(k).time.ros_stamp = gps_stamp(k);
    data(k).time.imu_seq = imu_seq(k);
    data(k).time.gps_itow = gps_itow(k);
    data(k).time.pozyx_seq = pozyx_seq(k);
    data(k).gps.lat = lat(k);
    data(k).gps.lon = lon(k);
    data(k).gps.alt = alt(k);
    data(k).gps.v_n = v_n(k);
    data(k).gps.v_e = v_e(k);
    data(k).gps.v_d = v_d(k);
    data(k).gps.grnd_speed = grnd_speed(k);
    data(k).gps.hdg = hdg(k);
    data(k).gps.cov_speed = cov_speed(k);
    data(k).gps.cov_hdg = cov_hdg(k);
    data(k).imu.ang_vel(1) = ang_vel1(k);
    data(k).imu.ang_vel(2) = ang_vel2(k);
    data(k).imu.ang_vel(3) = ang_vel3(k);
    data(k).imu.quat(1) = quat1(k); %q_1 pr q_2
    data(k).imu.quat(2) = quat2(k); %q_3
    data(k).imu.quat(3) = quat3(k); %q_1 pr q_2
    data(k).imu.quat(4) = quat4(k); %q_0
    data(k).imu.acc(1) = acc1(k); % A_x
    data(k).imu.acc(2) = acc2(k); % A_y
    data(k).imu.acc(3) = acc3(k); % A_z
    
    data(k).imu.ts = mtits(k);
    
    data(k).pozyx.pos(1) = pos1(k);
    data(k).pozyx.pos(2) = pos2(k);
    data(k).pozyx.pos(3) = pos3(k);
    data(k).pozyx.imu_on = imu_on(k);
    
    data(k).imu.id = id(k);
    
    if (strcmp(char(uwb(k)),'NaN') == 0)
        
        ranges = strsplit(char(uwb(k)), '|');
        for n=1:size(ranges,2)-1
            fields = strsplit(char(ranges(n)), ',');
            vals = struct();
            for i=1:size(fields,2)
                %fields(i)
                parts = strsplit(char(fields(i)), '=');
                if (strcmp(char(parts(1)), 's') || strcmp(char(parts(1)), 't'))
                    vals.(char(parts(1))) = parts(2);
                else
                    vals.(char(parts(1))) = str2double(char(parts(2)));
                end
            end
            if (isKey(uwbtags, vals.('t')))
                data(k).uwb.tag(uwbtags(char(vals.('t')))) = vals;
            else
                key = uwbtags.size(1)+1;
                uwbtags(char(vals.('t'))) = key;
                data(k).uwb.tag(key) = vals;
            end
        end
    end
    
    % for each additional mtw
    for n=1:(size(T,2)-32)/12

        data(k).mtw(n).id2 = table2array(T(k,34+(n-1)*12));
        data(k).mtw(n).seq = mtw(k,1,n);
        data(k).mtw(n).ang_vel(1) = mtw(k,2,n);
        data(k).mtw(n).ang_vel(2) = mtw(k,3,n);
        data(k).mtw(n).ang_vel(3) = mtw(k,4,n);
        data(k).mtw(n).quat(1) = mtw(k,5,n);
        data(k).mtw(n).quat(2) = mtw(k,6,n);
        data(k).mtw(n).quat(3) = mtw(k,7,n);
        data(k).mtw(n).quat(4) = mtw(k,8,n);
        data(k).mtw(n).acc(1) = mtw(k,9,n);
        data(k).mtw(n).acc(2) = mtw(k,10,n);
        data(k).mtw(n).acc(3) = mtw(k,11,n);
        
        data(k).mtw(n).ts = mtwts(n,k);
     end
    
end

toc


% formatSpec = '%s %d %2.1f %s\n';
% fileID = fopen('Oct20_celldata.dat','w');
% [nrows,ncols] = size(data);
% for row = 1:nrows
%     fprintf(fileID,formatSpec,data{row,:});
% end