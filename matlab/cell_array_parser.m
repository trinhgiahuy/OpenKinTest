function data = cell_array_parser(fileID)
% creates a nested structure from a cell array
%fileID='2016-10-16-20-57-38.txt';
%fileID='2016-11-03-16-22-42.txt';
%fileID='/Users/heikki/Projektit/tut/dualimu.txt';
% load the data in form of cell array
%data_cell=importdata(fileID);

% read data to table, delimited by tab
T = readtable(fileID,'Delimiter','\t','ReadVariableNames',false);

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
pos1 = table2array(T(:,27));
pos2 = table2array(T(:,28));
pos3 = table2array(T(:,29));
imu_on = table2array(T(:,26));
id = table2array(T(:,32));

uwb = table2array(T(:,31));

uwbtags = containers.Map;
    

% number of rows in the table max(size(T))
for k=1:max(size(T))
    % place data to struct
    
    data(k).time.gps_seq = gps_seq(k);
    data(k).time.gps_stamp = gps_stamp(k);
    data(k).time.imu_seq = imu_seq(k);
%     data(k).time.gps_itow = table2array(T(k,17));
%     data(k).time.pozyx_seq = table2array(T(k,27));
%     data(k).gps.lat = table2array(T(k,4));
%     data(k).gps.lon = table2array(T(k,5));
%     data(k).gps.alt = table2array(T(k,6));
%     data(k).gps.v_n = 1e-2*table2array(T(k,18));
%     data(k).gps.v_e = 1e-2*table2array(T(k,19));
%     data(k).gps.v_d = 1e-2*table2array(T(k,20));
%     data(k).gps.grnd_speed = 1e-2*table2array(T(k,22));
%     data(k).gps.hdg = 1e-5*table2array(T(k,23));
%     data(k).gps.cov_speed = 1e-2*table2array(T(k,24));
%     data(k).gps.cov_hdg = 1e-5*table2array(T(k,25));
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
                if (strcmp(char(parts(1)), 's') | strcmp(char(parts(1)), 't'))
                    vals.(char(parts(1))) = parts(2);
                else
                    vals.(char(parts(1))) = str2num(char(parts(2)));
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
        data(k).mtw(n).seq = table2array(T(k,33+(n-1)*12));
        data(k).mtw(n).ang_vel(1) = table2array(T(k,35+(n-1)*12));
        data(k).mtw(n).ang_vel(2) = table2array(T(k,36+(n-1)*12));
        data(k).mtw(n).ang_vel(3) = table2array(T(k,37+(n-1)*12));
        data(k).mtw(n).quat(1) = table2array(T(k,38+(n-1)*12));
        data(k).mtw(n).quat(2) = table2array(T(k,39+(n-1)*12));
        data(k).mtw(n).quat(3) = table2array(T(k,40+(n-1)*12));
        data(k).mtw(n).quat(4) = table2array(T(k,41+(n-1)*12));
        data(k).mtw(n).acc(1) = table2array(T(k,42+(n-1)*12));
        data(k).mtw(n).acc(2) = table2array(T(k,43+(n-1)*12));
        data(k).mtw(n).acc(3) = table2array(T(k,44+(n-1)*12));
     end
    
end




% formatSpec = '%s %d %2.1f %s\n';
% fileID = fopen('Oct20_celldata.dat','w');
% [nrows,ncols] = size(data);
% for row = 1:nrows
%     fprintf(fileID,formatSpec,data{row,:});
% end