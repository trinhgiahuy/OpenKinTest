clear
%% Create database files' name
subjectId=3;
sortedInputFeatures=true;

%% Insert more files here into database
db_oxy = containers.Map({'1','2','3','4','5','7','8'}, ...
                    {'OXYGA1 marker 0120_s','OXYGA2 marker 0035_s','OXYGA3 marker 0110_s', ...
                     'OXYGA4 marker 0033_s','OXYGA5 marker 0036_s','OXYGA7 marker 0036_s', ...
                     'OXYGA8 marker 0106_s'});
db_gaitpod = containers.Map({'1','2','3','4','5','7','8'}, ...
                            {'2022-05-23-08-06-47_s','2022-05-23-08-50-25_s','2022-05-23-09-32-52_s', ...
                             '2022-05-23-10-21-26_s','2022-05-23-10-59-15_s','2022-05-24-07-32-47_s', ...
                             '2022-05-24-08-10-47_s'});

db_filterOrder = containers.Map({'1','2','3','4','5','7','8'},{ ...
                          [1,3,1,3,1,3],[5,7,5,7,5,7],[1,3,1,3,1,3],[1,3,1,3,1,3], ...
                          [1,3,1,3,1,3],[1,3,1,3,1,3],[1,3,1,3,1,3]});


fileID=[db_gaitpod(num2str(subjectId)),'.txt'];
oxygenID=[db_oxy(num2str(subjectId)),'.txt'];
%% Load HR file

HR_fn="2022-11-01_09-25-19_203130000581_HeartRateTestActivity_s.csv";
HRopts = detectImportOptions(HR_fn);
HRopt.DataLines = 1;
%HRopts.VariableNamesLine = 1;
HR_data_table = readtable(HR_fn,HRopts);

HR_data = table2array(HR_data_table(:,2));


orderArr=db_filterOrder(num2str(subjectId));
order=orderArr(1);
framelen=orderArr(2);
order2=orderArr(3);
framelen2=orderArr(4);
order3=orderArr(5);
framelen3=orderArr(6);

T = readtable(fileID,'Delimiter','\t','ReadVariableNames',false);

v_n = table2array(T(:,18));
v_e = table2array(T(:,19));


V_lon = NaN(max(size(T)), 1);


for k=1:max(size(T))
   
    % IMU horizontal speed
    V_lon(k) = sqrt(v_n(k)^2 + v_e(k)^2);
end


disp('Data Parsing has been done!!');

%load vel_imu.mat
%load vel_gps.mat
V_lon=V_lon.'; 
%V_gps_lon=V_gps_lon.';
%V_lon_smooth=smoothdata(V_lon,'movmedian',8000);
fig=figure('Name','Velocity','NumberTitle','off','units','normalized','outerposition',[0 0 1 1]);
%plot(V_lon.*5,'m')
%plot(V_lon);
%plot(V_lon_smooth);
%hold on
%plot(1./(V_lon_smooth.^2))
%a=thresholdFilter(1./(V_lon_smooth.^3),4000,5);
%plot(a,'-or')
%b=avgThresBased(V_lon_smooth,a);
%plot(b,'-xk')
%plot(V_lon_smooth);
%hold off

% Smooth before sampling
%V_lon=smoothdata(V_lon,'movmean',8000);

N = length(V_lon);
idx=(1:2000:N);

%b_match=meanSampling(V_lon,idx);
b_match = zeros(size(idx));
for i=1:length(idx)-1
    b_match(i)=mean(V_lon(idx(i):idx(i+1)-1));
end

%% HR indexing based on average time stamp difference
timeStamp = table2array(HR_data_table(:,1));
timeDiffAvg=1/(mean(diff(timeStamp))/1000)*5;
timeDiffAvg_f = floor(1/(mean(diff(timeStamp))/1000)*5);
hr_N=length(HR_data);
%hr_idx=(1: 8.7566:hr_N);
%floor(1/(mean(diff(timeStamp))/1000)*5)

%% TODO: Check these 2 options for hr_match and try with ceil
% TOTAL 5 options:
% 1. Keep average timeDiff for indexing (1:8.765:hr_N)
% 2. **Look promising** Use floor average timeDiff for indexing (1:floor(8.765):hr_N)
% 3. Use ceil average 
% 4. Use floor for whole indexing floor(1:8.765:hr_N)
% 5. Use ceil for whole indexing ceil(1:8.765:hr_N)
% And return the hr_match for step segmented in input_features.m
%hr_idx=floor(1:timeDiffAvg:hr_N);
hr_idx=(1:floor(timeDiffAvg):hr_N);

hr_match = zeros(size(hr_idx))
for i=1:length(hr_idx)-1
    hr_match(i)=mean(HR_data(hr_idx(i):hr_idx(i+1)-1));
end
%opt = delimitedTextImportOptions('VariableNamesLine',5,'DataLines',8,'NumVariables',15);
%oxygenData=readtable('OXYGA1 marker 0120.csv',opt);

%oxygenData=readtable(oxygenID,'Delimiter','\t','ReadVariableNames',false,'HeaderLines',7)
%No need ignore header
oxygenData=readtable(oxygenID,'Delimiter','\t','ReadVariableNames',false);
VO2_in=table2array(oxygenData(:,10));
VO2_in=VO2_in.';

%% TEST: USING Savitzky-Golay filtering for get rid of outlined data
% The degree must be less than the frame length.)
% Frame length must be odd.

VO2_in_sgolay1=sgolayfilt(VO2_in,order,framelen);
%plot(VO2_in_sgolay,'r');
%hold on

%% TEST2 USING Savitzky-Golay filtering 2 for get rid of 2nd order outline data

VO2_in_sgolay=sgolayfilt(VO2_in_sgolay1,order2,framelen2);
%plot(VO2_in_sgolay,'c');

%% TEST3 USING Savitzky-Golay filtering 3 for get rid of 3rd order outline data

VO2_in_sgolay=sgolayfilt(VO2_in_sgolay,order3,framelen3);

%% TODO: TAKE b_match as average value for all sequences in 5 secs
%b_match=V_lon(idx);

% Smooth after sampling
%b_match=smoothdata(b_match,'movmean',3)
%VO2_in_sgolay=VO2_in;
yyaxis left          
plot(b_match(1,1:length(VO2_in_sgolay)),'m');
hold on

yyaxis right
plot(VO2_in,'c--o');
hold on

yyaxis right
plot(VO2_in_sgolay,'-k');
hold on

%yaxis left
plot(hr_match(1,1:length(hr_match)),'-r');
%% TEST: USING SMOOTH DATA
%VO2_in_final=smoothdata(VO2_in_sgolay,'movmean');
%plot(VO2_in_final,'p');            
                       
%a2=thresholdFilter(VO2_in_sgolay,3,5);
%b2=avgThresBased(VO2_in_sgolay,a2);
%plot(b2.*2,'p');
hold off     

%plot(a2,'-or')

%plot(VO2_in)

title('Longitudinal velocity')
xlabel('time, *5 sec')
yyaxis left
ylabel('Longitudinal velocity, m/s')

yyaxis right
ylabel('VO2/kg, ml/min/kg')
legend('Longitudinal velocity INS','VO2 Uptake Original','VO2 Uptake per kg smooth[ml/min/kg]','location','best')

