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

% newdata_smooth=[b_match(1,1:length(VO2_in_sgolay));VO2_in_sgolay(1,:)].';
% csvwrite(['O',oxygenID(6),'_output_smooth.csv'],newdata_smooth);
csvwrite(['O',oxygenID(6),'_output_smooth.csv'],VO2_in_sgolay(1,:).');
% newdata=[b_match(1,1:length(VO2_in));VO2_in(1,:)].';
% csvwrite(['O',oxygenID(6),'_output.csv'],newdata);

% Walking/running data processing
cell_array_parser_vn200(fileID,subjectId,'vn200');

% This function will create acceleration, velocity_imu and Euler mat files
%disp('Call features_OK()...');
%features_OK(data,subjectId);
%disp('Finish features_OK()')

% This function will load following files acc.mat, vel_imu.mat, euler.mat
% and calculate input's features for ML model: 
% speed, speedChange, stepDuration, vertOscillation_dis_amp with oxygen
disp('Call velSeg()...');
[windowEnd_r,back_r,stepCount]=velSeg(subjectId);
disp('Finish velSeg()')

% This function will use those input's feature and creating
% training/testing set for ML model
disp('Call input_features()...');
input_features(subjectId,sortedInputFeatures);
disp('Finish everything! Please find the input features file in same directory')
pause(15);
disp("front:windowEnd")
disp(windowEnd_r);
disp("Back:")
disp(back_r)
disp("Step count:");
disp(stepCount);
disp("front:windowEnd")
disp(windowEnd_r);