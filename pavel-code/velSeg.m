clear all

load('O1-out\vel_imu.mat')
load('O1-out\acc.mat')
load('O1-out\euler.mat')
path_motion = "C:\Users\pavel\Desktop\OpenKin DNN implementation\Oxygen prediction LSTM\dataO2_O8\Params_O2-O8\O2-out"

path_oxygen = "C:\Users\pavel\Desktop\OpenKin DNN implementation\Oxygen prediction LSTM\dataO2_O8\VO2_O2-O8"
%oxygen=csvread('O1-out\O1_output.csv')
oxygen=csvread('path_oxygen\O1_output_smooth.csv')

%%
windowStart = 1;
windowEnd = length(V_d);

Fs = 400;
vertVel = V_d(1, windowStart:windowEnd);
%%
vertVel_d = detrend(vertVel);
maxV = max(vertVel_d);
thresh = 0.05 * maxV;
a = false;
b = false;
front = 0;
back = 0;

while (true) 
    if (abs(vertVel_d(1, 1)) > thresh || vertVel_d(1, 1) > vertVel_d(1, 2))
        vertVel_d = vertVel_d(1, 3:end);
        vertVel_d = detrend(vertVel_d);
        front = front + 2;
        a = false;
    else
        a = true;
    end
    if (vertVel_d(1, end) > thresh || vertVel_d(1, 1) > vertVel_d(1, 2))
        vertVel_d = vertVel_d(1, 1:end - 3);
        vertVel_d = detrend(vertVel_d);
        back = back + 3;
        a = false;
        b = false;
    else
        b = true;
    end
    if (a && b)
        break;
    end
end
%%
yaw = detrend(yaw(1, windowStart + front:windowEnd - back));
pitch = pitch(1, windowStart + front:windowEnd - back);
v_lon = V_lon(1, windowStart + front:windowEnd - back);
v_d = V_d(1, windowStart + front:windowEnd - back);
v_e = V_e(1, windowStart + front:windowEnd - back);
v_n = V_n(1, windowStart + front:windowEnd - back);
a_d = acc_d(1, windowStart + front:windowEnd - back);
a_lon = A_lon(1, windowStart + front:windowEnd - back);




vel_thrsld=(v_lon>=1.5);
vel_thrsld_step=(v_lon>=1.5).*v_lon;
numCode = vel_thrsld;
runMat = [];
numCode = [numCode nan]; % dummy ending
N = 1;
valuecode=0;
lengthcode=0;

for i = 1:length(numCode)-1
    if numCode(i)==numCode(i+1)
        N = N + 1;
    else
        valuecode = numCode(i);
        lengthcode = lengthcode+N;
        N = 1;
        runMat = [runMat; valuecode lengthcode];
    end
end
runMat'

FastRunMat = [];
runCount =0;
for i=1:length(runMat)-1
     if runMat(i,1)==1 
         runCount=runCount+1; 
         if i==1
         FastRunMat=[FastRunMat; runCount 1 runMat(i,2)]
         else 
         FastRunMat=[FastRunMat; runCount runMat(i-1,2)+1 runMat(i,2)]
         end 
     end  
end




%%
steps = [];
if (vertVel_d(1, 1) > 0)
    intersections = 1;
    steps = 1;
else
    intersections = 0;
end
count = 0;
points = [];

for i = 1:length(vertVel_d) - 1
    if (vertVel_d(1, i) < 0 && vertVel_d(1, i + 1) > 0)
        intersections = intersections + 1;
        points = [points, i];
        steps = [steps, i];
        count = count + 1;
    end 
    if (vertVel_d(1, i) > 0 && vertVel_d(1, i + 1) < 0)
        intersections = intersections + 1;
        points = [points, i];
        count = count + 1;
    end
end
stepCount = length(steps) - 1;

%%
stepDuration = [];
stepLength = [];
cadence = [];
vertOscillation_vel_amp = [];
vertOscillation_dist_amp = [];
postureAmp = [];
postureMean = [];
torque = [];
speed = [];
speedChange = [];
stepVels_d_aray=[];

for i = 1:stepCount
    stepDuration = [stepDuration, (steps(i + 1) - steps(i))/Fs];
    cadence = [cadence, 1/stepDuration(i)];
    stepVelsLon = v_lon(steps(i):steps(i + 1));
    stepVels_d = v_d(steps(i):steps(i + 1));
    stepPitch = pitch(steps(i):steps(i + 1));
    stepYaw = yaw(steps(i):steps(i + 1));
      
    stepLength = [stepLength, trapz(stepVelsLon./Fs)];% step displacement longitudinal in a step
    
    vertOscillation_vel_amp = [vertOscillation_vel_amp, ...
        ((max(stepVels_d)) + (min(stepVels_d)))/2];    
    
    for j=steps(i):1:(steps(i+1)-1) 
        v_d(j)=v_d(j)-vertOscillation_vel_amp(i);
    end
    stepVels_d = v_d(steps(i):steps(i + 1));
    stepDispDown = cumtrapz(stepVels_d./Fs); %step displacement downwards in a step
    stepVels_d_aray=[stepVels_d_aray,stepDispDown]; 
    vertOscillation_dist_amp = [vertOscillation_dist_amp, ...
         (abs(max(stepDispDown)) + abs(min(stepDispDown)))/2];
 
    speed = [speed, mean(stepVelsLon)];
    speedChange = [speedChange, max(stepVelsLon) - min(stepVelsLon)];
    postureAmp = [postureAmp, max(stepPitch)];
    postureMean = [postureMean, mean(stepPitch)];
    
    torque = [torque, max(abs(stepYaw))];
end
    plot(v_d)
%     figure
%     plot(vertOscillation_vel_amp)    
%     figure 
%     plot(stepVels_d_aray)
    
% %%
% time = 1/Fs:1/Fs:length(v_d)/Fs;
% figure               
% 
% plot(time, v_lon, 'b')
% hold on 
% plot(time, v_d, 'k')
% grid on
% % lims = [-0.4, 1];
% % lims = [1.2, 2];
% lims = [0, 25];
% % lims = [0, 10];
% % lims = [-0.1, 0.1];
% ylim(lims)
% xlim([1/Fs, length(v_d)/Fs])
% 
% for i = 1:length(steps)
%     
%    plot([steps(i)/Fs steps(i)/Fs], lims, 'r-.')
%     
% end
% 
% ylabel('Vertical Velocity - m/s')
% title('Vertical Velocity Profile')
% xlabel('time - s')
% 
% legend('Velocity', 'gait segmentation')
% hold off
% 
% %%
% 
% clearvars -except torque v_d vertVel_d v_lon postureAmp postureMean ...
%     speed speedChange vertOscillation_dist_amp vertOscillation_vel_amp ...
%     stepLength stepDuration cadence stepCount steps points Fs a_d a_lon ...
%     time yaw front windowStart windowEnd
% 

%% rough
% 
% figure
% 
% hold on
% 
% plot(time, cumtrapz(v_d./Fs), 'b')
% grid on
% lims = [-0.4, 1];
% 
% for i = 1:length(steps)
%     
%    plot([steps(i)/Fs steps(i)/Fs], lims, 'r-.')
%     
% end
% hold off
% 
% figure
% plot(time, (v_d./Fs))