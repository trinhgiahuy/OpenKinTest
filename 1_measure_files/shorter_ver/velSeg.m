function [windowEnd_r,back_r,stepCount] = velSeg(subjectId)
fileDir=['O',num2str(subjectId),'_out\'];
load([fileDir,'vel_imu.mat']);
oxyFile=['O',num2str(subjectId),'_output_smooth.csv'];
oxygen=csvread(oxyFile);
windowStart = 1;
windowEnd = length(V_d);
disp(windowEnd);
Fs = 400;
vertVel = V_d(1, windowStart:windowEnd);

vertVel_d = detrend(vertVel);
maxV = max(vertVel_d);
thresh = 0.05 * maxV;
a = false;
b = false;
front = 0;
back = 0;

tic
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

v_lon = V_lon(1, windowStart + front:windowEnd - back);
v_d = V_d(1, windowStart + front:windowEnd - back);
%%disp("front:windowEnd - back:")
windowEnd_r=front:windowEnd;
back_r=back;
%%v_e = V_e(1, windowStart + front:windowEnd - back);
%%v_n = V_n(1, windowStart + front:windowEnd - back);
%%a_d = acc_d(1, windowStart + front:windowEnd - back);
%%a_lon = A_lon(1, windowStart + front:windowEnd - back);

%% Calculating steps
steps = [];
if (vertVel_d(1, 1) > 0)
    steps = 1;
end

for i = 1:length(vertVel_d) - 1
    if (vertVel_d(1, i) < 0 && vertVel_d(1, i + 1) > 0)
        steps = [steps, i];
    end 
end

%disp('Length Steps');
%length(steps);

stepCount = length(steps) - 1;
%disp("Step count:");
%%disp(stepCount)
%%
stepDuration = [];
stepLength = [];
vertOscillation_vel_amp = [];
vertOscillation_dist_amp = [];
speed = [];
speedChange = [];
stepVels_d_aray=[];
oxyTest=[];

%disp('v_lon');
%length(v_lon);

for i = 1:stepCount
    stepDuration = [stepDuration, (steps(i + 1) - steps(i))/Fs];
    stepVelsLon = v_lon(steps(i):steps(i + 1));
    stepVels_d = v_d(steps(i):steps(i + 1));
      
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

    time(i)=steps(i)/400;
    j_ = floor(time(i)/5)+1;
    oxyTest = [oxyTest, oxygen(j_)];
  
end
toc
% 
% nCut=4667;
% rat=nCut/length(speed);
% figure;
% plot(speed);
% n = ceil(numel(speed)*rat);
% 
% B=speed(1:n);
% C=speed(n+1:end);

%figure
%plot(B)
%figure
%plot(C)
%figure
%plot(speedChange)
%figure
%plot(stepDuration)
%figure
%plot(vertOscillation_dist_amp)
%figure
%plot(oxyTest)

resultTest=[speed;speedChange;stepDuration;vertOscillation_dist_amp;oxyTest];
fileName=['velSeg_O',num2str(subjectId),'_out.mat'];
save(fileName,'steps', 'speed', 'speedChange', 'stepDuration', 'vertOscillation_dist_amp', 'oxyTest');
end
% resultWalk=resultTest(:,1:n);
% resultRun=resultTest(:,n+1:end);