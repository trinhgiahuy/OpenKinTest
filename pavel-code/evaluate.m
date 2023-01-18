% Many-to-one LSTM for oxygen uptake prediction
clear all
close all
id='14';
motion4=true;
speedOnly=false;
HROnly=false;
wHR=false;


if motion4
    fn=['output_',id,'_4_motions.mat'];
    preFix = '_noHR';
end

if speedOnly
    fn=['output_',id,'_speed_only.mat'];
    preFix = '_speedOnly';
end

if HROnly
    fn=['output_',id,'_HR_only'];
    preFix = '_HRonly';
end

if wHR
    fn=['output_',id,'_wHR.mat'];
    preFix = '_wHR'
end

load(fn)

% Network training

Ypred = double(Ypred);

fileSave=['O_',id,'_predict',preFix,'.csv'];

diff = Ypred - TTest;
avg = 0.5*(Ypred + TTest);

save(fileSave,'Ypred','TTest','diff','avg');

rmse=sqrt(mean((Ypred(:)-TTest(:)).^2));

[Tsorted idxY] = sort(TTest);
Ysorted = Ypred(idxY);

%plot(1:length(Ypred), Ypred,'or', 1:length(TTest), TTest,'*b')

plot(1:length(Ypred), Ysorted,'or', 1:length(TTest), Tsorted,'*b')
%plot(1:length(Ypred), Ypred,'or', 1:length(TTest), Tsorted,'*b')


% title('Oxygen uptake prediction for participant', LSTM with 4 input features')
% legend('predicted','measured')
% xlabel('steps')
% ylabel('oxygen uptake, ml/min/kg')
grid
% 
title(['Bland-Altman plot for participant',id,'-',preFix(2:end)])
legend('VO2 difference','mean','standard deviation')
xlabel('oxygen uptake, ml/min/kg')
ylabel('measured - predicted , ml/min/kg')
grid
