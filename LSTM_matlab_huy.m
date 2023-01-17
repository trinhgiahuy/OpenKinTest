% Many-to-one LSTM for oxygen uptake prediction
clear all
close all

load inputFeatures11_sorted.mat
%load LSTM_results_8.mat

% Input features for each step: speed, speedChange, step Duration
% verticalOscillations
% Input sequence length 50 steps

%input_length = 10; % 10 

% LSTM net
% Layers
numFeatures = 4;
%size(XTest{1},1); % 4 input features
numHiddenUnits1 = 125;
numHiddenUnits2 = 150;
numResponses = 1; % one target feature

layers = [ ...
    sequenceInputLayer(numFeatures, Normalization="rescale-symmetric")
    %lstmLayer(numHiddenUnits1,'OutputMode','sequence')
    %dropoutLayer(0.2)
    lstmLayer(numHiddenUnits2,'OutputMode','last')
    %dropoutLayer(0.2)
    fullyConnectedLayer(numResponses)
    regressionLayer];

%     sequenceInputLayer(numFeatures)
%     lstmLayer(numHiddenUnits1,'OutputMode','sequence')
%     dropoutLayer(0.2)
%     lstmLayer(numHiddenUnits2,'OutputMode','last')
%     dropoutLayer(0.2)
%     fullyConnectedLayer(numClasses)
    
 

% Options
maxEpochs = 8000;

% options = trainingOptions('adam', ...
%     'ExecutionEnvironment','cpu', ...
%     'MaxEpochs',maxEpochs, ...
%     'SequenceLength','longest', ...
%     'Shuffle','never', ...
%     'Verbose',0, ...
%     'Plots','training-progress');

options = trainingOptions("adam", ...
    'ExecutionEnvironment','cpu', ...
    'MaxEpochs',maxEpochs, ...
    InitialLearnRate=0.005, ...
    SequenceLength="shortest", ...
    Plots="training-progress", ...
    Verbose= false);

% one input feature
for k=1:length(XTrain)
    XTrain1{k} = XTrain{k}(1:4,:);
end

for k=1:length(XTest)
    XTest1{k} = XTest{k}(1:4,:);
end

% Network training
net = trainNetwork(XTrain1',TTrain',layers,options);

Ypred = predict(net, XTest1);

rmse=sqrt(mean((Ypred(:)-TTest(:)).^2));

[Tsorted idxY] = sort(TTest);
Ysorted = Ypred(idxY);

%plot(1:length(Ypred), Ypred,'or', 1:length(TTest), TTest,'*b')
plot(1:length(Ypred), Ysorted,'or', 1:length(TTest), Tsorted,'*b')

title('Oxygen uptake prediction for participant #1, LSTM with 4 input features')
legend('predicted','measured')
xlabel('steps')
ylabel('oxygen uptake, ml/min/kg')
grid
% 
title('Bland-Altman plot for participant #1')
legend('VO2 difference','mean','standard deviation')
xlabel('oxygen uptake, ml/min/kg')
ylabel('measured - predicted , ml/min/kg')
grid
networkSave = net;
save ('model_11_sorted_speed_only.mat','networkSave')
Ypred = Ypred';
save ('output_11_speed_only.mat','Ypred','TTest')
save LSTM_results_11_speed_only.mat net Ypred
