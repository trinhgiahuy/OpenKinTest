% Vanilla LSTM example
% clear all
% close all
% % Sine function
% Xtrain = 1:500;
% Ytrue = sin(0.1*Xtrain);
% Ytrain = Ytrue + 0.01*randn(1,length(Xtrain));
% Xtest = 1:200;
% Ytest = sin(0.1*Xtest);
% Ytest_meas = Ytest + 0.01*randn(1,length(Xtest));


input_length = 10;
%X = cell(50,1);
%Xtrain = cell(1,30);
%Xtest = cell(10,1);

%for k = 1:30
 %   Xtrain{k} = k:(k+9);
  %  Ytrue(k) = sin(0.1*(k+9));
   % Ytrain(k) = Ytrue(k) + 0.2*randn;
%end

%Xtrain = X(1:30);
%Ytrain = Y(1:30);

%Xtest = X(31:40);
%Ytest = Y(31:40);

% LSTM net
% Layers
numFeatures = 2;
numHiddenUnits = 150;
numResponses = 1;

layers = [ ...
    sequenceInputLayer(numFeatures)
    lstmLayer(numHiddenUnits) %,'OutputMode','last')
    fullyConnectedLayer(numResponses)
    regressionLayer];

% Options
maxEpochs = 5000;

options = trainingOptions('adam', ...
    'ExecutionEnvironment','cpu', ...
    'MaxEpochs',maxEpochs, ...
    'SequenceLength','longest', ...
    'Shuffle','never', ...
    'Verbose',0, ...
    'Plots','training-progress');

% Network training
net = trainNetwork(Xtrain,Ytrain,layers,options);

Ypred = predict(net, 1:300)




