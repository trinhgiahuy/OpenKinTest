% Multiple features
% Start from the step 
clear inputFeatures
clear oxygen
clear targetFeaters

oxygen = csvread('O1-out\O1_VO2_output_smooth.csv')

step0 = 50;
stepEnd = length(speed);
examples = stepEnd - step0;
numTimeSteps = 50;
inputFeatures = cell(1,examples);
targetFeatures = zeros(1,examples,1);

for k=1:(examples-numTimeSteps)
      
   time(k) = steps(k)/400;
   sequence = (step0+k):(step0+k+numTimeSteps-1);
   inputFeatures{k}(1,:) =  speed(sequence);
   inputFeatures{k}(2,:) =  speedChange(sequence);
   inputFeatures{k}(3,:) =  stepDuration(sequence);
   inputFeatures{k}(4,:) =  vertOscillation_dist_amp(sequence);
   
   j = floor(time(k)/5)+1;
   targetFeatures(k) = oxygen(j);
  
end



[idxTrain,idxTest] = trainingPartitions(examples-numTimeSteps,[0.8 0.2]);

XTrain = inputFeatures(idxTrain);
%XValidation = inputFeatures(idxValidation);
XTest = inputFeatures(idxTest);

TTrain = targetFeatures(idxTrain);
%TValidation = targetFeatures(idxValidation);
TTest = targetFeatures(idxTest); 
for k=1:length(XTrain)
    XTrain1{k} = XTrain{k}(1,:);
end

for k=1:length(XTest)
    XTest1{k} = XTest{k}(1,:);
end

save inputFeatures.mat XTrain XTest TTrain TTest