function [] = input_features(subjectId,sorted)
oxygen = csvread(['O',num2str(subjectId),'_output_smooth.csv']);
load (['velSeg_O',num2str(subjectId),'_out.mat']);

%HRopts = detectImportOptions(HR_fn);
%HRopt.DataLines = 1;
%HRopts.VariableNamesLine = 1;
%HR_data_table = readtable(HR_fn,HRopts);
%HR_data = table2array(HR_data_table(:,2));

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
   
   %%TEST
   inputFeatures{k}(5,:) = HRTest(sequence);
   %i = floor(time(k)/1)+1;
   %i=k;
   %hr_sequence = (step0+i):(step0+i+numTimeSteps-1);

   %% HR data here should be in fixing sample rate
   %inputFeatures{k}(5,:) =  HR_data(hr_sequence);

   %disp('j');
   j = floor(time(k)/5)+1;
   targetFeatures(k) = oxygen(j);
  
end

[idxTrain,idxTest] = trainingPartitions(examples-numTimeSteps,[0.8 0.2]);

if sorted
    idxTrain=sort(idxTrain);
    idxTest=sort(idxTest);
    sortPrefix = '_sorted';
else
    sortPrefix = '';
end

XTrain = inputFeatures(idxTrain);
%XValidation = inputFeatures(idxValidation);

XTest = inputFeatures(idxTest);

TTrain = targetFeatures(idxTrain);
%TValidation = targetFeatures(idxValidation);
TTest = targetFeatures(idxTest);

fileName=['inputFeatures',num2str(subjectId),sortPrefix];
save(fileName,'XTrain','XTest','TTrain','TTest');
end