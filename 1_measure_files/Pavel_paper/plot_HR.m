HR_fn="2022-11-01_09-25-19_203130000581_HeartRateTestActivity_s.csv";
HRopts = detectImportOptions(HR_fn);
HRopt.DataLines = 1;
%HRopts.VariableNamesLine = 1;
HR_data_table = readtable(HR_fn,HRopts);

HR_data = table2array(HR_data_table(:,2));
timeStamp = table2array(HR_data_table(:,1));
timeDiff = diff(timeStamp)
figure,
plot(HR_data*15)
hold on
plot(timeDiff)