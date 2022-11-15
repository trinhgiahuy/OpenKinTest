function HRLinear = linearInter(HR_fn)
%HR_fn="2022-11-01_10-08-41_203130000581_HeartRateTestActivity_s.csv";
HRopts = detectImportOptions(HR_fn);
HRopt.DataLines = 1;
%HRopts.VariableNamesLine = 1;
HR_data_table = readtable(HR_fn,HRopts);

HR_data = table2array(HR_data_table(:,2));
timeStamp = table2array(HR_data_table(:,1));

HR_data_table(:,1) = array2table(timeStamp);

%firstEle=floor(timeStamp(1,:)/1000);
%lastEle=floor(timeStamp(end,:)/1000);
firstEle=timeStamp(1,:);
lastEle=timeStamp(end,:);

idx = (firstEle:1000/400:lastEle);

HRLinear = interp1(timeStamp,HR_data,idx);
%newHR = newHR(2:,:);
%numIdx=floor((lastEle-firstEle)/1000);
%timeDiff = diff(timeStamp);
%a=interp1(timeStamp,HR_data,idx,'linear')

figure,
%yyaxis left
%plot(HR_data);
%hold on
%yyaxis right
%plot(HRLinear);
%plot(timeDiff)
%plot(detrend(timeDiff))
end