function [outputArg1,outputArg2] = BlandAltman(inputArg1,inputArg2)

%   Detailed explanation goes here
S1 = inputArg1;
S2 = inputArg2;
Saver = 0.5 * (S1 + S2);
Sdiff = S1 - S2;

meanDiff = mean(Sdiff);
SD = std(Sdiff);

plot(Saver,Sdiff,'o')
hold
xm = [0 max(Saver)];
ym = [meanDiff meanDiff];
ysd1 = [meanDiff+2*SD meanDiff+2*SD];
ysd2 = [meanDiff-2*SD meanDiff-2*SD];
plot(xm, ym, xm, ysd1,'g', xm, ysd2,'g')
outputArg1 = meanDiff;
outputArg2 = SD;



outputArg1 = meanDiff;
outputArg2 = SD;
end

