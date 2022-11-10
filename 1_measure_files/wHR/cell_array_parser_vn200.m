function [] = cell_array_parser_vn200(fileID,subjectId, vn200)

if ~exist('vn200', 'var')
    vn200 = false;
end

if (vn200)
    of = 8;
else
    of = 0;
end

tic

% read data to table, delimited by tab
T = readtable(fileID,'Delimiter','\t','ReadVariableNames',false);

if (~vn200)
    if (size(T,2) == 32+8)
        disp("Probably (old) vn200-data? add true as second argument or press to continue anyway");
        pause;
    elseif (size(T,2) == 47)
        disp("Probably vn200-data? add true as second argument or press to continue anyway");
        pause;
    end
end

if (vn200)
    if (size(T,2) == 47)
        % 7 more fields in new
        of = of+7;
    end
end

if (vn200)
    % Loop go inside this
    v_n = table2array(T(:,18));
    v_e = table2array(T(:,19));
    v_d = table2array(T(:,20));
    for k=1:max(size(T))
        V_lon(k) = sqrt(v_n(k)^2 + v_e(k)^2);
        V_d(k) = v_d(k); 
    end
else
	v_n = 1e-2.*table2array(T(:,18));
	v_e = 1e-2.*table2array(T(:,19));
	v_d = 1e-2.*table2array(T(:,20)); 
end

%% Plot original V_lon
%figure,
%plot(V_lon)
subjOutputDir=['O',num2str(subjectId),'_out\'];
save([subjOutputDir,'vel_imu.mat'],'V_d','V_lon');

toc
end