function [] = features_OK(data,subjectId)
t0=1;
tend=max(size(data));

for k=t0:tend
   
    V_n(k)=data(k).gps.v_n;
    V_e(k)=data(k).gps.v_e;
    V_d(k)=data(k).gps.v_d;

    % IMU horizontal speed
    V_lon(k) = sqrt(V_n(k)^2 + V_e(k)^2);

end

subjOutputDir=['O',num2str(subjectId),'_out\'];
save([subjOutputDir,'vel_imu.mat'],'V_d','V_lon');

end