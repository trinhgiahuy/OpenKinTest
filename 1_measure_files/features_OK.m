function [] = features_OK(data,subjectId)
% % Walking/running data processing
% data = cell_array_parser_vn200(fileID,'vn200');
% data = cell_array_parser_vn200('2017-11-15-11-39-39-gpsreferenced.txt','vn200')
% data(k): time, d, gps, imu, pozyx
%t0=200000;
t0=1;
tend=max(size(data));
rad2deg=180/pi;
%tend=250000;
%fsmgaitv2.m
for k=t0:tend
    %k=i-t0+1;
    V_n(k)=data(k).gps.v_n;
    V_e(k)=data(k).gps.v_e;
    V_d(k)=data(k).gps.v_d;

%     V_gps_n(k)=data(k).gps.raw_v_n;
%     V_gps_e(k)=data(k).gps.raw_v_e;
%     V_gps_d(k)=data(k).gps.raw_v_d;

    %Speed(k)=sqrt(V_n(k)^2+V_e(k)^2);
%     %Hdg(k)=atan2(V_n(k),V_e(k));
%     wx(k)=data(k).imu.ang_vel(1); % rad/sec
%     wy(k)=data(k).imu.ang_vel(2); % rad/sec
%     wz(k)=data(k).imu.ang_vel(3); % rad/sec

    q0(k)=data(k).imu.quat(1);
    q1(k)=data(k).imu.quat(2);
    q2(k)=data(k).imu.quat(3);
    q3(k)=data(k).imu.quat(4);

    acc_n(k)=data(k).imu.acc(1);
    acc_e(k)=data(k).imu.acc(2);
    acc_d(k)=data(k).imu.acc(3);

%     delVx(k) = data(k).d.vel(1);
%     delVy(k) = data(k).d.vel(2);
%     delVz(k) = data(k).d.vel(3);
%     delTx(k) = data(k).d.theta(1);
%     delTy(k) = data(k).d.theta(2);
%     delTz(k) = data(k).d.theta(3);

    % Ground track
    GrndTrack(k)=rad2deg*atan2(V_e(k),V_n(k));
    % Lateral velocity constraint V_y = 0 (straight line running?)
    V_lat(k) = V_e(k)*cosd(GrndTrack(k)) - V_n(k)*sind(GrndTrack(k));
    % IMU horizontal speed
    V_lon(k) = sqrt(V_n(k)^2 + V_e(k)^2);
    % GPS horizontal speed
    %V_gps_lon(k) = sqrt(V_gps_n(k)^2 + V_gps_e(k)^2);
    % Longitudinal acceleration
    A_lon(k) = sqrt(acc_n(k)^2 + acc_e(k)^2);
    % Euler angle computation
    %dcm_xsens = quat2dcm( [q0(k) q1(k) q2(k) q3(k)] );
    [yaw(k) pitch(k) roll(k)] = quat2angle( [q3(k) q0(k) q1(k) q2(k)] );
%     [yaw(k) pitch(k) roll(k)] = quat2angle( [q0(k) q1(k) q2(k) q3(k)] );
    %V_ref(k,:)= data(k).d.vel;
end

subjOutputDir=['O',num2str(subjectId),'_out\'];
save([subjOutputDir,'acc.mat'], 'acc_n', 'acc_e','acc_d','A_lon');
save([subjOutputDir,'vel_imu.mat'], 'V_n', 'V_e','V_d','V_lon','V_lat','GrndTrack');
save([subjOutputDir,'euler.mat'], 'yaw', 'pitch', 'roll');
% save('ang_vel.mat', 'wx', 'wy', 'wz')
% save('quat.mat', 'q0', 'q1', 'q2','q3')
% save('delV.mat', 'delVx', 'delVy', 'delVz') 
% save('delT.mat', 'delTx', 'delTy', 'delTz') 



% figure(1)
% plot(t0:tend,A_lon(t0:tend))
% hold
% plot(t0:tend,acc_d(t0:tend))
% title('Longitudinal and vertical acceleration')
% xlabel('time, *0.005 sec')
% ylabel('acceleration, m/s^2')
% legend('longitudinal acceleration','vertical acceleration')
% grid
% 
% figure(2)
% plot(t0:tend,V_lon(t0:tend))
% hold
% plot(t0:tend,V_d(t0:tend))
% plot(t0:tend,V_gps_lon(t0:tend),':')
% title('Longitudinal and vertical velocity')
% xlabel('time, *0.005 sec')
% ylabel('Longitudinal and vertical velocity')
% legend('Longitudinal velocity INS', 'Vertical velocity', 'Longitudinal velocity GPS')
% grid
% 
% figure(3)
% plot(t0:tend,GrndTrack(t0:tend))
% title('Ground track')
% xlabel('time, *0.005 sec')
% ylabel('Ground track, deg')
% grid
% 
% figure(4)
% plot(t0:tend,rad2deg*yaw(t0:tend), t0:tend,rad2deg*pitch(t0:tend), t0:tend,rad2deg*roll(t0:tend))
% title('Upper body posture')
% xlabel('time, *0.005 sec')
% ylabel('yaw, pitch, roll, deg')
% legend('yaw', 'pitch', 'roll')
% grid
% 
% figure(5)
% plot(t0:tend,V_ref(t0:tend,1), t0:tend,V_ref(t0:tend,2), t0:tend,V_ref(t0:tend,3))

end