function [pos_mean, velo_mean, covar] = uwb_filter(time, y, tagpos)
% UWB_FILTER
%
% Estimate position using distance measurements and Extended Kalman filter.
%
% Input:
%  time - 1 x n_times, timestamps in seconds
%  y - n_tags x n_times, distance measurements, missing measurements labeled NaN
%  tagpos - dim x n_tags, tag locations
%
% Output:
%  pos_mean - 2 x n_times, position estimates
%  velo_mean - NaN, do not use
%  covar - 2 x 2 x n_times, position&velocity covariance matrices
%
% Henri Nurminen 10.6.2016

d = 2;      % position dimension
q = (45)^2; % process noise parameter (regulates the rate of change in 
%q = (0.1)^2; % process noise parameter (regulates the rate of change in 
% position between two time instants)
mu = 0;     % measurement bias
r = (0.5*1e3)^2;  % measurement noise variance
%r = (5)^2;  % measurement noise variance

A = eye(d);
Q = @(dt) q*eye(d);%q*dt*eye(d);
h = @(x) sqrt(sum(bsxfun(@minus,x(1:d),tagpos(1:d,:)).^2,1))';
H = @(x) bsxfun(@rdivide, bsxfun(@minus,x(1:d),tagpos(1:d,:))', max(h(x),0.01));
R = r*eye(size(tagpos,2));

% Initial prior with Gauss-Newton:
dx = inf(d,1);
mm = zeros(d,1);
is_meas = ~isnan(y(:,1));
while norm(dx)>0.01
    hh = h(mm);
    HH = H(mm);
    hh = hh(is_meas);
    HH = HH(is_meas,1:d);
    
    dx = HH \ (y(is_meas,1)-hh);
    mm = mm + dx;
end
m0 = mm;
P0 = inv(HH'/R(is_meas,is_meas)*HH);
%P0 = [1 0; 0 1];

% Extended Kalman filter

nm = size(y,2);
mf = nan(d,nm);
Pf = nan(d,d,nm);
mf(:,1) = m0;
Pf(:,:,1) = P0;
time = [time(1), time];
% Forward filtering step:
for i = 2:nm
    dt = time(i)-time(i-1);
    mm = A*mf(:,i-1);
    Pm = A*Pf(:,:,i-1)*A' + Q(dt);
    
    % Missing measurements:
    hh = h(mm);
    HH = H(mm);
    is_meas = ~isnan(y(:,i));
    hh = hh(is_meas);
    HH = HH(is_meas,:);
    RR = R(is_meas,is_meas);
    
    K = Pm*HH'/(HH*Pm*HH'+RR);
    mf(:,i) = mm + K*(y(is_meas,i)-mu-hh);
    Pf(:,:,i) = (eye(d)-K*HH)*Pm*(eye(d)-K*HH)' + K*RR*K';
end

pos_mean = mf(1:d,:);
velo_mean = nan;
covar = Pf;