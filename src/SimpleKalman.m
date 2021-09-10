% Simple Kalman Filter for sensor fusion
%
% In:
%   z1: Last measure of the sensor 1
%   z2: Last measure of the sensor 2
%   z1History: past measures of the sensor 1
%   z2History: past measures of the sensor 2
%
% Out:
%   o: Output sample
%
function o = SimpleKalman(z1, z2, z1History, z2History)
%
%
persistent A H Q #R 
persistent x P
persistent firstRun

if isempty(firstRun)
  
%% Model: constant
  A = 1;

%% Model variance.
%% Trade-off between response time and noise immunity. 
  Q = 0.4;
  
%% Combining the measurements from the two sensors
  H = [1;1];
  
  x = 0;
  P = 6;
  
  firstRun = 1; 
end
  
%% Obtaining the variance of sensor 
%% measurements from the history 
if length(z1History < 4)
%% Initial matrix (without history) 
  R = [4 0; 0 4];
else 
%% Matlab equivalent syntax: cov(z1History, z2History)
  R = cov([z1History(:) z2History(:)]);
end

z = [z1; z2];

xp = A*x;
Pp = A*P*A' + Q;

K = Pp*H'*inv(H*Pp*H' + R);

x = xp + K*(z - H*xp);
P = Pp - K*H*Pp;


o = x;