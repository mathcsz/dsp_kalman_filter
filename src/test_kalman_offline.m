% 'off-line' Kalman Filter sensor fusion simuation.
% 

clear all

% Load sensors samples.
load('sensors_samples.m', 'gyr', 'acc');

Xsaved = zeros(length(gyr), 1);

for k=1:length(gyr)
  z1 = gyr(k);
  z2 = acc(k);
  
% Filtered signal.
  x = SimpleKalman(z1, z2, gyr, acc);
  
  Xsaved(k) = x;
end


figure
plot(Xsaved, 'o-')
hold on
plot(gyr, 'r:*') 
plot(acc, 'g:*') 
grid on
legend ("fusion", "gyro", "acc")