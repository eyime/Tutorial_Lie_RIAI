function [th, thp, thpp] = FunctionQact( t )

% Actuator data
% Initial position: t1 = 120 degrees
% Angular velocity: t1p = 6 deg/s
% Angular acceleration: t1pp = 0 deg/s^2
t10 = 120*pi/180;
tp10 = 6*pi/180;
tpp10 = 0;

% output
th = t10 + tp10*t + 0.5*tpp10*t^2;
thp = tp10 + tpp10*t;
thpp = tpp10;

end
