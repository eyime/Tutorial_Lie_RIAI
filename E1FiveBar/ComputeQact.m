function [ q, qp, qpp ]= ComputeQact( t )
% This function computes the actuator function for a five bar mechanism
%
% Eugenio Yime Rodriguez
% 2020

% Sinosoidal motion
% theta2 = theta20 + Amplitude2*sin(period*time)
% theta5 = theta50 + Amplitude5*sin(period*time)

Amp = 20;
frec = 1;

% Position
q2 = 120 + Amp*sin( 2*pi*frec*t );
q5 = 60 - Amp*sin( 2*pi*frec*t );
% Velocity
qp2 =  2*pi*frec*Amp*cos( 2*pi*frec*t );
qp5 = -2*pi*frec*Amp*cos( 2*pi*frec*t );
% Acceleration
qpp2 = -(2*pi*frec)^2*Amp*sin( 2*pi*frec*t );
qpp5 =  (2*pi*frec)^2*Amp*sin( 2*pi*frec*t );

% Output
q = [q2; q5 ]*pi/180;
qp = [qp2; qp5]*pi/180;
qpp = [qpp2; qpp5]*pi/180;

return
