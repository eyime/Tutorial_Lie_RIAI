function res = ComputeFi( Tij )
% This function computes the constraint vector Fi
% from homogeneus matrix stored in Tij.
%
% Kinematic chain located in left side
% T02 = Tij.T02
% T23 = Tij.T23
% T3m = Tij.T3m
%
% Kinematic chain located in right side
% T05 = Tij.T05
% T54 = Tij.T54
% T4n = Tij.T4n
%
% Eugenio Yime Rodriguez
% 2020

% FIVE BAR

% left side
T0m = (Tij.T02)*(Tij.T23)*(Tij.T3m);
% right Side
T0n = (Tij.T05)*(Tij.T54)*(Tij.T4n);

% Position vectors
rm = T0m(1:3, 4);
rn = T0n(1:3, 4);
Rn = T0n(1:3, 1:3);

% Fi vector
Fi = Rn'*( rn - rm );

% output
res = Fi(1:2,1);

return
