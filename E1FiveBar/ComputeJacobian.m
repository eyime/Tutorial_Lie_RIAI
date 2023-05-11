function Jtheta = ComputeJacobian( Tij )
% This function computes the jacobian matrix
% for fivebar mechanism
%
% Eugenio Yime Rodriguez
% 2020

% left side
T3m = (Tij.T3m);
T2m = (Tij.T23)*T3m;
T0m = (Tij.T02)*T2m;
% right Side
T4n = (Tij.T4n);
T5n = (Tij.T54)*T4n;
T0n = (Tij.T05)*T5n;

% Adjoints
AdTm2 = AdT( invT(T2m) );
AdTm3 = AdT( invT(T3m) );
AdTn4 = AdT( invT(T4n) );
AdTn5 = AdT( invT(T5n) );
AdTnm = AdT( invT(T0n)*T0m );

% constants
% All joints are rotational
Si = [0, 0, 1, 0, 0, 0]';
% cnn. Constraints on X-Y plane
cnn = [0, 0, 0, 1, 0, 0; ...
       0, 0, 0, 0, 1, 0 ]';

% Whole Jacobian
Jtheta = cnn'*[ -AdTnm*AdTm2*Si, -AdTnm*AdTm3*Si, AdTn4*Si, AdTn5*Si ];

return
