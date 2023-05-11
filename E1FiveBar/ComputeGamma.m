function Gamma = ComputeGamma( Tij, qp )
% This function computes the gamma vector for acceleration analysis
%
% Eugenio Yime Rodriguez
% 2020

% All joints are rotational
Si = [0, 0, 1, 0, 0, 0]';
% cnn. Constraints on X-Y plane
cnn = [0, 0, 0, 1, 0, 0; ...
       0, 0, 0, 0, 1, 0 ]';

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

% velocity on constraint point
Vm = AdTm2*Si*qp(1) + AdTm3*Si*qp(2);
Vn = AdTn5*Si*qp(4) + AdTn4*Si*qp(3);

% Gamma vector
% Coriollis vectors
Acm = adV(AdTm2*Si)*AdTm3*Si*qp(1)*qp(2);
Acn = adV(AdTn5*Si)*AdTn4*Si*qp(4)*qp(3);

% Gamma = -cnn'*( Acn - AdTnm*Acm + adV(Vn)*AdTnm*Vm )
Gamma = -cnn'*( Acn - AdTnm*Acm + adV(Vn)*AdTnm*Vm );

return
