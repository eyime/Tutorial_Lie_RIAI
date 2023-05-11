function Jtheta = ComputeJacobian( Tij, q )

% q(1) - Rotational joint between ground and body 2
% q(2) - Rotational joint between ground and body 4
% q(3) and q(4) - Universal joint between body 4 and body 3

% Cnn constraint (x,y,z)
Cnn = [0, 0, 0, 1, 0, 0; ...
    0, 0, 0, 0, 1, 0; ...
    0, 0, 0, 0, 0, 1 ]';

% Restore Tij
T2m = (Tij.T2m);
T3n = (Tij.T3n);
T4n = (Tij.T43)*T3n;

% Compute Adjoints
AdTm2 = AdTij( invT(T2m) );
AdTn4 = AdTij( invT(T4n) );
AdTn3 = AdTij( invT(T3n) );
AdTnm = AdTij( invT(Tij.T0n)*Tij.T0m ); 

% updates S
% S2, S4 are equals: Rotational joint
Si = [0, 0, 1, 0, 0, 0]';
% Universal Joint
S3 = [-sin(q(4)), 0, cos(q(4)), 0, 0, 0; ...
    0, 1, 0, 0, 0, 0]';

% Jacobian
Jtheta = Cnn'*[ -AdTnm*AdTm2*Si, AdTn4*Si, AdTn3*S3 ];

return
