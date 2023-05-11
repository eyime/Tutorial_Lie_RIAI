function Gamma = ComputeGamma( Tij, q, qp )

% qp(1) - angular velocity (active rotational joint)
% qp(2) - angular velocity (passive rotational joint)
% qp(3) - angular velocity (passive first universal joint)
% qp(4) - angular velocity (passive second universal joint)

% constants

% For rotational joints (S2, S4)
Si = [0, 0, 1, 0, 0, 0]';
% Universal Joint
S3 = [-sin(q(4)), 0, cos(q(4)), 0, 0, 0; ...
    0, 1, 0, 0, 0, 0]';
% \dot{S}_i for universal joint
S3p = [-cos(q(4))*qp(4), 0, -sin(q(4))*qp(4), 0, 0, 0; ...
    0, 0, 0, 0, 0, 0]';
% Cnn constraint (x,y,z)
Cnn = [0, 0, 0, 1, 0, 0; ...
    0, 0, 0, 0, 1, 0; ...
    0, 0, 0, 0, 0, 1 ]';

% Adjoints
AdTm2 = AdTij( invT(Tij.T2m) );
AdTn3 = AdTij( invT(Tij.T3n) );
AdTn4 = AdTij( invT((Tij.T43)*(Tij.T3n)) );
AdTnm = AdTij( invT(Tij.T0n)*(Tij.T0m) ); 

% velocity on constraint point
Vm = AdTm2*Si*qp(1);
Vn = AdTn4*Si*qp(2) + AdTn3*S3*[qp(3); qp(4)];

% Coriollis terms
% Left size
% Acm = zeros(6,1);

% Right Size
Acn = AdTn3*S3p*[qp(3); qp(4)] + ...
    adV(AdTn4*Si)*AdTn3*S3(:,1)*qp(2)*qp(3) + ...
    adV(AdTn4*Si)*AdTn3*S3(:,2)*qp(2)*qp(4) + ...
    AdTn3*adV(S3(:,1))*S3(:,2)*qp(3)*qp(4);

% Output
% Gamma = -cnn'*( Acn - AdTnm*Acm + adV(Vn)*AdTnm*Vm )
Gamma = -Cnn'*( Acn + adV(Vn)*AdTnm*Vm );

return
