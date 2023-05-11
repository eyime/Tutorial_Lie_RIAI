function Ji = ComputeJi( Tij, q, Jact, Jpas )

% updates S
% S2, S4 are equals: Rotational joint
Si = [0, 0, 1, 0, 0, 0]';
% S3 : Universal Joint
S3 = [-sin(q(4)), 0, cos(q(4)), 0, 0, 0; ...
    0, 1, 0, 0, 0, 0]';

% Rho
Rho = -inv( Jpas )*Jact;

% Adjoints
AdT34 = AdTij( invT(Tij.T43) );

% Left side
% Body 2
% Only one active joint, on body 2
J2 = Si;

% Right side
% Body 4
% First pasive joint
J4 = Si*Rho(1,:);
% Body 3
J3 = zeros(6,1);
% First passive joint
J3 = J3 + AdT34*Si*Rho(1,:);
% Second passive joint
J3 = J3 + S3*Rho(2:3,:);

% Output
Ji.J2 = J2;
Ji.J4 = J4;
Ji.J3 = J3;

return
