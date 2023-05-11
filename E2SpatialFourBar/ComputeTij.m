function Tij = ComputeTij( q, L )

% angles
t2 = q(1);
t4 = q(2);
t3a = q(3);
t3b = q(4);

% lengths
L1 = L(1);
L2 = L(2);
L3 = L(3);
L4 = L(4);

% Update T
% Compute Tm
T02 = [ Roty(pi/2), zeros(3,1); zeros(1,3), 1 ]*[ Rotz(t2), zeros(3,1); zeros(1,3), 1 ];
T2m = [ eye(3), [L2;0;0]; zeros(1,3), 1 ];
T0m = T02*T2m;
 
% Compute Tn
T04 = [ eye(3), [L1;0;0]; zeros(1,3), 1 ]*[ Rotz(t4), zeros(3,1); zeros(1,3), 1 ];
T43 = [ eye(3), [L4;0;0]; zeros(1,3), 1 ]*[ Rotx(pi/2), zeros(3,1); zeros(1,3), 1 ]*[ Rotz(t3a)*Roty(t3b), zeros(3,1); zeros(1,3), 1 ];
T3n = [ eye(3), [L3;0;0]; zeros(1,3), 1 ];
T0n = T04*T43*T3n;

% output
% Left side
Tij.T02 = T02;
Tij.T2m = T2m;
Tij.T0m = T0m;
% Right side
Tij.T04 = T04;
Tij.T43 = T43;
Tij.T3n = T3n;
Tij.T0n = T0n;

return
