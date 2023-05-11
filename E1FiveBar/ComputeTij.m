function Tij = ComputeTij(q, L)

% FIVE BAR

% angles
t2 = q(1);
t3 = q(2);
t4 = q(3);
t5 = q(4);

% lengths
L1 = L(1);
L2 = L(2);
L3 = L(3);
L4 = L(4);
L5 = L(5);

% Update T
% Compute Tm - left side
% T02 - between ground and after active joint, number 2.
T02 = [ Rotz(t2), [0;0;0]; zeros(1,3), 1];
% T2A - After active join and before passive joint
T2A = [ eye(3), [L2; 0; 0]; zeros(1,3), 1];
% TA3 - After passive joint, number 3
TA3 = [ Rotz(t3), [0;0;0]; zeros(1,3), 1];
% T3m - After passive joint and before m point.
T3m = [ eye(3), [L3; 0; 0]; zeros(1,3), 1];

% Compute Tn - right side
T01 = [ eye(3), [L1; 0; 0]; zeros(1,3), 1];
T15 = [ Rotz(t5), [0;0;0]; zeros(1,3), 1];
T5C = [ eye(3), [L5; 0; 0]; zeros(1,3), 1];
TC4 = [ Rotz(t4), [0;0;0]; zeros(1,3), 1];
T4n = [ eye(3), [L4; 0; 0]; zeros(1,3), 1];

% Output
% left side
Tij.T02 = T02;
Tij.T23 = T2A*TA3;
Tij.T3m = T3m;
% Right side
Tij.T05 = T01*T15;
Tij.T54 = T5C*TC4;
Tij.T4n = T4n;

return
