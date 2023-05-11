function taui = ComputeTaui( Vi, Ai, Ji, L )

% Datos del robot
La = L(2);
Lb = L(3);

% Short link
% Mass
ma = 8.29e-3; % Kg
% Inertia (Kg*m^2)
Ixxa = 35e-9;
Iyya = 1259.69e-9;  % ma*La^2/12;
Izza = Iyya;
% Mass Matrix
rca = [La/2; 0; 0];
Rca = eye(3);
Ica = diag([Ixxa, Iyya, Izza]);
% no centroidal mass matrix
Ma = [ Rca*Ica*Rca' - ma*Skew3(rca)*Skew3(rca),  ma*Skew3(rca); ...
    -ma*Skew3(rca), ma*eye(3) ];

% Large link
mb = 20.14e-3; % Kg
% centroidal inertia (Kg*m^2)
Ixxb = 84.16e-9;
Iyyb = 17553.4e-9;  % mb*Lb^2/12;
Izzb = Iyyb;
% Mass Matrix
rcb = [Lb/2; 0; 0];
Rcb = eye(3);
Icb = diag([Ixxb, Iyyb, Izzb]);
% no centroidal mass matrix
Mb = [ Rcb*Icb*Rcb' - mb*Skew3(rcb)*Skew3(rcb),  mb*Skew3(rcb); ...
    -mb*Skew3(rcb), mb*eye(3) ];

% For each link
% Body 2
V2 = Vi.V2;
A2 = Ai.A2;
J2 = Ji.J2;
taui = J2'*(Ma*A2 + CadV(V2)*Ma*V2);
% Body 3
V3 = Vi.V3;
A3 = Ai.A3;
J3 = Ji.J3;
taui = taui + J3'*(Mb*A3 + CadV(V3)*Mb*V3);

% Body 4
V4 = Vi.V4;
A4 = Ai.A4;
J4 = Ji.J4;
taui = taui + J4'*(Mb*A4 + CadV(V4)*Mb*V4);
% Body 5
V5 = Vi.V5;
A5 = Ai.A5;
J5 = Ji.J5;
taui = taui + J5'*(Ma*A5 + CadV(V5)*Ma*V5);

return
