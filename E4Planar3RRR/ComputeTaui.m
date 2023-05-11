function taui = ComputeTaui(Vi, Ai, Ji, RobotData)

% Datos del robot
L   = RobotData.L;
ree = RobotData.ree;
hee = 1.5*ree;

% Para los eslabones
% Masa
m1 = 0.33; % Kg
% Inercia centroidad (Kg*m^2)
Ixx1 = 0;
Iyy1 = m1*L^2/12;
Izz1 = m1*L^2/12;

% Para el efector final
m2 = 0.7; % Kg
% inercia centroidad (Kg*m^2)
Ixx2 = 1/18*m2*hee^2; 
Iyy2 = 1/18*m2*hee^2;
Izz2 = 1/9*m2*hee^2;

% Matriz de masas de los eslabones
% Eslabones
rc1 = [L/2; 0; 0];
Rc1 = eye(3);
Ic1 = diag([Ixx1, Iyy1, Izz1]);
% Matriz no centroidal
Mi = [ Rc1*Ic1*Rc1' - m1*Skew3(rc1)*Skew3(rc1),  m1*Skew3(rc1); ...
    -m1*Skew3(rc1), m1*eye(3) ];

% Matriz de masas del efector final
% la centroidad visto desde el sistema del cuerpo
rc2 = [0; ree; 0];
Rc2 = eye(3);
Ic2 = diag([Ixx2, Iyy2, Izz2]);
% Matriz no centroidal
Mee = [ Rc2*Ic2*Rc2' - m2*Skew3(rc2)*Skew3(rc2),  m2*Skew3(rc2); ...
    -m2*Skew3(rc2), m2*eye(3) ];

% Para cada cuerpo,
% Cuerpo 1
V1 = Vi.V1;
A1 = Ai.A1;
J1 = Ji.J1;
taui = J1'*(Mi*A1 + CadV(V1)*Mi*V1);

% Cuerpo 2
V2 = Vi.V2;
A2 = Ai.A2;
J2 = Ji.J2;
taui = taui + J2'*(Mi*A2 + CadV(V2)*Mi*V2);

% Cuerpo 3 - Efector Final
V3 = Vi.V3;
A3 = Ai.A3;
J3 = Ji.J3;
taui = taui + J3'*(Mee*A3 + CadV(V3)*Mee*V3);

% Cuerpo 4
V4 = Vi.V4;
A4 = Ai.A4;
J4 = Ji.J4;
taui = taui + J4'*(Mi*A4 + CadV(V4)*Mi*V4);

% Cuerpo 5
V5 = Vi.V5;
A5 = Ai.A5;
J5 = Ji.J5;
taui = taui + J5'*(Mi*A5 + CadV(V5)*Mi*V5);

% Cuerpo 6
V6 = Vi.V6;
A6 = Ai.A6;
J6 = Ji.J6;
taui = taui + J6'*(Mi*A6 + CadV(V6)*Mi*V6);

% Cuerpo 7
V7 = Vi.V7;
A7 = Ai.A7;
J7 = Ji.J7;
taui = taui + J7'*(Mi*A7 + CadV(V7)*Mi*V7);

return
