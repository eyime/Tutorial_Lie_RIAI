function Tij = ComputeTij( q, RobotData )

% Datos del robot
% Radio de la base: circunferencia de las juntas 
rbase = RobotData.rbase;
% Longitud de los eslabones
L     = RobotData.L;
% Lados del triangulo formado por las juntas en el efector final 
lee   = RobotData.lee;

% Matrices de transformacion homogenea

% Primera cadena cinematica
T01p = [ eye(3), [0; -rbase; 0]; zeros(1,3), 1 ];
T1p1 = [ Rotz(q(1)), zeros(3,1); zeros(1,3), 1 ];
T12p = [ eye(3), [L; 0; 0]; zeros(1,3), 1 ];
T2p2 = [ Rotz(q(2)), zeros(3,1); zeros(1,3), 1 ];
T23p = [ eye(3), [L; 0; 0]; zeros(1,3), 1 ];
T3p3 = [ Rotz(q(3)), zeros(3,1); zeros(1,3), 1 ];
T3n1 = [ eye(3), lee*[ cos(pi/3); sin(pi/3); 0]; zeros(1,3), 1 ];
T3n2 = [ eye(3), lee*[-cos(pi/3); sin(pi/3); 0]; zeros(1,3), 1 ];

% Segunda cadena cinematica
T04p = [ Rotz(2*pi/3), rbase*[cos(pi/6); sin(pi/6); 0]; zeros(1,3), 1 ];
T4p4 = [ Rotz(q(4)), zeros(3,1); zeros(1,3), 1 ];
T45p = [ eye(3), [L; 0; 0]; zeros(1,3), 1 ];
T5p5 = [ Rotz(q(5)), zeros(3,1); zeros(1,3), 1];
T5m1 = [ eye(3,3), [L; 0; 0]; zeros(1,3), 1];

% Tercera cadena cinematica
T06p = [ Rotz(-2*pi/3),  rbase*[-cos(pi/6); sin(pi/6); 0]; zeros(1,3), 1 ];
T6p6 = [ Rotz(q(6)), zeros(3,1); zeros(1,3), 1 ];
T67p = [ eye(3), [L; 0; 0]; zeros(1,3), 1 ];
T7p7 = [ Rotz(q(7)), zeros(3,1); zeros(1,3), 1 ];
T7m2 = [ eye(3), [L; 0; 0]; zeros(1,3), 1 ];

% Salida
% Primera cadena cinematica
Tij.T01 = T01p*T1p1;
Tij.T12 = T12p*T2p2;
Tij.T23 = T23p*T3p3;
Tij.T3n1 = T3n1;
Tij.T3n2 = T3n2;

% Segunda cadena cinematica
Tij.T04 = T04p*T4p4;
Tij.T45 = T45p*T5p5;
Tij.T5m1 = T5m1;

% Tercera cadena cinematica
Tij.T06 = T06p*T6p6;
Tij.T67 = T67p*T7p7;
Tij.T7m2 = T7m2;

return
