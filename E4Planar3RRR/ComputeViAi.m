function [ Vi, Ai ] = ComputeViAi( Tij, qp, qpp )

% All joints are rotational
Si = [0, 0, 1, 0, 0, 0]';

% Adjoints
AdT21 = AdTij( invT(Tij.T12) );
AdT32 = AdTij( invT(Tij.T23) );
AdT54 = AdTij( invT(Tij.T45) );
AdT76 = AdTij( invT(Tij.T67) );

% Velocidad y Aceleracion
% Cadena 1
% Cuerpo 1
V1 = Si*qp(1);
A1 = Si*qpp(1);
% Cuerpo 2
V2 = AdT21*V1 + Si*qp(2);
A2 = AdT21*A1 + adV(V2)*Si*qp(2) + Si*qpp(2);
% Cuerpo 3
V3 = AdT32*V2 + Si*qp(3);
A3 = AdT32*A2 + adV(V3)*Si*qp(3) + Si*qpp(3);

% Cadena 2
% Cuerpo 4
V4 = Si*qp(4);
A4 = Si*qpp(4);
% Cuerpo 5
V5 = AdT54*V4 + Si*qp(5);
A5 = AdT54*A4 + adV(V5)*Si*qp(5) + Si*qpp(5);

% Cadena 3
% Cuerpo 6
V6 = Si*qp(6);
A6 = Si*qpp(6);
% Cuerpo 7
V7 = AdT76*V6 + Si*qp(7);
A7 = AdT76*A6 + adV(V7)*Si*qp(7) + Si*qpp(7);

% Salida
% Vi
Vi.V1 = V1;
Vi.V2 = V2;
Vi.V3 = V3;
Vi.V4 = V4;
Vi.V5 = V5;
Vi.V6 = V6;
Vi.V7 = V7;
% Ai
Ai.A1 = A1;
Ai.A2 = A2;
Ai.A3 = A3;
Ai.A4 = A4;
Ai.A5 = A5;
Ai.A6 = A6;
Ai.A7 = A7;

return
