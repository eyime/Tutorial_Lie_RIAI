function [ Vi, Ai ] = ComputeViAi( Tij, qp, qpp )

% All joints are rotational
Si = [0, 0, 1, 0, 0, 0]';

T32 = invT( Tij.T23 );
T45 = invT( Tij.T54 );

AdT32 = AdT( T32 );
AdT45 = AdT( T45 );

% left side
% body 2
V2 = Si*qp(1);
A2 = Si*qpp(1);
% body 3
V3 = AdT32*V2 + Si*qp(2);
A3 = AdT32*A2 + Si*qpp(2) + adV(V3)*Si*qp(2);

% Right side
% body 5
V5 = Si*qp(4);
A5 = Si*qpp(4);
% body 4
V4 = AdT45*V5 + Si*qp(3);
A4 = AdT45*A5 + Si*qpp(3) + adV(V4)*Si*qp(3);

% Output
% Vi
Vi.V2 = V2;
Vi.V3 = V3;
Vi.V4 = V4;
Vi.V5 = V5;
% Ai
Ai.A2 = A2;
Ai.A3 = A3;
Ai.A4 = A4;
Ai.A5 = A5;

return
