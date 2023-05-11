function Taui = ComputeTaui( Tij, q, qp, qpp, Ji, L )

% updates S
% S2, S4 are equals: Rotational joint
Si = [0, 0, 1, 0, 0, 0]';
% Universal Joint
S3 = [-sin(q(4)), 0, cos(q(4)), 0, 0, 0; ...
    0, 1, 0, 0, 0, 0]';
S3p = [-cos(q(4))*qp(4), 0, -sin(q(4))*qp(4), 0, 0, 0; ...
    0, 0, 0, 0, 0, 0]';

% masas
m2 = 0.1;
m4 = 0.4;
m3 = 0.4;

% Adjoints
AdT34 = AdTij( invT(Tij.T43) );

% Matriz de masas de los eslabones
% Eslabon 2
rc2 = [L(1)/2; 0; 0];
Rc2 = eye(3);
Ic2 = diag([ 0, 1/12*m2*L(1)^2, 1/12*m2*L(1)^2 ]);
% Matriz no centroidal
M2 = [ Rc2*Ic2*Rc2' - m2*Skew3(rc2)*Skew3(rc2),  m2*Skew3(rc2); ...
    -m2*Skew3(rc2), m2*eye(3) ];
% Velocidad
V2 = Si*qp(1);
% Aceleracion
A2 = Si*qpp(1);
% Par
Taui = (Ji.J2)'*( M2*A2 + CadV(V2)*M2*V2 );

% Eslabon 4
rc4 = [L(3)/2; 0; 0];
Rc4 = eye(3);
Ic4 = diag([ 0, 1/12*m4*L(3)^2, 1/12*m4*L(3)^2 ]);
% Matriz no centroidal
M4 = [ Rc4*Ic4*Rc4' - m4*Skew3(rc4)*Skew3(rc4),  m4*Skew3(rc4); ...
    -m4*Skew3(rc4), m4*eye(3) ];
% Velocidad
V4 = Si*qp(2);
% Aceleracion
A4 = Si*qpp(2);
% Par
Taui = Taui + (Ji.J4)'*(M4*A4 + CadV(V4)*M4*V4 );

% Eslabon 3
rc3 = [L(2)/2; 0; 0];
Rc3 = eye(3);
Ic3 = diag([ 0, 1/12*m3*L(2)^2, 1/12*m3*L(2)^2 ]);
% Matriz no centroidal
M3 = [ Rc3*Ic3*Rc3' - m3*Skew3(rc3)*Skew3(rc3),  m3*Skew3(rc3); ...
    -m3*Skew3(rc3), m3*eye(3) ];
% Velocidad
V3 = AdT34*V4 + S3*[qp(3); qp(4)];
% Aceleracion
A3 = AdT34*A4 + S3p*[qp(3); qp(4)] + S3*[qpp(3); qpp(4)] + adV(V3)*S3*[qp(3); qp(4)];
% Par
Taui = Taui + (Ji.J3)'*( M3*A3 + CadV(V3)*M3*V3 );

return
