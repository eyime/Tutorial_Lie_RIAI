function Ji = ComputeJi( Tij, Jact, Jpas )

% All joints are rotational
Si = [0, 0, 1, 0, 0, 0]';

% Rho
Rho = -inv( Jpas )*Jact;

% Matrices Adjoint
AdT21 = AdTij( invT(Tij.T12) );
AdT31 = AdTij( invT(Tij.T12*Tij.T23) );
AdT32 = AdTij( invT(Tij.T23) );
AdT54 = AdTij( invT(Tij.T45) );
AdT76 = AdTij( invT(Tij.T67) );

% Compute Ji

% Primera cadena cinematica
% ** Cuerpo 1
J1 = zeros(6,3);
% Primera junta es la junta activa no 1 (theta1)
J1(:,1) = Si;
% ** Cuerpo 2
J2 = zeros(6,3);
% Primera junta es una junta activa no 1 (theta1)
J2(:,1) = AdT21*Si;
% Segunda junta es una junta pasiva no 1 (theta2)
J2 = J2 + Si*Rho(1,:);
% ** Cuerpo 3
J3 = zeros(6,3);
% Primera junta es junta activa No 1 (theta1)
J3(:,1) = AdT31*Si;
% Segunda junta es junta pasiva No 1 (theta2)
J3 = J3 + AdT32*Si*Rho(1,:);
% Tercera junta es junta pasiva No 2 (theta3)
J3 = J3 + Si*Rho(2,:);

% Segunda cadena cinematica
% ** Cuerpo 4
J4 = zeros(6,3);
% Primera junta es una junta activa no 2 (theta4)
J4(:,2) = Si;
% ** Cuerpo 5
J5 = zeros(6,3);
% Primera junta es una junta activa no 2 (theta4)
J5(:,2) = AdT54*Si;
% Segunda junta es una junta pasiva no 3 (theta5)
J5 = J5 + Si*Rho(3,:);

% Tercera cadena cinematica
% ** Cuerpo 6
J6 = zeros(6,3);
% Primera junta es una junta activa no 3 (theta6)
J6(:,3) = Si;
% ** Cuerpo 7
J7 = zeros(6,3);
% Primera junta es una junta activa no 3 (theta6)
J7(:,3) = AdT76*Si;
% Segunda junta es una junta pasiva no 4 (theta7)
J7 = J7 + Si*Rho(4,:);

% Salida
Ji.J1 = J1;
Ji.J2 = J2;
Ji.J3 = J3;
Ji.J4 = J4;
Ji.J5 = J5;
Ji.J6 = J6;
Ji.J7 = J7;

return
