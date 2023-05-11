function Fi = ComputeFi( Tij )

% Posiciones absolutas
% Primera cadena cinematica
T0n1 = (Tij.T01)*(Tij.T12)*(Tij.T23)*(Tij.T3n1);
T0n2 = (Tij.T01)*(Tij.T12)*(Tij.T23)*(Tij.T3n2);
% Segunda cadena cinematica
T0m1 = (Tij.T04)*(Tij.T45)*(Tij.T5m1);
% Tercera cadena cinematica
T0m2 = (Tij.T06)*(Tij.T67)*(Tij.T7m2);

% Vectores posicion
rn1 = T0n1(1:3,4);
rn2 = T0n2(1:3,4);
rm1 = T0m1(1:3,4);
rm2 = T0m2(1:3,4);

% Vectores orientacion
uxx_n1 = T0n1(1:3,1); 
uyy_n1 = T0n1(1:3,2);
uxx_n2 = T0n2(1:3,1); 
uyy_n2 = T0n2(1:3,2);

% Fi function
Fi = [ uxx_n1'*( rn1 - rm1  ); uyy_n1'*( rn1 - rm1  ); ...
    uxx_n2'*( rn2 - rm2  ); uyy_n2'*( rn2 - rm2  ) ];

return

