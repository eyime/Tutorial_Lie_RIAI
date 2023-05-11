function Gamma = ComputeGamma( Tij, qp )

% constants
% All joints are rotational
Si = [0, 0, 1, 0, 0, 0]';
% Cnn constraint
Cnn = [0, 0, 0, 1, 0, 0; 0, 0, 0, 0, 1, 0]';

% Computes Adjoints
% Primera cadena cinematica
% Primer Punto n1
T3n1 = (Tij.T3n1);
T2n1 = (Tij.T23)*T3n1;
T1n1 = (Tij.T12)*T2n1;
% global
T0n1 = (Tij.T01)*T1n1;
% Segundo Punto n2
T3n2 = (Tij.T3n2);
T2n2 = (Tij.T23)*T3n2;
T1n2 = (Tij.T12)*T2n2;
% global
T0n2 = (Tij.T01)*T1n2;

% Segunda cadena cinematica
T5m1 = (Tij.T5m1);
T4m1 = (Tij.T45)*T5m1;
% global
T0m1 = (Tij.T04)*T4m1;

% Tercera cadena cinematica
T7m2 = (Tij.T7m2);
T6m2 = (Tij.T67)*T7m2;
% global
T0m2 = (Tij.T06)*T6m2;

% Primer lazo cinematico
AdTn11 = AdTij( invT( T1n1 ) );
AdTn12 = AdTij( invT( T2n1 ) );
AdTn13 = AdTij( invT( T3n1 ) );
AdTn21 = AdTij( invT( T1n2 ) );
AdTn22 = AdTij( invT( T2n2 ) );
AdTn23 = AdTij( invT( T3n2 ) );
% Segundo lazo cinematico
AdTm14 = AdTij( invT( T4m1 ) );
AdTm15 = AdTij( invT( T5m1 ) );
% Tercer lazo cinematico
AdTm26 = AdTij( invT(T6m2 ) );
AdTm27 = AdTij( invT(T7m2 ) );
% De n a m
Tn1m1 = invT( T0n1 )* T0m1;
Tn2m2 = invT( T0n2 )* T0m2;
% Adjoints
AdTn1m1 = AdTij( Tn1m1 );
AdTn2m2 = AdTij( Tn2m2 );

% analisis aceleracion, terminos coriollis
Acn1 = adV(AdTn11*Si)*AdTn12*Si*qp(2)*qp(1) + adV(AdTn11*Si)*AdTn13*Si*qp(3)*qp(1) + ...
    adV(AdTn12*Si)*AdTn13*Si*qp(3)*qp(2);
Acn2 = adV(AdTn21*Si)*AdTn22*Si*qp(2)*qp(1) + adV(AdTn21*Si)*AdTn23*Si*qp(3)*qp(1) + ...
    adV(AdTn22*Si)*AdTn23*Si*qp(3)*qp(2);
Acm1 = adV(AdTm14*Si)*AdTm15*Si*qp(5)*qp(4);
Acm2 = adV(AdTm26*Si)*AdTm27*Si*qp(7)*qp(6);
% Velocidades al final de cada cadena cinematica abierta
Vn1 = AdTn11*Si*qp(1) + AdTn12*Si*qp(2) + AdTn13*Si*qp(3);
Vn2 = AdTn21*Si*qp(1) + AdTn22*Si*qp(2) + AdTn23*Si*qp(3);
Vm1 = AdTm14*Si*qp(4) + AdTm15*Si*qp(5);
Vm2 = AdTm26*Si*qp(6) + AdTm27*Si*qp(7);

% Vector Gamma
Gamma = zeros(4,1);
Gamma(1:2,1) = -Cnn'*( Acn1 - AdTn1m1*Acm1 + adV(Vn1)*AdTn1m1*Vm1 );
Gamma(3:4,1) = -Cnn'*( Acn2 - AdTn2m2*Acm2 + adV(Vn2)*AdTn2m2*Vm2 );

return

