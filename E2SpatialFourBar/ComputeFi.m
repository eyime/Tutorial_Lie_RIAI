function Fi = ComputeFi( Tij )

r0m = Tij.T0m(1:3, 4);
r0n = Tij.T0n(1:3, 4);

R0n = Tij.T0n(1:3,1:3);

% uxx, uyy and uzz are equal to I
% (R0n*I)' = R0n'
Fi = R0n'*( r0n - r0m );

return
