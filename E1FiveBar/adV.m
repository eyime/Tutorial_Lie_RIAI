function res = adV( V )
% This function compute the adjoint velocity matrix from 
% velocity vector V
%
% V = [ w; v ]
% adV = [ skew(w), 0; skew(v), skew(w) ]
%
% Eugenio Yime Rodriguez
% 2020


w = V(1:3,1);
v = V(4:6,1);

res = [ Skew3(w), zeros(3,3); Skew3(v), Skew3(w) ];

return

