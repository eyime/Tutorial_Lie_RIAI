function res = CadV( V )
% This function compute the co-adjoint velocity matrix from 
% velocity vector V
%
% V = [ w; v ]
% adV* = [ skew(w), skew(v); 0, skew(w) ]
%
% Eugenio Yime Rodriguez
% 2020

w = V(1:3,1);
v = V(4:6,1);

res = [ Skew3(w), Skew3(v); zeros(3,3), Skew3(w) ];

return

