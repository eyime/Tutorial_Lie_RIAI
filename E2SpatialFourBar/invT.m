function res = invT(T)
% This function compute 
% Tinv = inv(T)
%
% Where
% T = [R, r; 0, 1]
% invT = [R', -R'*r; 0, 1]
%
% Eugenio Yime Rodriguez
% 2020

R = T(1:3,1:3);
r = T(1:3,4);

res = [ R', -R'*r; 0, 0, 0, 1];

return
