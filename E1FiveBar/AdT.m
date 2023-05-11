function res = AdT( T )
% This function compute the adjoint matrix from
% homogeneus matrix T
% T = [R, r; 0, 1]
% AdT = [R, 0; skew3(r)*R, R]
%
% Eugenio Yime Rodriguez
% 2020

R = T(1:3,1:3);
r = T(1:3, 4);

res = [ R, zeros(3,3); Skew3(r)*R, R ];

end
