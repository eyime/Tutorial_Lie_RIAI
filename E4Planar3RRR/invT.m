function res = invT(T)

R = T(1:3,1:3);
r = T(1:3,4);

res = [ R', -R'*r; 0, 0, 0, 1];

return
