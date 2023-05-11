function res = CadV( V )

w = V(1:3,1);
v = V(4:6,1);

res = [ Skew3(w), Skew3(v); zeros(3,3), Skew3(w) ];

return

