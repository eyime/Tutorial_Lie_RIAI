function res = adV( V )

w = V(1:3,1);
v = V(4:6,1);

res = [ Skew3(w), zeros(3,3); Skew3(v), Skew3(w) ];

return

