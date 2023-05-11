function res = AdTij( T )

R = T(1:3,1:3);
r = T(1:3, 4);

res = [ R, zeros(3,3); Skew3(r)*R, R ];

end
