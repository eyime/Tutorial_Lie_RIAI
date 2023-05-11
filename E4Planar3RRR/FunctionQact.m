function [th, thp, thpp] = FunctionQact( t )

th   = pi/6 + (pi/30)*sin( (2*pi/5)*t );
thp  =  (pi/30)*(2*pi/5)*cos( (2*pi/5)*t );
thpp = -(pi/30)*(2*pi/5)^2*sin( (2*pi/5)*t );

%th = pi/6 + (6*pi/180)*t;
%thp = 6*pi/180;
%thpp = 0;

end
