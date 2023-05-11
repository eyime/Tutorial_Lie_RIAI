function Ji = ComputeJi( Tij, Jact, Jpas )
% This function computes the jacobian link matrix Ji
% for each link in Five Bar Mechanism
%
% Eugenio Yime Rodriguez
% 2020

% constant
Si = [0, 0, 1, 0, 0, 0]';

% Adjoints
AdT32 = AdT( invT(Tij.T23) );
AdT45 = AdT( invT(Tij.T54) );

% Rho matrix
rho = -inv(Jpas)*Jact;

% Two active joints: theta1 and theta 4

% Left chain

% Body 2
J2 = zeros(6,2);
% only one active joint
J2(:,1) = Si;

% Body 3
J3 = zeros(6,2);
% First joint: active joint (2)
J3(:,1) = AdT32*Si;
% Second joint: passive joint (3)
J3 = J3 + Si*rho(1,:);

% Right chain

% Body 5
J5 = zeros(6,2);
% only one active joint (5)
J5(:,2) = Si;

% Body 4
J4 = zeros(6,2);
% First joint: active joint (5)
J4(:,2) = AdT45*Si;
% Second joint: passive joint (4)
J4 = J4 + Si*rho(2,:);

% Output
Ji.J2 = J2;
Ji.J3 = J3;
Ji.J4 = J4;
Ji.J5 = J5;

return
