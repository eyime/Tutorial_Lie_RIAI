% Main function
clc
clear
close all

% Length data
L1 = 150;
L2 = 50;
L3 = 120;
L4 = 120;
L = [L1; L2; L3; L4];

% Real Solution - Spatial Four Bar
% t2 = 90, t4 = 112.77, t3z = 0, t3y = 97.58

% Initial guess data
% Position
t1 = 90;
t2 = 112.77;
t3 = 0.0;
t4 = 97.58;
% Initial position
q = [t1; t2; t3; t4]*pi/180;
% Initial Velocity
qp = zeros(4,1);
% Initial acceleration
qpp = zeros(4,1);

% Final time
tf = 60;

% Compute data
dt = 0.01;
t_data = 0:dt:tf;
nsteps = size(t_data, 2);

% Store data
q_data = zeros(4, nsteps);
qp_data = zeros(4, nsteps);
qpp_data = zeros(4, nsteps);
taui_data = zeros(1, nsteps);

% NR total counter
NRcounter = 0;

% main loop
for i=1:nsteps

    % time
    t = t_data(i);

    % Actuator
    [ qact, qpact, qppact ] = FunctionQact( t );

    % Asign active values
    % Position
    q(1) = qact;
    % Velocity
    qp(1) = qpact;
    % Acceleration
    qpp(1) = qppact;

    % While
    Fi = 1;
    count = 0;
    while ( ( max( abs(Fi) ) > 0.001 ) && (count < 200) )
        % compute T
        Tij = ComputeTij( q, L );

        % Copmute Fi
        Fi = ComputeFi( Tij );

        % Compute jacobian
        Jtheta = ComputeJacobian( Tij, q );
        % Passive Jacobian
        Jpas = Jtheta(:, 2:4);

        % Compute deltaq
        deltaqpas = -(Jpas \ Fi);

        % next step, passive joints
        % q(1) - Active (rotational joint)
        % q(2) - Passive (rotational joint)
        % q(3:4) - Passive (Universal joint)
        q = q + [0; deltaqpas];

        % counter
        count = count + 1;
    end

    % counter error
    if ( count >= 200 )
        count, i
        error('NR not converge')
    end

    % update NR counter
    NRcounter = NRcounter + count;

    % compute T
    Tij = ComputeTij( q, L );

    % Velocity correction
    % Compute jacobian
    Jtheta = ComputeJacobian( Tij, q );
    % Active and Passive jacobian
    Jact = Jtheta(:,1);
    Jpas = Jtheta(:,2:4);
    % new passive velocities
    qp_pas = - (Jpas \ (Jact*qp(1)) );
    % New corrected velocity
    qp(2:4,1) = qp_pas;

    % Acceleration Correction
    % Compute gamma vector
    Gamma = ComputeGamma( Tij, q, qp );
    % Jpas*qpp_pas + Jact*qpp_act = Gamma
    qpp_pas = Jpas \ (Gamma - Jact*qpp(1));
    % New corrected acceleration
    qpp(2:4,1) = qpp_pas;

    % Compute Ji for active forces
    Ji = ComputeJi( Tij, q, Jact, Jpas );

    % Compute active forces
    Taui = ComputeTaui( Tij, q, qp, qpp, Ji, L );

    % Store solution
    q_data(:,i) = q;
    qp_data(:,i) = qp;
    qpp_data(:,i) = qpp;
    taui_data(i) = Taui;

    % Next step
    q = q + qp*dt + 0.5*qpp.^2*dt;
end

% disp NR iterations
disp( 'Mean NR iterations:' )
NRcounter/nsteps

% few data
a = 1:50:nsteps;

% Plot Results
figure
plot( t_data(a), q_data(2,a)*180/pi )
hold on
plot( t_data(a), q_data(3,a)*180/pi, '--' )
plot( t_data(a), q_data(4,a)*180/pi, '.-' )
grid on
legend('\theta_4', '\theta_{3,z}', '\theta_{3,y}')
title('Grafica de posicion angular')
xlabel('tiempo (s)')
ylabel('ángulo (grados)')

% Velocity results
figure
plot( t_data(a), qp_data(2,a)*180/pi )
hold on
plot( t_data(a), qp_data(3,a)*180/pi, '--' )
plot( t_data(a), qp_data(4,a)*180/pi, '.-' )
grid on
legend('\omega_4', '\omega_{3,z}', '\omega_{3,y}')
title('Grafica de velocidad')
xlabel('tiempo (s)')
ylabel('velocidad (grados/s)')

% Acceleration results
figure
plot( t_data(a), qpp_data(2,a)*180/pi )
hold on
plot( t_data(a), qpp_data(3,a)*180/pi, '--' )
plot( t_data(a), qpp_data(4,a)*180/pi, '.-' )
grid on
legend('\alpha_4', '\alpha_{3,z}', '\alpha_{3,y}')
title('Grafica de aceleración')
xlabel('tiempo (s)')
ylabel('velocidad (grados/s^2)')

% torque results
figure
plot( t_data, taui_data )
grid on
title('Grafica de par de entrada')
xlabel('tiempo (s)')
ylabel('Par (N m)')
