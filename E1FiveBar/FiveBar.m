% This is the main program
% FIVE BAR - MECHANISM

clc
clear
close all

% Norton nomenclature
% Length data (m)
L1 = 0.100;
L2 = 0.040;
L3 = 0.100;
L4 = 0.100;
L5 = 0.040;
% Length vector
L = [L1, L2, L3, L4, L5];

% Absolute angle solution
% t2 = 120, t3 = 45.57, t4 = 134.43, t5 = 60
% Using actual nomenclature
% q = [theta2, theta3, theta4, theta5]
% t2 = 120, t3 = 45.74 - 120, t4 = 134.43-60, t5 = 60

% increment
dt = 0.001;
% 1000 steps = 1 seconds
nsteps = 1000;

% initial position in angles
t2 = 120; t3 = -74.46; t4 =  74.46; t5 = 60;
% Initial position vector
q = [t2; t3; t4; t5]*pi/180;

% Store data
t_data = zeros(nsteps);
q_data = zeros(4, nsteps);
qp_data = zeros(4, nsteps);
qpp_data = zeros(4, nsteps);
taui_data = zeros(2, nsteps);

% NR total counter
NRcounter = 0;

%% MAIN LOOP
time = 0;
for i=1:nsteps

    % Compute actuator data
    [ q_act, qp_act, qpp_act ]= ComputeQact( time );

    % Update active join
    q(1) = q_act(1); q(4) = q_act(2);

    %% Position: Newton-Raphson correction
    Fi = 1;
    count = 0;
    while ( ( max( abs(Fi) ) > 0.001 ) && (count < 200) )

        % compute T
        Tij = ComputeTij(q, L);

        % Jacobian
        Jtheta = ComputeJacobian( Tij );
        % Jacobian - [ (2), (3), (4), (5) ]
        % Passive Jacobian = [ (3) - (4) ]
        Jpas = [ Jtheta(:,2), Jtheta(:,3) ];

        % Copmute Fi
        Fi = ComputeFi( Tij );

        % Compute deltaq
        deltaqpas = -(Jpas \ Fi);

        % next step
        q(2:3) = q(2:3) + deltaqpas;
        %q = mod(q, 2*pi);

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

    %% Velocity

    % update Tij position
    Tij = ComputeTij( q, L );
    % Compute Jacobian
    Jtheta = ComputeJacobian( Tij );

    % Passive and active Jacobian
    Jpas = [ Jtheta(:,2), Jtheta(:,3) ];
    Jact = [ Jtheta(:,1), Jtheta(:,4) ];

    % Correct passive velocities
    qp_pas = - (Jpas \ (Jact*qp_act) );

    % update velocity
    qp = [qp_act(1); qp_pas; qp_act(2)];

    %% Acceleration

    % Compute gamma
    Gamma = ComputeGamma( Tij, qp );

    % Correct Acceleration
    % Jp*qpp_pas + Ja*qpp_act = Gamma
    qpp_pas = Jpas \ ( Gamma - Jact*qpp_act );

    % update acceleration
    qpp = [ qpp_act(1); qpp_pas; qpp_act(2) ];

    %% Inverse Dynamics
    % Compute Vi, Ai
    [ Vi, Ai ] = ComputeViAi( Tij, qp, qpp );

    % Compute Ji
    Ji = ComputeJi( Tij, Jact, Jpas );

    % Compute active torque
    Taui = ComputeTaui( Vi, Ai, Ji, L );

    %% Store data
    t_data(i) = time;
    q_data(:,i) = q;
    qp_data(:,i) = qp;
    qpp_data(:,i) = qpp;
    taui_data(:,i) = Taui;

    % Next step
    q = q + qp*dt + 0.5*qpp.^2*dt;
    time = time + dt;
end

% disp NR iterations
disp( 'Mean NR iterations:' )
NRcounter/nsteps

% Few Results
a = 1:5:size(t_data,2);

% Plot Results
figure
plot( t_data(a), q_data(1,a)*180/pi )
hold on
plot( t_data(a), q_data(2,a)*180/pi, '--' )
plot( t_data(a), q_data(3,a)*180/pi, '.-' )
plot( t_data(a), q_data(4,a)*180/pi, ':' )
grid on
legend('\theta_2', '\theta_3', '\theta_4', '\theta_5')
title('Grafica de posicion angular')
xlabel('tiempo (s)')
ylabel('ángulo (grados)')

% Velocity results
figure
plot( t_data(a), qp_data(1,a)*180/pi )
hold on
plot( t_data(a), qp_data(2,a)*180/pi, '--' )
plot( t_data(a), qp_data(3,a)*180/pi, '.-' )
plot( t_data(a), qp_data(4,a)*180/pi, ':' )
grid on
legend('\omega_2', '\omega_3', '\omega_4', '\omega_5')
title('Grafica de velocidad angular')
xlabel('tiempo (s)')
ylabel('velocidad (grados/s)')

% Acceleration results
figure
plot( t_data(a), qpp_data(1,a)*180/pi )
hold on
plot( t_data(a), qpp_data(2,a)*180/pi, '--' )
plot( t_data(a), qpp_data(3,a)*180/pi, '.-' )
plot( t_data(a), qpp_data(4,a)*180/pi, ':' )
grid on
legend('\alpha_2', '\alpha_3', '\alpha_4', '\alpha_5')
title('Grafica de aceleración angular')
xlabel('tiempo (s)')
ylabel('velocidad (grados/s^2)')

% torque results
figure
plot( t_data(a), taui_data(1,a)*1000 )
hold on
plot( t_data(a), taui_data(2,a)*1000, '--')
grid on
legend('\tau_2', '\tau_5')
title('Grafica de par de los motores')
xlabel('tiempo (s)')
ylabel('Par (N mm)')
