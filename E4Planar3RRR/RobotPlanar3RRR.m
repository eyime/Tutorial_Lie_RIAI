% Main function
clc
clear
close all

% Dimensiones del robot
% longitud de los eslabones (m)
L = 0.200;
% radio de la base (m)
rbase = 0.255;
% radio del efector final (m)
ree = 0.055;
% longitud entre juntas en el efector final
lee = sqrt( 2*ree^2 - 2*ree^2*cosd(120) );

% vector de datos geometricos
RobotData.L = L;
RobotData.rbase = rbase;
RobotData.ree = ree;
RobotData.lee = lee;

% Initial position
q_ini = [ 30; 120; -150; 30; 120; 30; 120 ]*pi/180;

% datos de simulacion
tf = 5.0;
dt = 0.01;

% Store data
t_data = 0:dt:tf;
nsteps = size(t_data, 2);
q_data = zeros(7, nsteps);
qp_data = zeros(7, nsteps);
qpp_data = zeros(7, nsteps);
taui_data = zeros(3, nsteps);

% Initial data
q = q_ini;
qp = zeros(size(q_ini));
qpp = zeros(size(q_ini));

% main loop
for i = 1:nsteps
   
    % actual time
    t = t_data(i);
    
    % Update active joints
    [th, thp, thpp] = FunctionQact( t );
    q(1) = th; q(4) = th; q(6) = th;
    
    % While
    Fi = 1;
    count = 0;
    while ( ( max( abs(Fi) ) > 1e-5 ) && (count < 200) )

        % compute T
        Tij = ComputeTij( q, RobotData );
        
        % Compute Fi
        Fi  = ComputeFi( Tij )
        
        % Compute Jacobian
        Jtheta = ComputeJacobian( Tij );
        
        % Jacobiano activos y pasivos
        Jpas = [ Jtheta(:,2), Jtheta(:,3), Jtheta(:,5), Jtheta(:,7) ];
        deltaqpas = -(Jpas \ Fi);
        
        % Corregir pasivas
        q(2,1) = q(2,1) + deltaqpas(1,1);
        q(3,1) = q(3,1) + deltaqpas(2,1);
        q(5,1) = q(5,1) + deltaqpas(3,1);
        q(7,1) = q(7,1) + deltaqpas(4,1);
        % 0 a 360
        %q = mod(q, 2*pi);
        
        % counter
        count = count + 1;
    end
    
    % counter error
    if ( count >= 200 )
        count, t
        error('NR not converge')
    end
    
    % Velocity correction
    % compute T
    Tij = ComputeTij( q, RobotData );
    
    % Computes jacobian
    Jtheta = ComputeJacobian( Tij );
    
    % Jacobiano de las activas
    Jact = [ Jtheta(:,1), Jtheta(:,4), Jtheta(:,6) ];
    % Jacobiano de las pasivas
    Jpas = [ Jtheta(:,2), Jtheta(:,3), Jtheta(:,5), Jtheta(:,7) ];
    % Calcular velocidades activas
    qpact = [ thp; thp; thp ];
    % Corregir las velocidades pasivas
    qppas = -( Jpas \ (Jact*qpact) );
    
    % vector velocidad
    qp = [ thp; qppas(1); qppas(2); thp; qppas(3); thp; qppas(4) ];
    
    % Corrigiendo aceleraciones pasivas
    Gamma = ComputeGamma( Tij, qp );
    % Aceleraciones activas
    qppact = [ thpp; thpp; thpp ];
    % Correccion de las aceleraciones pasivas
    qpppas = ( Jpas \ ( Gamma - Jact*qppact) );
    
    % vector aceleracion
    qpp = [ thpp; qpppas(1); qpppas(2); thpp; qpppas(3); thpp; qpppas(4) ];
    
    % Calcular Vi y Ai de los cuerpos
    [ Vi, Ai ] = ComputeViAi( Tij, qp, qpp );
    
    % Calcular Ji para las fuerzas activas
    Ji = ComputeJi( Tij, Jact, Jpas );
    
    % Calcular las fuerzas activas
    Taui = ComputeTaui( Vi, Ai, Ji, RobotData );
    
    % Guardar los datos
    q_data(:,i) = q;
    qp_data(:,i) = qp;
    qpp_data(:,i) = qpp;
    taui_data(:,i) = Taui;
    
    % Siguiente paso
    q = q + qp*dt + 0.5*qpp*dt*dt;
end


% few data
a = 1:5:nsteps;

% Plot Results
figure
plot( t_data(a), q_data(2,a)*180/pi )
hold on
plot( t_data(a), q_data(5,a)*180/pi, '--' )
plot( t_data(a), q_data(7,a)*180/pi, '.-' )
grid on
legend('\theta_2', '\theta_5', '\theta_7')
title('Grafica de posicion angular')
xlabel('tiempo (s)')
ylabel('ángulo (grados)')

% Velocity results
figure
plot( t_data(a), qp_data(2,a)*180/pi )
hold on
plot( t_data(a), qp_data(5,a)*180/pi, '--' )
plot( t_data(a), qp_data(7,a)*180/pi, '.-' )
grid on
legend('\omega_2', '\omega_5', '\omega_7')
title('Grafica de velocidad')
xlabel('tiempo (s)')
ylabel('velocidad (grados/s)')

% Acceleration results
figure
plot( t_data(a), qpp_data(2,a)*180/pi )
hold on
plot( t_data(a), qpp_data(5,a)*180/pi, '--' )
plot( t_data(a), qpp_data(7,a)*180/pi, '.-' )
grid on
legend('\alpha_2', '\alpha_5', '\alpha_7')
title('Grafica de aceleración')
xlabel('tiempo (s)')
ylabel('velocidad (grados/s^2)')

% torque results
figure
plot( t_data, taui_data(1,:)*1000 )
hold on
plot( t_data, taui_data(2,:)*1000, '--' )
plot( t_data, taui_data(3,:)*1000, '.-' )
grid on
legend('\tau_1', '\tau_4', '\tau_6')
title('Grafica de par de entrada')
xlabel('tiempo (s)')
ylabel('Par (N mm)')
