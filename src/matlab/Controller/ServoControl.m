%**************************************************************************
% Comparison between PID, PD and PI controllers to keep the reference angle
% , taking into account the effects of noises and saturation box (limiting 
% the input voltage of the motor).
%
% For each device, drone and ground station, 2 controllers and 2 motors 
% will be needed to move in Φ and Θ directions (3D).
%
% To see the impact on the communication gains look at
% '3D_system_simulation' folder on git, where the diffent controllers 
% present here can be tested there.
% 
%**************************************************************************

 clear all; close all;

% Motor parameters
% Ra  = 2.6;               % Resistor [Ohm]
% La  = 180*10^(-6);       % Inductance [H]
% Kt  = 7.67*10^(-3);      % Torque constant [N*m/A]
% Kb  = 7.67*10^(-3);      % Back-emf constant [V/(r/s)]
% Je  = 5.3*10^(-7);       % Load attached to motor shaft [kg*m^2]
% De  = 7.7*10^(-6);       % Damping behavior [N*m/(r/s)]


Ra  = 25.6;                % Resistor [Ohm]
La  = 14.8*10^(-3);        % Inductance [H.]
Kt  = 0.171;               % Torque constant [N*m/A]
Kb  = 0.171;               % Back-emf constant [V/(r/s)]
Je  = 3*10^(-6);           % Load attached to motor shaft [kg*m^2]
De  = 7.7*10^(-6);         % Damping behavior [N*m/(r/s)]


% Plot the effects of PID, PD and PI on the same plant
[T,X,Y] = sim('Servo_Control');
phi_gs_PI = phi_gs_PI.Data;
phi_gs_PD = phi_gs_PD.Data;
phi_gs_PID = phi_gs_PID.Data;
figure();
plot(T,phi_gs_PI,T,phi_gs_PD,T,phi_gs_PID);
xlabel('Time [s]');
ylabel('Angle [rad]');
legend('PI','PD','PID');
title('Φgs for PID, PD, & PI controllers');
grid on;
grid minor;


