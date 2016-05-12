%**************************************************************************
%               2D controller for DC servo motor
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


% GI  = 1/(Ra + La*s)
% GO  = 1/(Je*s^2 + De*s)

% PID parameters
% Kp  =  2;%0.3;

% Ti = 0.012*1.5;
% Ti  = Inf; %Kp/Ki
% Td  = Ti/4; %Kd/Kp

