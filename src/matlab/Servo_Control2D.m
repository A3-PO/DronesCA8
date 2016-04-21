%**************************************************************************
%               2D controller for DC servo motor
%**************************************************************************

clear all; close all;

%% Motor parameters
Ra  = 2.6;               % Resistor [Ohm]
La  = 180*10^(-6);       % Inductance [H]
Kt  = 7.67*10^(-3);      % Torque constant [N*m/A]
Kb  = 7.67*10^(-3);      % Back-emf constant [V/(r/s)]
Je  = 5.3*10^(-7);       % Load attached to motor shaft [kg*m^2]
De  = 7.7*10^(-6);       % Damping behavior [N*m/(r/s)]

%GI  = 1/(Ra + La*s)
%GO  = 1/(Je*s^2 + De*s)


% PID parameters
% The 'Good gain method' has been used to tune the PID
Kp  =  0.3;

Ti  = 0.2; %Kp/Ki
Td  = Ti/4; %Kd/Kp

