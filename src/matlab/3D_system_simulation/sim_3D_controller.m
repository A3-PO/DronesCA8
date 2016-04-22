%**************************************************************************
%
% sim_3D_controller.m - CA8 - DRONES
%
%**************************************************************************
%
% Group 832 - Control and Automation Msc.
% Aalborg University
% April 2016
%
%**************************************************************************
%
% DESCRIPTION:
%
% INPUTS:
%
% OUTPUTS:
%
%**************************************************************************
clear all; close all; clc;

%% Initial values
% Drone position
x_drone = 1:0.5:50;                     % The position X of the DRONE
y_drone = 50*ones(1,length(x_drone));   % The position Y of the DRONE
z_drone = 0.1*ones(1,length(x_drone));  % The position Z of the DRONE

% Ground station position
x_gs = 0*ones(1,length(x_drone));       % The position X of the GROUND STATION 
y_gs = 0*ones(1,length(x_drone));       % The position Y of the GROUND STATION
z_gs = 0*ones(1,length(x_drone));       % The position Z of the GROUND STATION

% Simulation variables
time = linspace(0,100,length(x_drone)); % Time
prec_d = 200;                           % Precision of the distance vector

% Plant and Controller constants
Ra  = 25.6;                % Resistor [Ohm]
La  = 14.8*10^(-3);        % Inductance [H.]
Kt  = 0.171;               % Torque constant [N*m/A]
Kb  = 0.171;               % Back-emf constant [V/(r/s)]
Je  = 3*10^(-6);           % Load attached to motor shaft [kg*m^2]
De  = 7.7*10^(-6);         % Damping behavior [N*m/(r/s)]

%% RUNNING Simulation of the model with the controller (dronesmodel.mdl)
sim('controller_v1')

%% Simulation data of angles
t = theta_d_vec.Time;                     % sample time
% theta_d = rad2deg(theta_d_vec.Data);      % current drone angle
% theta_gs = rad2deg(theta_gs_vec.Data);    % current ground station angle
% opt_theta_d = rad2deg(opt_theta_d.Data);    % optimal drone angle
% opt_theta_gs = rad2deg(opt_theta_gs.Data);  % optimal ground station angle

theta_d = theta_d_vec.Data;      % current drone angle
phi_d = phi_d_vec.Data;      % current drone angle
theta_gs = theta_gs_vec.Data;    % current ground station angle
phi_gs = phi_gs_vec.Data;    % current ground station angle
opt_theta_d = opt_theta_d.Data;    % optimal drone angle
opt_theta_gs = opt_theta_gs.Data;  % optimal ground station angle
opt_phi_d = opt_phi_d.Data;    % optimal drone angle
opt_phi_gs = opt_phi_gs.Data;  % optimal ground station angle
alpha_d = alpha_d.Data;
alpha_gs = alpha_gs.Data;
gamma_d = gamma_d.Data;
gamma_gs = gamma_gs.Data;

%% Plot current vs optimal angle of drone and ground station 
figure(1)
subplot(2,1,1);
plot(t,theta_d,'LineWidth',2);
hold on;
plot(t,opt_theta_d,'r');
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Drone','Optimal');
title('Theta Drone angle vs optimal');
grid on;
grid minor;

subplot(2,1,2);
plot(t,phi_d,'LineWidth',2);
hold on;
plot(t,opt_phi_d,'r');
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Drone','Optimal');
title('Phi Drone angle vs optimal');
grid on;
grid minor;

figure(2)
subplot(2,1,1);
plot(t,theta_gs,'LineWidth',2);
hold on;
plot(t,opt_theta_gs,'r');
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Ground station','Optimal');
title('Theta Ground station angle vs optimal');
grid on;
grid minor;

subplot(2,1,2);
plot(t,phi_gs,'LineWidth',2);
hold on;
plot(t,opt_phi_gs,'r');
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Ground station','Optimal');
title('Phi Ground station angle vs optimal');
grid on;
grid minor;

%% 2D Simulation of drone-groundstation scenario    
% for i=1:length(x_drone)
%     
%     % POINTING ANTENNA GS FRAME
%     x_start_gs = x_gs;
%     y_start_gs = y_gs;
%     x_end_gs = x_gs + x_gs/5*cos(theta_gs_vec.Data(i));
%     y_end_gs = y_gs + x_gs/5*sin(theta_gs_vec.Data(i));
% 
%     % POINTING ANTENNA DRONE FRAME
%     x_start_d(i) = x_drone(i);
%     y_start_d(i) = y_drone(i); 
%     x_end_d(i) = x_drone(i) + y_drone(i)/5*cos(theta_d_vec.Data(i));
%     y_end_d(i) = y_drone(i) + y_drone(i)/5*sin(theta_d_vec.Data(i));
% 
%     % LOS distance vector construction
%     [dxVector(i,:),dyVector(i,:),los_d] = LOS_distance(x_drone(i),...
%                                         y_drone(i),x_gs,y_gs,prec_d);
%     
%     % Small function to draw arrows
%     drawArrow = @(x,y) quiver(x(1),y(1),x(2)-x(1),y(2)-y(1)...
%                 ,'LineWidth',2.5,'MaxHeadSize',0.5);
% 
%     % PLOT
%     f1 = figure(2);
%     clf(f1);
%     hold on
%     plot(x_drone(i),y_drone(i),'o','LineWidth',2);
%     plot(x_gs,y_gs,'X','LineWidth',2);
%     drawArrow([x_start_gs,x_end_gs],[y_start_gs,y_end_gs]);
%     drawArrow([x_start_d(i),x_end_d(i)],[y_start_d(i),y_end_d(i)]);
%     plot(dxVector(i,:),dyVector(i,:),'LineWidth',1.5);
%     axis([0 100 0 50]);
%     grid on;
%     grid minor;
%     str = sprintf('Scenario Simulation 2D');
%     %     str = sprintf(strcat(str,'\n GS Antenna Gain: %.3f dB \n Drone Antenna Gain: %.3f dB',GSgain,Dgain);
%     title(str);
%     xlabel('X world axis');
%     ylabel('Y world axis');
%     legend('Drone','Ground Station','GS Frame','Drone Frame','LOS distance','Location','Best');
%     pause(0.1);
%     movegui(f1,'north');
% end

