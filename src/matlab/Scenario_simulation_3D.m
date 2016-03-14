%**************************************************************************
%
% Scenario_simulation_3D.m - CA8 - DRONES
%
%**************************************************************************
%
% Group 832 - Control and Automation Msc.
% Aalborg University
% February 2016
%
%**************************************************************************
%
% DESCRIPTION:
% Code to test and check out a STATIONARY SCENARIO of our drone and ground
% station in 3D
%
% Functions used during the code:
% - LOS_distance_3D.m
% - angle_frames.m
% - GSantenna.m
%
%**************************************************************************

clear all;
close all;
clc;

%% Enviroment parameters

step_x = 1;                 % Step of vector X [km]
step_y = 1;                 % Step of vector Y [km]
step_z = 0.01;              % Step of vector Z [km]
prec_d = 200;               % Precision of the distance vector

freq = 2.4 * 10^9;          % Frequency [Hz]
lambda = 3*10^8/freq;       % Wavelength [m]
Ptx = 10*log10(1/(10^-3));  % 1mW power transmiter

% Small function to draw arrows
drawArrow = @(x,y,z) quiver3(x(1),y(1),z(1),x(2)-x(1),y(2)-y(1),z(2)-z(1),'LineWidth',2.5,'MaxHeadSize',1.5);  

x_gs = 0;                  % The position X of the GROUND STATION
y_gs = 0;                   % The position Y of the GROUND STATION
z_gs = 0;                   % The position Z of the GROUND STATION
x_drone = 50;                % The position X of the DRONE
y_drone = 50;               % The position Y of the DRONE
z_drone = 100;              % The position Z of the DRONE

% LOS distance vector construction
[dxVector,dyVector,dzVector] = LOS_distance_3D(x_drone,y_drone,z_drone,x_gs,y_gs,z_gs,prec_d);
        
xVector = [min([x_gs x_drone]):step_x:max([x_gs x_drone])];    % World vector X [km]
yVector = [min([y_gs y_drone]):step_y:max([y_gs y_drone])];    % World Vector Y [km]
zVector = [min([z_gs z_drone]):step_z:max([z_gs z_drone])];    % World Vector Z [km]

%% Ground Station definition

% Polar angle: of the GROUND STATION FRAME [0:pi]. 0 = pointing along Z
% axis
phi_gs = pi/4;
% Azimuthal angle: of the GROUND STATION FRAME [0:2*pi]. 0 = pointing along
% X axis
theta_gs = pi/4;

% X axis of the GROUND STATION FRAME
x_start_gs = x_gs;
y_start_gs = y_gs;
z_start_gs = z_gs;
r_gs = 15;
x_end_gs = x_gs + r_gs*sin(phi_gs)*cos(theta_gs);
y_end_gs = y_gs + r_gs*sin(phi_gs)*sin(theta_gs);
z_end_gs = z_gs + r_gs*cos(phi_gs);

%% Drone definition

% Polar angle: of the DRONE FRAME [0:pi]. 0 = pointing along Z
% axis
phi_d = 3*pi/4;
% Azimuthal angle: of the Drone FRAME [0:2*pi]. 0 = pointing along
% X axis
theta_d = 5*pi/4;

% X axis of the DRONE FRAME
x_start_d = x_drone;
y_start_d = y_drone;
z_start_d = z_drone;
r_d = 15;
x_end_d = x_drone + r_d*sin(phi_d)*cos(theta_d);
y_end_d = y_drone + r_d*sin(phi_d)*sin(theta_d);
z_end_d = z_drone + r_d*cos(phi_d);

%% Calculation 
% 
% [phi_d,phi_gs] = angle_frames(x_drone,y_drone,angle_d,x_gs,y_gs,angle_gs);
% 
% [GSgain,angle3db_gs] = GSantenna(phi_gs,1);
% [Dgain,angle3db_d] = GSantenna(phi_d,0);
% 
% los_d = sqrt((dxVector(end)-dxVector(1)).^2 + (dyVector(1)-dyVector(end)).^2);
% Lfs = -20*log10(4*pi*los_d*10^3/lambda);
% Prx = Ptx + GSgain + Dgain + Lfs;

%% Representation

figure();
hold on
plot3(x_drone,y_drone,z_drone,'o','LineWidth',2);
plot3(x_gs,y_gs,z_gs,'X','LineWidth',2);
drawArrow([x_start_gs,x_end_gs],[y_start_gs,y_end_gs],[z_start_gs,z_end_gs]);
drawArrow([x_start_d,x_end_d],[y_start_d,y_end_d],[z_start_d,z_end_d]);
plot3(dxVector,dyVector,dzVector,'LineWidth',1.5);
% axis([0 100 0 50]);
grid on;
grid minor;
% str = sprintf('Scenario Simulation 2D \n GS Antenna Gain: %.3f dB \n Drone Antenna Gain: %.3f dB',GSgain,Dgain);
% title(str);
xlabel('X world axis');
ylabel('Y world axis');
zlabel('Z world axis');
legend('Drone','Ground Station','GS Frame','Drone Frame','LOS distance','Location','Best');

