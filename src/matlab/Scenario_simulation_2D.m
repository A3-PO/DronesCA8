%**************************************************************************
%
% Scenario_simulation_2D.m - CA8 - DRONES
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
% station in 2D
%
% Functions used during the code:
% - LOS_distance.m
% - angle_frames.m
% - GSantenna.m
%
%**************************************************************************

clear all;
close all;
clc;

%% Enviroment parameters
% Definition of some variables which will be important to draw the
% environment. It is going to be considered that one of the elements is
% going to be iin one axis.

step_x = 1;                 % Step of distsance vector X [km]
step_y = 1;                 % Step of height vector Y [km]
prec_d = 200;               % Precision of the distance vector

freq = 2.4 * 10^9;          % Frequency [Hz]
lambda = 3*10^8/freq;       % Wavelength [m]
Ptx = 10*log10(1/(10^-3));  % 1mW power transmiter

% Small function to draw arrows
drawArrow = @(x,y) quiver(x(1),y(1),x(2)-x(1),y(2)-y(1),'LineWidth',2.5,'MaxHeadSize',0.5);  

y_gs = 0;                   % The position Y of the GROUND STATION
x_gs = 50;                  % The position X of the GROUND STATION              
y_drone = 50;               % The position Y of the DRONE
x_drone = 0;                % The position X of the DRONE

% LOS distance vector construction
[dxVector,dyVector] = LOS_distance(x_drone,y_drone,x_gs,y_gs,prec_d);
        
xVector = [min([x_gs x_drone]):step_x:max([x_gs x_drone])];    % World vector X [km]
yVector = [min([y_gs y_drone]):step_y:max([y_gs y_drone])];    % World Vector Y [km]


%% Ground Station definition

% Angle of rotation the GROUND STATION FRAME with respecto to the X world
angle_gs = pi/2;
% angle_gs = 2*pi/3;
% angle_gs = atan(abs(x_gs-x_drone)/abs(y_drone-y_gs)) + pi/2;

% X axis of the GROUND STATION FRAME
x_start_gs = x_gs;
y_start_gs = y_gs;
x_end_gs = x_gs + x_gs/5*cos(angle_gs);
y_end_gs = y_gs + x_gs/5*sin(angle_gs);

%% Drone definition

% Angle of rotation the GROUND STATION FRAME with respecto to the X world
angle_d = 3*pi/2;
% angle_d = atan(abs(x_gs-x_drone)/abs(y_drone-y_gs)) + 3/2*pi;

% X axis of the DRONE FRAME
x_start_d = x_drone;
y_start_d = y_drone;
x_end_d = x_drone + y_drone/5*cos(angle_d);
y_end_d = y_drone + y_drone/5*sin(angle_d);

%% Calculation 

[phi_d,phi_gs] = angle_frames(x_drone,y_drone,angle_d,x_gs,y_gs,angle_gs);

[GSgain,angle3db_gs] = GSantenna(phi_gs,1);
[Dgain,angle3db_d] = GSantenna(phi_d,0);

los_d = sqrt((dxVector(end)-dxVector(1)).^2 + (dyVector(1)-dyVector(end)).^2);
Lfs = -20*log10(4*pi*los_d*10^3/lambda);
Prx = Ptx + GSgain + Dgain + Lfs;

%% Representation

figure();
hold on
plot(x_drone,y_drone,'o','LineWidth',2);
plot(x_gs,y_gs,'X','LineWidth',2);
drawArrow([x_start_gs,x_end_gs],[y_start_gs,y_end_gs]);
drawArrow([x_start_d,x_end_d],[y_start_d,y_end_d]);
plot(dxVector,dyVector,'LineWidth',1.5);
axis([0 100 0 50]);
grid on;
grid minor;
str = sprintf('Scenario Simulation 2D \n GS Antenna Gain: %.3f dB \n Drone Antenna Gain: %.3f dB',GSgain,Dgain);
title(str);
xlabel('X world axis');
ylabel('Y world axis');
legend('Drone','Ground Station','GS Frame','Drone Frame','LOS distance','Location','Best');


