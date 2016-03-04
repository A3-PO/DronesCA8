%**************************************************************************
%
% CA8 - DRONES
%
%**************************************************************************
%
% Group 832 - Control and Automation Msc.
% Aalborg University
% February 2016
%
%**************************************************************************
%
% Description:
% In order to understant better how it works the supervisor told us that
% cold be important to construct the graph of the antenna intensity 
% radiation.

clear all;
close all;
clc;

%% Development of the environment where we want to put our antennas

% Definition of some variables which will be important to draw the
% environment. It is going to be considered that one of the elements is
% going to be iin one axis.

% These values should be in kilometres
step_d = 1;
y_gs = 0; % The position y of the ground station
x_gs = 50; % The position x of the ground station
distance = 0:step_d:x_gs;

% These values should be in meters
step_h = 1;
y_drone = 50; % The position y of the ground station
x_drone = -10; % The position x of the ground station
height = 0:step_h:y_drone;

% Small function to draw arrows
drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0,'LineWidth',2.5,'MaxHeadSize',0.5);  

%% Ground Station definition

angle_gs = 2*pi/3;
% angle_gs = atan(abs(x_gs-x_drone)/abs(y_drone-y_gs)) + pi/2;
x_start_gs = x_gs;
y_start_gs = y_gs;

x_end_gs = x_gs + x_gs/5*cos(angle_gs);
y_end_gs = y_gs + x_gs/5*sin(angle_gs);

%% Drone definition

angle_d = 3*pi/2 + pi/9;
% angle_d = atan(abs(x_gs-x_drone)/abs(y_drone-y_gs)) + 3/2*pi;

x_start_d = x_drone;
y_start_d = y_drone;

x_end_d = x_drone + y_drone/5*cos(angle_d);
y_end_d = y_drone + y_drone/5*sin(angle_d);



%% Connection between two origins

% Definition of the variables which will be necessary to calculate the
% parameters of the straight line
x1 = x_drone;
y1 = y_drone;
x2 = x_gs;
y2 = y_gs;
step = 1;

syms m b

eq0 = y1 == m*x1 + b;
eq1 = y2 == m*x2 + b;

[m, b] = solve([eq0, eq1], [m, b]);

if x1 > x2
    d = x2:step:x1;
else
    d = x1:step:x2;
end

y = m*d + b;

%% Calculation 

beta = atan(abs(y_drone-y_gs)/abs(x_gs-x_drone));
phi_gs = pi - angle_gs - beta;
phi_gs = phi_gs*180/pi;
phi_d = 2*pi-(beta + angle_d);
phi_d = phi_d*180/pi;

[GSgain,angle3db_gs] = GSantenna(phi_gs,0);
[Dgain,angle3db_d] = GSantenna(phi_d,0);

% Representation

figure();
hold on
plot(x_drone,y_drone,'o','LineWidth',2);
plot(x_gs,y_gs,'X','LineWidth',2);
drawArrow([x_start_gs,x_end_gs],[y_start_gs,y_end_gs]);
drawArrow([x_start_d,x_end_d],[y_start_d,y_end_d]);
plot(d,y,'LineWidth',1.5);
grid on;
grid minor;
str = sprintf('Scenario Simulation 2D \n GS Antenna Gain: %.3f dB \n Drone Antenna Gain: %.3f dB',GSgain,Dgain);
title(str);
xlabel('X world axis');
ylabel('Y world axis');
legend('Drone','Ground Station','GS Frame','Drone Frame','LOS distance');


