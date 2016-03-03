%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                       Main File for the Project
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

figure(1)
plot(x_drone,y_drone,'o','LineWidth',2);
title('Simulation of the comunication between the drone and the Ground Station');
hold on
plot(x_gs,y_gs,'X','LineWidth',2);
legend('Drone','Ground Station');

%% Ground Station definition

%angle_gs = 2*pi/3;
angle_gs = atan(abs(x_gs-x_drone)/abs(y_drone-y_gs)) + pi/2;
x_start = x_gs;
y_start = y_gs;

x_end = x_gs + x_gs/5*cos(angle_gs);
y_end = y_gs + x_gs/5*sin(angle_gs);

hold on
line([x_start,x_end],[y_start,y_end]);


%% Drone definition

%angle_d = 3*pi/2 + pi/9;
angle_d = atan(abs(x_gs-x_drone)/abs(y_drone-y_gs)) + 3/2*pi;

x_start = x_drone;
y_start = y_drone;

x_end = x_drone + y_drone/5*cos(angle_d);
y_end = y_drone + y_drone/5*sin(angle_d);

hold on
line([x_start,x_end],[y_start,y_end]);

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

hold on
plot(d,y);


%% Calculation 

beta = atan(abs(y_drone-y_gs)/abs(x_gs-x_drone));

phi_gs = pi - angle_gs - beta;

phi_gs = phi_gs*180/pi;

phi_d = 2*pi-(beta + angle_d);

phi_d = phi_d*180/pi;


[GSgain,angle3db] = GSantenna(phi_gs,1);

[Dgain,angle3db] = GSantenna(phi_d,0);





