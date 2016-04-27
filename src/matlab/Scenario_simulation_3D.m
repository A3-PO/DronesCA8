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
% - LOS_angles.m
% - GSantenna3.m
%
%**************************************************************************

clear all;
close all;
clc;

%% Enviroment parameters

prec_d = 200;               % Precision of the distance vector
freq = 2.4 * 10^9;          % Frequency [Hz]
lambda = 3*10^8/freq;       % Wavelength [m]
Ptx = 10*log10(1/(10^-3));  % 1mW power transmiter

% Small function to draw arrows
drawArrow = @(x,y,z) quiver3(x(1),y(1),z(1),x(2)-x(1),y(2)-y(1),...
    z(2)-z(1),'LineWidth',2.5,'MaxHeadSize', 1/norm([x(2)-x(1);y(2)-y(1)...
    ;z(2)-z(1)]));  

x_gs = 0;                   % The position X of the GROUND STATION
y_gs = 0;                   % The position Y of the GROUND STATION
z_gs = 0;                   % The position Z of the GROUND STATION
x_drone = 50;               % The position X of the DRONE
y_drone = 0;                % The position Y of the DRONE
z_drone = 0;              % The position Z of the DRONE

% LOS distance vector construction
[dxVector,dyVector,dzVector,los_d] = LOS_distance_3D(x_drone,y_drone,z_drone,x_gs,y_gs,z_gs,prec_d);
        
offsetX = 1;
offsetY = 1;
offsetZ = 0.1;
xUpLim = max([x_gs x_drone])+offsetX;
xDownLim = min([z_gs x_drone]) - offsetX;
xVector = [xDownLim:(xUpLim-xDownLim)/prec_d:xUpLim];    % World Vector Z [km]
yUpLim = max([y_gs y_drone])+offsetY;
yDownLim = min([y_gs y_drone]) - offsetY;
yVector = [yDownLim:(yUpLim-yDownLim)/prec_d:yUpLim];    % World Vector Z [km]
zUpLim = max([z_gs z_drone])+offsetZ;
zDownLim = min([z_gs z_drone])- offsetZ;
zVector = [zDownLim:(zUpLim-zDownLim)/prec_d:zUpLim];    % World Vector Z [km]

% Optimal Angle
opTheta = atan2(y_drone-y_gs,x_drone-x_gs);
opPhi = atan2(z_drone-z_gs,sqrt(abs(x_gs-x_drone)^2+...
        abs(y_gs-y_drone)^2));

%% Ground Station definition

% Polar angle: of the GROUND STATION FRAME [-pi/2:pi/2]. 0 = X-Y plane
% axis
% phi_gs = pi/4;
phi_gs = opPhi;
% Azimuthal angle: of the GROUND STATION FRAME [-pi:pi]. 0 = pointing along
% X axis
% theta_gs = pi/2;
theta_gs = opTheta;

% X axis of the GROUND STATION FRAME
x_start_gs = x_gs;
y_start_gs = y_gs;
z_start_gs = z_gs;
x_end_gs = x_gs + los_d/10*cos(phi_gs)*cos(theta_gs);
y_end_gs = y_gs + los_d/10*cos(phi_gs)*sin(theta_gs);
z_end_gs = z_gs + los_d/10*sin(phi_gs);


%% Drone definition

% Polar angle: of the DRONE [-pi/2:pi/2]. 0 = X-Y plane
% phi_d = -pi/3;
phi_d = -opPhi;
% Azimuthal angle: of the DRONE[-pi:pi]. 0 = pointing along X axis
% theta_d = -pi/2;
theta_d = -pi + opTheta;

% X axis of the DRONE FRAME
x_start_d = x_drone;
y_start_d = y_drone;
z_start_d = z_drone;
x_end_d = x_drone + los_d/10*cos(phi_d)*cos(theta_d);
y_end_d = y_drone + los_d/10*cos(phi_d)*sin(theta_d);
z_end_d = z_drone + los_d/10*sin(phi_d);


% Calculation 

[alpha_d,alpha_gs,gamma_d,gamma_gs] = LOS_angles_3D(x_drone,y_drone,...
    z_drone,theta_d,phi_d,x_gs,y_gs,z_gs,theta_gs,phi_gs);

[GSgain,angle3db_gs] = GSantenna3(alpha_gs,gamma_gs,1);
[Dgain,angle3db_d] = GSantenna3(alpha_d,gamma_d,0);

Lfs = -20*log10(4*pi*los_d*10^3/lambda);
Prx = Ptx + GSgain + Dgain + Lfs;

%% Representation SCENARIO 3D

f = figure();
view(45,25);
hold on
plot3(x_drone,y_drone,z_drone,'o','LineWidth',2);
plot3(x_gs,y_gs,z_gs,'X','LineWidth',2);
drawArrow([x_start_gs,x_end_gs],[y_start_gs,y_end_gs],[z_start_gs,z_end_gs]);
drawArrow([x_start_d,x_end_d],[y_start_d,y_end_d],[z_start_d,z_end_d]);
plot3(dxVector,dyVector,dzVector,'LineWidth',1.5);
axis([xVector(1) xVector(end) yVector(1) yVector(end) ...
        zVector(1) zVector(end)]);
grid on;
grid minor;
str = sprintf('Scenario Simulation 3D - Full View');
str = sprintf(strcat(str,'\nGS Antenna Gain: %.3f dB | Drone Antenna Gain: %.3f dB'),GSgain,Dgain);
str = sprintf(strcat(str,'\nAlpha GS: %.3f | Alpha D: %.3f'),alpha_gs,alpha_d);
str = sprintf(strcat(str,'\nGamma GS: %.3f | Gamma D: %.3f'),gamma_gs,gamma_d);
title(str);
xlabel('X world axis');
ylabel('Y world axis');
zlabel('Z world axis');
legend('Drone','Ground Station','GS Frame','Drone Frame','LOS distance','Location','Best');
movegui(f,'north');

% Ploting the SCENARIO in the XZ PLANE
f = figure();
view(0,0);
hold on
plot3(x_drone,y_drone,z_drone,'o','LineWidth',2);
plot3(x_gs,y_gs,z_gs,'X','LineWidth',2);
drawArrow([x_start_gs,x_end_gs],[y_start_gs,y_end_gs],[z_start_gs,z_end_gs]);
drawArrow([x_start_d,x_end_d],[y_start_d,y_end_d],[z_start_d,z_end_d]);
plot3(dxVector,dyVector,dzVector,'LineWidth',1.5);
axis([xVector(1) xVector(end) yVector(1) yVector(end) ...
        zVector(1) zVector(end)]);
grid on;
grid minor;
str = sprintf('Scenario Simulation 3D - XZ plane');
str = sprintf(strcat(str,'\nGS Antenna Gain: %.3f dB | Drone Antenna Gain: %.3f dB'),GSgain,Dgain);
str = sprintf(strcat(str,'\nAlpha GS: %.3f | Alpha D: %.3f'),alpha_gs,alpha_d);
str = sprintf(strcat(str,'\nGamma GS: %.3f | Gamma D: %.3f'),gamma_gs,gamma_d);
title(str);
xlabel('X world axis');
ylabel('Y world axis');
zlabel('Z world axis');
legend('Drone','Ground Station','GS Frame','Drone Frame','LOS distance','Location','Best');
movegui(f,'west');

% Ploting the SCENARIO in the YZ PLANE
f = figure();
view(90,0);
hold on
plot3(x_drone,y_drone,z_drone,'o','LineWidth',2);
plot3(x_gs,y_gs,z_gs,'X','LineWidth',2);
drawArrow([x_start_gs,x_end_gs],[y_start_gs,y_end_gs],[z_start_gs,z_end_gs]);
drawArrow([x_start_d,x_end_d],[y_start_d,y_end_d],[z_start_d,z_end_d]);
plot3([x_gs; x_drone],[y_gs;y_drone],[z_gs; z_drone],'LineWidth',1.5);
axis([xVector(1) xVector(end) yVector(1) yVector(end) ...
        zVector(1) zVector(end)]);
grid on;
grid minor;
str = sprintf('Scenario Simulation 3D - YZ plane');
str = sprintf(strcat(str,'\nGS Antenna Gain: %.3f dB | Drone Antenna Gain: %.3f dB'),GSgain,Dgain);
str = sprintf(strcat(str,'\nAlpha GS: %.3f | Alpha D: %.3f'),alpha_gs,alpha_d);
str = sprintf(strcat(str,'\nGamma GS: %.3f | Gamma D: %.3f'),gamma_gs,gamma_d);
title(str);
xlabel('X world axis');
ylabel('Y world axis');
zlabel('Z world axis');
legend('Drone','Ground Station','GS Frame','Drone Frame','LOS distance','Location','Best');
movegui(f,'east');

% Ploting the SCENARIO in the XY PLANE
f = figure();
view(0,90);
hold on
plot3(x_drone,y_drone,z_drone,'o','LineWidth',2);
plot3(x_gs,y_gs,z_gs,'X','LineWidth',2);
drawArrow([x_start_gs,x_end_gs],[y_start_gs,y_end_gs],[z_start_gs,z_end_gs]);
drawArrow([x_start_d,x_end_d],[y_start_d,y_end_d],[z_start_d,z_end_d]);
plot3(dxVector,dyVector,dzVector,'LineWidth',1.5);
axis([xVector(1) xVector(end) yVector(1) yVector(end) ...
        zVector(1) zVector(end)]);
grid on;
grid minor;
str = sprintf('Scenario Simulation 3D - XY plane');
str = sprintf(strcat(str,'\nGS Antenna Gain: %.3f dB | Drone Antenna Gain: %.3f dB'),GSgain,Dgain);
str = sprintf(strcat(str,'\nAlpha GS: %.3f | Alpha D: %.3f'),alpha_gs,alpha_d);
str = sprintf(strcat(str,'\nGamma GS: %.3f | Gamma D: %.3f'),gamma_gs,gamma_d);
title(str);
xlabel('X world axis');
ylabel('Y world axis');
zlabel('Z world axis');
legend('Drone','Ground Station','GS Frame','Drone Frame','LOS distance','Location','Best');
movegui(f,'south');
