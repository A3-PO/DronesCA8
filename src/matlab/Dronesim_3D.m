%**************************************************************************
%
% Dronesim_3D.m - CA8 - DRONES
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
% Code to test and check out a DYNAMIC SCENARIO of our drone and ground
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

step_x = 1;                 % Step of vector X [km]
step_y = 1;                 % Step of vector Y [km]
step_z = 1;              % Step of vector Z [km]
prec_d = 200;               % Precision of the distance vector

freq = 2.4 * 10^9;          % Frequency [Hz]
lambda = 3*10^8/freq;       % Wavelength [m]
Ptx = 10*log10(1/(10^-3));  % 1mW power transmiter

% Small function to draw arrows
drawArrow = @(x,y,z) quiver3(x(1),y(1),z(1),x(2)-x(1),y(2)-y(1),z(2)-z(1),'LineWidth',2.5,'MaxHeadSize',1.5);  

x_gs = 0;                  % The position X of the GROUND STATION
y_gs = 0;                  % The position Y of the GROUND STATION
z_gs = 0;                   % The position Z of the GROUND STATION
% DYNAMIC SCENARIO: Drone makes a circle
circle_r = 50;
circle_angle = [-pi:0.1:pi];
x_drone = circle_r*sin(circle_angle);       % The position X of the DRONE
y_drone = circle_r*cos(circle_angle);       % The position Y of the DRONE
z_drone = 50*ones(1,length(x_drone));       % The position Z of the DRONE

% LOS distance vector construction
for i = 1:length(x_drone)
    [dxVector(i,:),dyVector(i,:),dzVector(i,:),los_d(i)] = LOS_distance_3D(x_drone(i),...
        y_drone(i),z_drone(i),x_gs,y_gs,z_gs,prec_d);
end
        
xVector = [min([x_gs x_drone]):step_x:max([x_gs x_drone])];    % World vector X [km]
yVector = [min([y_gs y_drone]):step_y:max([y_gs y_drone])];    % World Vector Y [km]
zVector = [min([z_gs z_drone]):step_z:max([z_gs z_drone])];    % World Vector Z [km]

% Optimal Angle
for i = 1:length(x_drone)
    opTheta(i) = atan2(y_drone(i)-y_gs,x_drone(i)-x_gs);
    opPhi(i) = atan2(z_drone(i)-z_gs,sqrt(abs(x_gs-x_drone(i))^2+...
        abs(y_gs-y_drone(i))^2));
end
%% Ground Station definition

% Polar angle: of the GROUND STATION FRAME [-pi/2:pi/2]. 0 = X-Y plane
% axis
phi_gs = pi/4;
% phi_gs = opPhi;
% Azimuthal angle: of the GROUND STATION FRAME [-pi:pi]. 0 = pointing along
% X axis
theta_gs = pi/2;
% theta_gs = opTheta;

% X axis of the GROUND STATION FRAME
x_start_gs = x_gs;
y_start_gs = y_gs;
z_start_gs = z_gs;
x_end_gs = x_gs + los_d/10*cos(phi_gs)*cos(theta_gs);
y_end_gs = y_gs + los_d/10*cos(phi_gs)*sin(theta_gs);
z_end_gs = z_gs + los_d/10*sin(phi_gs);


%% Drone definition

% Polar angle: of the DRONE [-pi/2:pi/2]. 0 = X-Y plane
phi_d = -pi/3;
% phi_d = -opPhi;
% Azimuthal angle: of the DRONE[-pi:pi]. 0 = pointing along X axis
theta_d = -pi/2;
% theta_d = -pi + opTheta;

% X axis of the DRONE FRAME
for i = 1:length(x_drone)
    x_start_d(i) = x_drone(i);
    y_start_d(i) = y_drone(i);
    z_start_d(i) = z_drone(i);
    x_end_d(i) = x_drone(i) + los_d(i)/10*cos(phi_d)*cos(theta_d);
    y_end_d(i) = y_drone(i) + los_d(i)/10*cos(phi_d)*sin(theta_d);
    z_end_d(i) = z_drone(i) + los_d(i)/10*sin(phi_d);
    
    % Calculation
    
    [alpha_d(i),alpha_gs(i),gamma_d(i),gamma_gs(i)] = LOS_angles(...
        x_drone(i),y_drone(i),z_drone(i),theta_d,phi_d,x_gs,y_gs,z_gs,...
        theta_gs,phi_gs);
    fprintf('Alpha Drone: %.3f | Gamma Drone: %.3f\n',alpha_d(i),gamma_d(i));
    fprintf('Alpha GS: %.3f | Gamma GS: %.3f\n',alpha_gs(i),gamma_gs(i));
    
    [GSgain(i),angle3db_gs] = GSantenna3(alpha_gs(i),gamma_gs(i),0);
    [Dgain(i),angle3db_d] = GSantenna3(alpha_d(i),gamma_d(i),0);
    fprintf('Gain Drone: %.3f \nGain GS: %.3f\n',Dgain(i),GSgain(i));
    
    Lfs(i) = -20*log10(4*pi*los_d(i)*10^3/lambda);
    Prx(i) = Ptx + GSgain(i) + Dgain(i) + Lfs(i);

    %% Representation SCENARIO 3D
    
    clf;
    
    f = figure(1);
    view(45,25);
    hold on
    plot3(x_drone(i),y_drone(i),z_drone(i),'o','LineWidth',2);
    plot3(x_gs,y_gs,z_gs,'X','LineWidth',2);
    drawArrow([x_start_gs,x_end_gs],[y_start_gs,y_end_gs],[z_start_gs,z_end_gs]);
    drawArrow([x_start_d(i),x_end_d(i)],[y_start_d(i),y_end_d(i)],[z_start_d(i),z_end_d(i)]);
    plot3(dxVector(i,:),dyVector(i,:),dzVector(i,:),'LineWidth',1.5);
    plot3(x_drone(1:i),y_drone(1:i),z_drone(1:i));
    axis([-60 60 -60 60 0 50]);
    grid on;
    grid minor;
%     str = sprintf('Scenario Simulation 3D - Full View');
%     str = sprintf(strcat(str,'\nGS Antenna Gain: %.3f dB | Drone Antenna Gain: %.3f dB'),GSgain,Dgain);
%     str = sprintf(strcat(str,'\nAlpha GS: %.3f | Alpha D: %.3f'),alpha_gs,alpha_d);
%     str = sprintf(strcat(str,'\nGamma GS: %.3f | Gamma D: %.3f'),gamma_gs,gamma_d);
%     title(str);
    xlabel('X world axis');
    ylabel('Y world axis');
    zlabel('Z world axis');
    legend('Drone','Ground Station','GS Frame','Drone Frame','LOS distance','Location','Best');
    pause(0.1);
%     movegui(f,'north');
end

% Plotting Power in the receiver
figure(2);
plot(Prx);
grid on;
grid minor;
str = sprintf('Prx');
title(str);
xlabel('Time sample');
ylabel('Relative Amplitude');
% axis([0 length(Prx) -200 0]);

% Plotting error angles for DRONE
figure(3);
plot(alpha_d);
hold on;
plot(gamma_d);
grid on;
grid minor;
str = sprintf('Error angles Drone');
title(str);
xlabel('Time sample');
ylabel('Relative Amplitude');
legend('Alpha D','Gamma D');

% Plotting error angles for GS
figure(4);
plot(alpha_d);
hold on;
plot(gamma_d);
grid on;
grid minor;
str = sprintf('Error angles Ground Station');
title(str);
xlabel('Time sample');
ylabel('Relative Amplitude');
legend('Alpha GS','Gamma GS');
