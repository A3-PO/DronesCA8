%**************************************************************************
%
% Dronesim_2D.m - CA8 - DRONES
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
% Code to test the DYNAMIC SCENARIO of our drone and ground
% station in 2D.
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

step_x = 1;                 % Step of distsance vector X [km]
step_y = 1;                 % Step of height vector Y [km]
prec_d = 200;               % Precision of the distance vector

freq = 2.4 * 10^9;          % Frequency [Hz]
lambda = 3*10^8/freq;       % Wavelength [m]
Ptx = 10*log10(1/(10^-3));  % 1mW power transmiter

% Small function to draw arrows
drawArrow = @(x,y) quiver(x(1),y(1),x(2)-x(1),y(2)-y(1)...
    ,'LineWidth',2.5,'MaxHeadSize',0.5);  

%% Location of GS and DRONE 

% Ground station position
x_gs = 50;                  % The position X of the GROUND STATION 
y_gs = 0;                   % The position Y of the GROUND STATION

% Drone position: DYNAMIC (Line)
x_drone = [0:2:100];                    % The position X of the DRONE
y_drone = 50*ones(1,length(x_drone));   % The position Y of the DRONE

%% NOT USED AT THE MOMENT
% World vectors X, Y[km]
xVector = [min([x_gs x_drone]):step_x:max([x_gs x_drone])];    
yVector = [min([y_gs y_drone]):step_y:max([y_gs y_drone])];    

%% Actual Calculations
% LOS distance vector construction
for i = 1:length(x_drone)
    [dxVector(i,:),dyVector(i,:),los_d(i)] = LOS_distance(x_drone(i),...
        y_drone(i),x_gs,y_gs,prec_d);
end
%Optimal angle
for i = 1:length(x_drone)
    opTheta(i) = atan2(y_drone(i)-y_gs,x_drone(i)-x_gs);
    
    %% Ground Station definition
    % Azimuthal angle: of the GROUND STATION FRAME [-pi:pi]. 0 = pointing along
    % X axis
    theta_gs = opTheta(i);
    
    % POINTING ANTENNA GS FRAME
    x_start_gs = x_gs;
    y_start_gs = y_gs;
    x_end_gs = x_gs + x_gs/5*cos(theta_gs);
    y_end_gs = y_gs + x_gs/5*sin(theta_gs);
    
    %% Drone definition
    % Angle of rotation the GROUND STATION FRAME with to the X world
    theta_d = -pi + opTheta(i);
    
    % POINTING ANTENNA DRONE FRAME
    x_start_d(i) = x_drone(i);
    y_start_d(i) = y_drone(i);
    x_end_d(i) = x_drone(i) + y_drone(i)/5*cos(theta_d);
    y_end_d(i) = y_drone(i) + y_drone(i)/5*sin(theta_d);
    
    %% Calculation
    
    [alpha_d(i),alpha_gs(i)] = LOS_angles(x_drone(i),y_drone(i),...
        theta_d,x_gs,y_gs,theta_gs);
    fprintf('Alpha Drone: %.3f \nAlpha GS: %.3f\n',alpha_d(i),alpha_gs(i));
    
    [GSgain(i),angle3db_gs] = GSantenna(alpha_gs(i),0);
    [Dgain(i),angle3db_d] = GSantenna(alpha_d(i),0);
    fprintf('Gain Drone: %.3f \nGain GS: %.3f\n',Dgain(i),GSgain(i));
    
    Lfs(i) = -20*log10(4*pi*los_d(i)*10^3/lambda);
    Prx(i) = Ptx + GSgain(i) + Dgain(i) + Lfs(i);
    
    %% Representation
    
    f1 = figure(1);
    clf(f1);
    hold on
    plot(x_drone(i),y_drone(i),'o','LineWidth',2);
    plot(x_gs,y_gs,'X','LineWidth',2);
    drawArrow([x_start_gs,x_end_gs],[y_start_gs,y_end_gs]);
    drawArrow([x_start_d(i),x_end_d(i)],[y_start_d(i),y_end_d(i)]);
    plot(dxVector(i,:),dyVector(i,:),'LineWidth',1.5);
    axis([0 100 0 50]);
    grid on;
    grid minor;
    str = sprintf('Scenario Simulation 2D');
    %     str = sprintf(strcat(str,'\n GS Antenna Gain: %.3f dB \n Drone Antenna Gain: %.3f dB',GSgain,Dgain);
    title(str);
    xlabel('X world axis');
    ylabel('Y world axis');
    legend('Drone','Ground Station','GS Frame','Drone Frame','LOS distance','Location','Best');
    pause(0.1);
    movegui(f1,'north');
    
    % Plotting Power in the receiver
    f2 = figure(2);
    clf(f2);
    hold on;
    plot(Prx);
    grid on;
    grid minor;
    str = sprintf('Prx');
    title(str);
    xlabel('Time sample');
    ylabel('Relative Amplitude');
    axis([0 length(x_drone) -200 0]);
    pause(0.1);
    movegui(f2,'south');
    
    % Plotting error angles for DRONE
    f3 = figure(3);
    clf(f3);
    plot(alpha_d);
    grid on;
    grid minor;
    str = sprintf('Error angle Drone');
    title(str);
    xlabel('Time sample');
    ylabel('Relative Amplitude');
    axis([0 length(x_drone) 0 180]);
    pause(0.1);
    movegui(f3,'east');
    
    % Plotting error angles for GS
    f4 = figure(4);
    clf(f4);
    plot(alpha_gs);
    grid on;
    grid minor;
    str = sprintf('Error angle Ground Station');
    title(str);
    xlabel('Time sample');
    ylabel('Relative Amplitude');
    axis([0 length(x_drone) 0 180]);
    pause(0.1);
    movegui(f4,'west');
end

%% STATIONARY PLOTS (JUST IN CASE)
% figure(2);
% plot(x_drone,Prx);
% grid on;
% grid minor;
% str = sprintf('Prx');
% title(str);
% xlabel('X world axis');
% ylabel('Relative Amplitude');
% axis([0 100 -200 0]);
% 
% figure(3);
% plot(x_drone,alpha_d);
% hold on;
% plot(x_drone,alpha_gs);
% grid on;
% grid minor;
% str = sprintf('Prx');
% title(str);
% xlabel('Degrees');
% ylabel('Relative Amplitude');
% legend('Alpha D','Alpha GS');

