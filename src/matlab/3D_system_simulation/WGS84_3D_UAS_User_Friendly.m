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

%% VARIABLES
% Simulation variables
plotr = 0;
fsampling = 1;
prec_geo = 100;
prec_d = 200;               % Precision of the distance vector
yaw = [0,0,1];
pitch = [0,1,0];
roll = [1,0,0];
                         
% Plant and Controller constants
Ra  = 25.6;                 % Resistor [Ohm]
La  = 14.8*10^(-3);         % Inductance [H.]
Kt  = 0.171;                % Torque constant [N*m/A]
Kb  = 0.171;                % Back-emf constant [V/(r/s)]
Je  = 3*10^(-6);            % Load attached to motor shaft [kg*m^2]
De  = 7.7*10^(-6);          % Damping behavior [N*m/(r/s)]
upSat = 5;                  % Upper saturation limit
lowSat = -5;                % Lower saturation limit
gear = 0.001;               % Gear ratio
freq = 2.4 * 10^9;          % Frequency [Hz]
lambda = 3*10^8/freq;       % Wavelength [m]
Ptx = 10*log10(1/(10^-3));  % 1mW power transmiter

%% LOADING MAP
% Input parameters
H_gs = 20;
H_ua = 100;
% H_gs = input('Input GS altitude: ');     % [m] GS altitude 
% H_ua = input('Input UA altitude: ');     % [m] UA altitude
latlim = [54 58];                         % Map latitude limits
lonlim = [8 16];                          % Map longitude limits
R_e = earthRadius('meters');              % Earth radius

%Import map as a Web Map Service (WMS)
layers = wmsfind('nasa.network*elev', 'SearchField', 'serverurl');
layers = wmsupdate(layers);
% disp(layers,'Properties',{'LayerTitle','LayerName'})   % map info
aster = layers.refine('earthaster', 'SearchField', 'layername');
cellSize = dms2degrees([0,1,0]);
[ZA, RA] = wmsread(aster, 'Latlim', latlim, 'Lonlim', lonlim, ...
   'CellSize', cellSize, 'ImageFormat', 'image/bil');
Z = double(ZA);
R = RA;

%   LOADING SCRIPT 
fprintf('\n1.Angle range');
fprintf('\n2.Curvature');
fprintf('\n3.Above');
fprintf('\n4.Mountain');
scenario = input('\nEnter scenario: ');
switch scenario
    case 1
        load('Scenario2.mat');
        lat1 = GSPOS(1);
        long1 = GSPOS(2);
        lat2 = UASTART(1);
        long2 = UASTART(2);
        lat3 = UAEND(1);
        long3 = UAEND(2);
    case 2
        load('Scenario1.mat');
        lat1 = GSPOS(1);
        long1 = GSPOS(2);
        lat2 = UASTART(1);
        long2 = UASTART(2);
        lat3 = UAEND(1);
        long3 = UAEND(2);
    case 3
        load('Scenario3.mat');
        lat1 = GSPOS(1);
        long1 = GSPOS(2);
        lat2 = UASTART(1);
        long2 = UASTART(2);
        lat3 = UAEND(1);
        long3 = UAEND(2);
    case 4
        load('Scenario4.mat');
        lat1 = GSPOS(1);
        long1 = GSPOS(2);
        lat2 = UASTART(1);
        long2 = UASTART(2);
        lat3 = UAEND(1);
        long3 = UAEND(2);
end

f1 = figure(1);
worldmap(latlim, lonlim);
geoshow(ZA, RA, 'DisplayType', 'texturemap')
demcmap(double(ZA))
title({'Northen Europe - Topographic Map'});
% [lat1, long1] = inputm(1);                   % Input GS and UA locations
plotm(lat1,long1,'o','Color',[1 0.5 0.2],'LineWidth',3);textm(lat1,long1,'  GS');
% [lat2, long2] = inputm(1);
plotm(lat2,long2,'bo','LineWidth',3);textm(lat2,long2,'  UA Start');
% [lat3, long3] = inputm(1);
plotm(lat3,long3,'bo','LineWidth',3);textm(lat3,long3,'  UA End');
% GSPOS = [lat1, long1];
% UASTART = [lat2,long2];
% UAEND = [lat3,long3];
% save('Scenario4','GSPOS','UASTART','UAEND');

%% Initial values
% Drone position
lat_init_d = lat2;
lat_end_d = lat3;
long_init_d = long2;
long_end_d = long3;

lat_drone = lat_init_d:(lat_end_d-lat_init_d)/prec_geo:lat_end_d;       
long_drone = long_init_d:(long_end_d - long_init_d)/prec_geo:long_end_d;   
for t=1:length(lat_drone)
    alt_drone(t) = H_ua + ltln2val(Z, R, lat_drone(t), long_drone(t));
end

% Drone deviation
yawAngle = 0;
pitchAngle = 0;
rollAngle = 0;
for i = 2:length(lat_drone)
    yawAngle = [yawAngle yawAngle(end)+round(-1 + 2*rand(1))];
    pitchAngle = [pitchAngle pitchAngle(end)+round(-1 + 2*rand(1))];
    rollAngle = [rollAngle rollAngle(end)+round(-1 + 2*rand(1))];
end

% Ground station position
lat_gs = lat1 * ones(1,length(lat_drone));   % The position X of the GS 
long_gs = long1 * ones(1,length(lat_drone)); % The position Y of the GS
alt_gs = H_gs * ones(1,length(lat_drone));   % The position Z of the GS

% Small function to draw arrows
drawArrow = @(x,y,z) quiver3(x(1),y(1),z(1),x(2)-x(1),y(2)-y(1),...
    z(2)-z(1),'LineWidth',2.5,'MaxHeadSize',1.5);  

%% RUNNING Simulation of the model with the controller (dronesmodel.mdl)
time = 0:1/fsampling:(length(lat_drone)-1)/fsampling;    % Time
fprintf('\nControllers: ');
fprintf('\n1.P  ');
fprintf('\n2.PI ');
fprintf('\n3.PD ');
fprintf('\n4.PID');
controller = input('\nChoose controller: ');
switch controller
    case 1  
        [T,X,Y] = sim('controller_P',time);
    case 2
        [T,X,Y] = sim('controller_PI',time);
    case 3
        [T,X,Y] = sim('controller_PD',time);
    case 4
        [T,X,Y] = sim('controller_PID',time);
end

%% Simulation data of angles
t = theta_d_vec.Time;                       % sample time
theta_d_vec = theta_d_vec.Data;             % current UA angle
phi_d_vec = phi_d_vec.Data;                 % current UA angle
theta_gs_vec = theta_gs_vec.Data;           % current GS angle
phi_gs_vec = phi_gs_vec.Data;               % current GS angle
opt_theta_d = opt_theta_d.Data;             % optimal UA angle
opt_theta_gs = opt_theta_gs.Data;           % optimal GS angle
opt_phi_d = opt_phi_d.Data;                 % optimal UA angle
opt_phi_gs = opt_phi_gs.Data;               % optimal GS angle
alpha_d = alpha_d.Data;
alpha_gs = alpha_gs.Data;
gamma_d = gamma_d.Data;
gamma_gs = gamma_gs.Data;
los_d_vec = los_d_vec.Data;

%% Plot current vs optimal angle of UA and GS
% UA vs Optimal Angle
f2 = figure(2);
subplot(2,1,1);
    plot(t,rad2deg(theta_d_vec),'LineWidth',2);
    hold on;
    plot(t,rad2deg(opt_theta_d),'r');
    xlabel('Sample');
    ylabel('Angle [deg]');
    legend('\theta_{UA}','\theta_{OPTIMAL}');
    title('Azimuth Angle: UA vs OPTIMAL');
    grid on;
    grid minor;
subplot(2,1,2);
    plot(t,rad2deg(phi_d_vec),'LineWidth',2);
    hold on;
    plot(t,rad2deg(opt_phi_d),'r');
    xlabel('Sample');
    ylabel('Angle [deg]');
    legend('\phi_{UA}','\phi_{OPTIMAL}');
    title('Elevation Angle: UA vs OPTIMAL');
    grid on;
    grid minor;
movegui(f2,'northwest');
% print('../../../doc/report/figures/Drone_angles_1.eps','-depsc');

% GS vs Optimal Angle
f3 = figure(3);
subplot(2,1,1);
    plot(t,rad2deg(theta_gs_vec),'LineWidth',2);
    hold on;
    plot(t,rad2deg(opt_theta_gs),'r');
    xlabel('Sample');
    ylabel('Angle [deg]');
    legend('\theta_{GS}','\theta_{OPTIMAL}');
    title('Azimuth Angle: GS vs OPTIMAL');
    grid on;
    grid minor;
subplot(2,1,2);
    plot(t,rad2deg(phi_gs_vec),'LineWidth',2);
    hold on;
    plot(t,rad2deg(opt_phi_gs),'r');
    xlabel('Sample');
    ylabel('Angle [deg]');
    legend('\phi_{GS}','\phi_{OPTIMAL}');
    title('Elevation Angle: GS vs OPTIMAL');
    grid on;
    grid minor;
movegui(f3,'southwest');
% print('../../../doc/report/figures/GS_angles_1.eps','-depsc');

% LOS and Distance Plot
f4 = figure(4);
    xlabel('Sample');
    ylabel('Distance [km]');
    title('LOS and Distance between GS and UA ');
    grid on;
    grid minor;
    hold on;
movegui(f4,'northeast');

% Power and Gains 
for i = 1:length(lat_drone)
       
    % GAINS
    alpha_gs_deg = rad2deg(alpha_gs(i));
    alpha_d_deg = rad2deg(alpha_d(i));
    gamma_gs_deg = rad2deg(gamma_gs(i));
    gamma_d_deg = rad2deg(gamma_d(i));
    [GSgain(i),angle3db_gs] = GSantenna3(alpha_gs_deg,gamma_gs_deg,0);
    [Dgain(i),angle3db_d] = GSantenna3(alpha_d_deg,gamma_d_deg,0);
    
    % POWER IN RECEIVER
    Lfs(i) = -20*log10(4*pi*los_d_vec(i)/lambda);
    Prx(i) = Ptx + GSgain(i) + Dgain(i) + Lfs(i);
    
    % Line Of Sight check
    [vis(i)] = los2(Z, R, lat_gs(i), long_gs(i), lat_drone(i), ...
        long_drone(i), H_gs, H_ua, 'AGL', 'AGL', R_e, 4/3*R_e);
    if vis(i) == 1
        tmp = 'bo';
    else
        tmp = 'rx';
        Prx(i) = -200;
    end
    
    plot(t(i),los_d_vec(i)/1000,tmp);
end
% print('../../../doc/report/figures/LOS_1.eps','-depsc');

% Plotting Power in the receiver
f5 = figure(5);
    clf(f5);
    hold on;
    plot(Prx);
    grid on;
    grid minor;
    str = sprintf('Power at the receiver - P_{RX}');
    title(str);
    xlabel('Time sample');
    ylabel('Relative Amplitude [dBm]');
    axis([1 length(lat_drone) -120 0]);
movegui(f5,'south');

%% Save graphs to file
% print(f1,'../../../doc/report/figures/scenario_4_map.png','-dpng');
% print(f2,'../../../doc/report/figures/scenario_4_ua.png','-dpng');
% print(f3,'../../../doc/report/figures/scenario_4_gs.png','-dpng');
% print(f4,'../../../doc/report/figures/scenario_4_los.png','-dpng');
% print(f5,'../../../doc/report/figures/scenario_4_power.png','-dpng');



% Define arrow
% f6 = figure(6);
% arrow = arrow3D([-1,0,0] ,[2,0,0]);
% view(45, 25); 
% axis ([-1.5 1.5 -1.5 1.5 -1.5 1.5]);
% grid on;
% grid minor;
% set(gca,'TickLength',[ 0 0 ]);
% set(gca,'XTickLabel',[],'YTickLabel',[],'ZTickLabel',[]);
% xlabel('East Axis'); ylabel('North Axis'); zlabel('Down Axis');
% title('Drone Body Rotation - Local NED Frame');
% set(gca,'xticklabel',[]);
% movegui(f6,'southeast');
% for i = 2:length(yawAngle)
%     yawDif = yawAngle(i)-yawAngle(i-1);
%     pitchDif = pitchAngle(i)-pitchAngle(i-1);
%     rollDif = rollAngle(i)-rollAngle(i-1);
%     
%     rotate(arrow,yaw,yawDif);
%     rotate(arrow,pitch,pitchDif);
%     rotate(arrow,roll,rollDif);
%     drawnow
% end

