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

%% Initial values
% Drone position
lat_init_d = 56.08736247;
lat_end_d = 56.15166933;
long_init_d = 8.26781273;
long_end_d = 10.20492554;

lat_drone = lat_init_d:(lat_end_d-lat_init_d)/prec_geo:lat_end_d;       
long_drone = long_init_d:(long_end_d - long_init_d)/prec_geo:long_end_d;   
alt_drone = 100*ones(1,length(lat_drone));   

% Drone deviation
yaw = 0;
pitch = 0;
roll = 0;
for i = 2:length(lat_drone)
    yaw = [yaw yaw(end)+5*round(rand(1))];
    pitch = [pitch pitch(end)+5*round(rand(1))];
    roll = [roll roll(end)+5*round(rand(1))];
end

% Ground station position
lat_gs = 56.08736247 * ones(1,length(lat_drone));       % The position X of the GROUND STATION 
long_gs = 8.26446533 * ones(1,length(lat_drone));        % The position Y of the GROUND STATION
alt_gs = 19 * ones(1,length(lat_drone));                % The position Z of the GROUND STATION

% Small function to draw arrows
drawArrow = @(x,y,z) quiver3(x(1),y(1),z(1),x(2)-x(1),y(2)-y(1),...
    z(2)-z(1),'LineWidth',2.5,'MaxHeadSize',1.5);  

%% RUNNING Simulation of the model with the controller (dronesmodel.mdl)
time = 0:1/fsampling:(length(lat_drone)-1)/fsampling;    % Time

[T,X,Y] = sim('controller_v3',time);

%% Simulation data of angles
t = theta_d_vec.Time;                       % sample time
theta_d_vec = theta_d_vec.Data;             % current drone angle
phi_d_vec = phi_d_vec.Data;                 % current drone angle
theta_gs_vec = theta_gs_vec.Data;           % current ground station angle
phi_gs_vec = phi_gs_vec.Data;               % current ground station angle
opt_theta_d = opt_theta_d.Data;             % optimal drone angle
opt_theta_gs = opt_theta_gs.Data;           % optimal ground station angle
opt_phi_d = opt_phi_d.Data;                 % optimal drone angle
opt_phi_gs = opt_phi_gs.Data;               % optimal ground station angle
alpha_d = alpha_d.Data;
alpha_gs = alpha_gs.Data;
gamma_d = gamma_d.Data;
gamma_gs = gamma_gs.Data;
los_d_vec = los_d_vec.Data;

%% Axis for ploting

xUpLim = ceil(max([lat_gs lat_drone]));
xUpLim = xUpLim + xUpLim/3;
xDownLim = floor(min([alt_gs lat_drone]));
xDownLim = xDownLim -xDownLim/3;

yUpLim = ceil(max([long_gs long_drone])*1000)/1000;
yUpLim = yUpLim + yUpLim/3;
yDownLim = floor(min([long_gs long_drone])*1000)/1000;
yDownLim = yDownLim - yDownLim/3;

zUpLim = ceil(max([alt_gs alt_drone])*1000)/1000;
zUpLim = zUpLim + zUpLim/3;
zDownLim = floor(min([alt_gs alt_drone])*1000)/1000;
zDownLim = zDownLim - zDownLim/3;
% zDownLim = -0.13


%% Plot current vs optimal angle of drone and ground station 
f1 = figure(1);
subplot(2,1,1);
plot(t,theta_d_vec,'LineWidth',2);
hold on;
plot(t,opt_theta_d,'r');
xlabel('Sample');
ylabel('Angle [deg]');
legend('Drone','Optimal');
title('Theta Drone angle vs optimal');
grid on;
grid minor;
subplot(2,1,2);
plot(t,phi_d_vec,'LineWidth',2);
hold on;
plot(t,opt_phi_d,'r');
xlabel('Sample');
ylabel('Angle [deg]');
legend('Drone','Optimal');
title('Phi Drone angle vs optimal');
grid on;
grid minor;
movegui(f1,'northwest');

f2 = figure(2);
subplot(2,1,1);
plot(t,theta_gs_vec,'LineWidth',2);
hold on;
plot(t,opt_theta_gs,'r');
xlabel('Sample');
ylabel('Angle [deg]');
legend('Ground station','Optimal');
title('Theta Ground station angle vs optimal');
grid on;
grid minor;
subplot(2,1,2);
plot(t,phi_gs_vec,'LineWidth',2);
hold on;
plot(t,opt_phi_gs,'r');
xlabel('Sample');
ylabel('Angle [deg]');
legend('Ground station','Optimal');
title('Phi Ground station angle vs optimal');
grid on;
grid minor;
movegui(f2,'southwest');

f3 = figure(3);
plot(t,los_d_vec/1000,'LineWidth',2);
xlabel('Sample');
ylabel('Distance[km]');
title('LOS Distance');
grid on;
grid minor;
movegui(f3,'northeast');

%% 3D
% for i = 1:length(x_drone)
%     
%     % LOS DISTANCe
%     [dxVector(i,:),dyVector(i,:),dzVector(i,:),los_d(i)] = ...
%         LOS_distance_3D(x_drone(i),y_drone(i),z_drone(i),...
%         x_gs(i),y_gs(i),z_gs(i),prec_d);
%     
%     % GAINS
%     alpha_gs_deg = rad2deg(alpha_gs(i));
%     alpha_d_deg = rad2deg(alpha_d(i));
%     gamma_gs_deg = rad2deg(gamma_gs(i));
%     gamma_d_deg = rad2deg(gamma_d(i));
%     [GSgain(i),angle3db_gs] = GSantenna3(alpha_gs_deg,gamma_gs_deg,0);
%     [Dgain(i),angle3db_d] = GSantenna3(alpha_d_deg,gamma_d_deg,0);
%     
%     % POWER IN RECEIVER
%     Lfs(i) = -20*log10(4*pi*los_d(i)*10^3/lambda);
%     Prx(i) = Ptx + GSgain(i) + Dgain(i) + Lfs(i);
%     
%     if plotr == 1
%         % Ground Station definition
%         % GROUND STATION FRAME Polar angle [-pi/2:pi/2]. 0 = X-Y plane
%         phi_gs = phi_gs_vec(i);
%         % GROUND STATION FRAME Azimuthal angle: [-pi:pi]. 0 = along X axis
%         theta_gs = theta_gs_vec(i);
%         
%         % POINTING ANTENNA GS FRAME
%         x_start_gs(i) = x_gs(i);
%         y_start_gs(i) = y_gs(i);
%         z_start_gs(i) = z_gs(i);
%         x_end_gs(i) = x_gs(i) + los_d(i)/5*cos(phi_gs)*cos(theta_gs);
%         y_end_gs(i) = y_gs(i) + los_d(i)/5*cos(phi_gs)*sin(theta_gs);
%         z_end_gs(i) = z_gs(i) + los_d(i)/5*sin(phi_gs);
%         
%         % Drone definition
%         % DRONE Polar angle [-pi/2:pi/2]. 0 = X-Y plane
%         phi_d = phi_d_vec(i);
%         % DRONE Azimuthal angle: [-pi:pi]. 0 = along X axis
%         theta_d = theta_d_vec(i);
%         
%         % POINTING ANNTENA DRONE FRAME
%         x_start_d(i) = x_drone(i);
%         y_start_d(i) = y_drone(i);
%         z_start_d(i) = z_drone(i);
%         x_end_d(i) = x_drone(i) + los_d(i)/5*cos(phi_d)*cos(theta_d);
%         y_end_d(i) = y_drone(i) + los_d(i)/5*cos(phi_d)*sin(theta_d);
%         z_end_d(i) = z_drone(i) + los_d(i)/5*sin(phi_d);
%         
%         % Representation SCENARIO 3D
%         
%         f3 = figure(3);
%         clf(f3);
%         view(45,25);
%         hold on
%         plot3(x_drone(i),y_drone(i),z_drone(i),'o','LineWidth',2);
%         plot3(x_gs(i),y_gs(i),z_gs(i),'X','LineWidth',2);
%         drawArrow([x_start_gs(i),x_end_gs(i)],[y_start_gs(i),y_end_gs(i)],...
%             [z_start_gs(i),z_end_gs(i)]);
%         drawArrow([x_start_d(i),x_end_d(i)],[y_start_d(i),y_end_d(i)],...
%             [z_start_d(i),z_end_d(i)]);
%         plot3(dxVector(i,:),dyVector(i,:),dzVector(i,:),'LineWidth',1.5);
%         plot3(x_drone(1:i),y_drone(1:i),z_drone(1:i));
%         %     axis([0 60 0 1 0 0.1]);
%         axis([xDownLim xUpLim yDownLim yUpLim ...
%             zDownLim zUpLim]);
%         grid on;
%         grid minor;
%         str = sprintf('Scenario Simulation 3D - Full View');
%         title(str);
%         xlabel('X world axis');
%         ylabel('Y world axis');
%         zlabel('Z world axis');
%         legend('Drone','Ground Station','GS Frame','Drone Frame',...
%             'LOS distance','Location','Best');
%         pause(0.1);
%         movegui(f3,'center');
%         
%         f4 = figure(4);
%         clf(f4);
%         view(90,0);
%         hold on
%         plot3(x_drone(i),y_drone(i),z_drone(i),'o','LineWidth',2);
%         plot3(x_gs(i),y_gs(i),z_gs(i),'X','LineWidth',2);
%         drawArrow([x_start_gs(i),x_end_gs(i)],[y_start_gs(i),y_end_gs(i)],...
%             [z_start_gs(i),z_end_gs(i)]);
%         drawArrow([x_start_d(i),x_end_d(i)],[y_start_d(i),y_end_d(i)],...
%             [z_start_d(i),z_end_d(i)]);
%         plot3(dxVector(i,:),dyVector(i,:),dzVector(i,:),'LineWidth',1.5);
%         plot3(x_drone(1:i),y_drone(1:i),z_drone(1:i));
%         %     axis([0 60 0 1 0 0.1]);
%         axis([xDownLim xUpLim yDownLim yUpLim ...
%             zDownLim zUpLim]);
%         grid on;
%         grid minor;
%         str = sprintf('Scenario Simulation 3D - Full View');
%         title(str);
%         xlabel('X world axis');
%         ylabel('Y world axis');
%         zlabel('Z world axis');
%         legend('Drone','Ground Station','GS Frame','Drone Frame',...
%             'LOS distance','Location','Best');
%         pause(0.1);
%         movegui(f4,'northeast');
%         
%         f5 = figure(5);
%         clf(f5);
%         view(0,90);
%         hold on
%         plot3(x_drone(i),y_drone(i),z_drone(i),'o','LineWidth',2);
%         plot3(x_gs(i),y_gs(i),z_gs(i),'X','LineWidth',2);
%         drawArrow([x_start_gs(i),x_end_gs(i)],[y_start_gs(i),y_end_gs(i)],...
%             [z_start_gs(i),z_end_gs(i)]);
%         drawArrow([x_start_d(i),x_end_d(i)],[y_start_d(i),y_end_d(i)],...
%             [z_start_d(i),z_end_d(i)]);
%         plot3(dxVector(i,:),dyVector(i,:),dzVector(i,:),'LineWidth',1.5);
%         plot3(x_drone(1:i),y_drone(1:i),z_drone(1:i));
%         %     axis([0 60 0 1 0 0.1]);
%         axis([xDownLim xUpLim yDownLim yUpLim ...
%             zDownLim zUpLim]);
%         grid on;
%         grid minor;
%         str = sprintf('Scenario Simulation 3D - Full View');
%         title(str);
%         xlabel('X world axis');
%         ylabel('Y world axis');
%         zlabel('Z world axis');
%         legend('Drone','Ground Station','GS Frame','Drone Frame',...
%             'LOS distance','Location','Best');
%         pause(0.1);
%         movegui(f5,'southeast');
%     end
% end
% 
% % Plotting Power in the receiver
% f6 = figure(6);
% clf(f6);
% hold on;
% plot(Prx);
% grid on;
% grid minor;
% str = sprintf('Prx');
% title(str);
% xlabel('Time sample');
% ylabel('Relative Amplitude');
% axis([1 length(x_drone) -140 -40]);
