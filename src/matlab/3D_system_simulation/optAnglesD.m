function [alpha_d,gamma_d,opt_theta_d,opt_phi_d] = ...
    optAnglesD(lat_drone,long_drone,alt_drone,lat_gs,long_gs,alt_gs,theta_d,phi_d)
%**************************************************************************
%
% optAnglesD.m - CA8 - DRONES
%
%**************************************************************************
%
% Group 832 - Control and Automation Msc.
% Aalborg University
% May 2016
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
%% Transform Drone position to Local NED GS

geoGS = [lat_gs;long_gs;alt_gs];
geoD = [lat_drone;long_drone;alt_drone];

ecefGS = geo2ecef(geoGS);
ecefD = geo2ecef(geoD);

nedGS = ecef2ned(ecefGS,geoD)

x_gs = nedGS(1);  % X = NORTH
y_gs = nedGS(2);  % Y = EAST
z_gs = nedGS(3);  % Z = DOWN

% Optimal angle THETA
opt_theta_d = atan2(x_gs,y_gs);

% Phi error calculations: GAMMA
opt_phi_d = atan2(-z_gs,sqrt(abs(x_gs)^2 + abs(y_gs)^2));

% Error angles
alpha_d = opt_theta_d - theta_d;
gamma_d = opt_phi_d - phi_d;

end

