function [alpha_gs,gamma_gs,opt_theta_gs,opt_phi_gs] = ...
    optAnglesGS(lat_drone,long_drone,alt_drone,lat_gs,long_gs,alt_gs,theta_gs,phi_gs)
%**************************************************************************
%
% optAnglesGS.m - CA8 - DRONES
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
% Function that performs the calculation for a 3 DIMENSION - 2 DEGREE OF
% FREEDOM 
%
% INPUTS:
% - x_drone = Position of the drone in X
% - y_drone = Position of the drone in Y
% - z_drone = Position of the drone in Z
% - x_gs = Position of the ground station in X
% - y_gs = Position of the ground station in Y
% - z_gs = Position of the ground station in Z
% - theta_gs = ANGLE of the ground station with respect to the X world
% - phi_gs = ANGLE of the ground station with respect to the Z world
%
% OUTPUTS:
% - alpha_gs = Angle from the direction of the pointing antenna of the ground station 
%   to the LINE OF SIGHT that joins ground station and drone. (SPHERICAL
%   COORDINATES)
% - phi_gs = Angle from the direction of the pointing antenna of the ground station 
%   to the LINE OF SIGHT that joins ground station and drone. (SPHERICAL
%   COORDINATES)
%
%**************************************************************************
%% Transform Drone position to Local NED GS

geoGS = [lat_gs;long_gs;alt_gs];
geoD = [lat_drone;long_drone;alt_drone];

ecefGS = geo2ecef(geoGS);
ecefD = geo2ecef(geoD);

nedD = ecef2ned(ecefD,geoGS)

x_drone = nedD(1);  % X = NORTH
y_drone = nedD(2);  % Y = EAST
z_drone = nedD(3);  % Z = DOWN

% Optimal angle THETA
opt_theta_gs = atan2(x_drone,y_drone);

% Phi error calculations: GAMMA
opt_phi_gs = atan2(-z_drone,sqrt(abs(x_drone)^2 + abs(y_drone)^2));

% Error angles
alpha_gs = opt_theta_gs - theta_gs;
gamma_gs = opt_phi_gs - phi_gs;

end


