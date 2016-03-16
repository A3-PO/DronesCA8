function [alpha_d,alpha_gs,gamma_d,gamma_gs] = LOS_angles(x_drone,y_drone,z_drone,theta_d,phi_d,x_gs,y_gs,z_gs,theta_gs,phi_gs)
%**************************************************************************
%
% angle_frames.m - CA8 - DRONES
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
% Function that performs the calculation for a 2 DIMENSION - 1 DEGREE OF
% FREEDOM 
%
% INPUTS:
% - x_drone = Position of the drone in X
% - y_drone = Position of the drone in Y
% - x_gs = Position of the ground station in X
% - y_gs = Position of the ground station in Y
% - theta_d = ANGLE of the drone with respect to the X world
% - theta_gs = ANGLE of the ground station with respect to the X world
%
% OUTPUTS:
% - alpha_gs = Angle from the direction of the pointing antenna of the ground station 
%   to the LINE OF SIGHT that joins ground station and drone.
% - alpha_d = Angle from the direction of the pointing antenna of the drone to the 
%   LINE OF SIGHT that joins ground station and drone.
%
%**************************************************************************

% Theta: Azimuthal angle - [-pi:pi] - 0 = pointing along X axis
beta = atan2(abs(y_drone-y_gs),abs(x_gs-x_drone));
% If the drone is lower than the Ground Station the angles are exchanged
if x_gs < x_drone
    alpha_gs = abs(abs(theta_gs) - beta);
    alpha_gs = rad2deg(alpha_gs);
    alpha_d = abs(pi - (beta + abs(theta_d)));
    alpha_d = rad2deg(alpha_d);
end
if x_gs >= x_drone
    alpha_gs = abs(pi - (beta + abs(theta_gs)));
    alpha_gs = rad2deg(alpha_gs);
    alpha_d = abs(abs(theta_d) - beta);
    alpha_d = rad2deg(alpha_d);
end

% Polar angle - [-pi/2:pi/2]. 0 = X-Y plane axis
beta = atan2(abs(z_drone-z_gs),sqrt(abs(x_gs-x_drone)^2+...
    abs(y_gs-y_drone)^2));
% If the drone is lower than the Ground Station the angles are exchanged

gamma_gs = abs(abs(phi_gs) - beta);
gamma_gs = rad2deg(gamma_gs);
gamma_d = abs(abs(phi_d)-beta);
gamma_d = rad2deg(gamma_d);

end

