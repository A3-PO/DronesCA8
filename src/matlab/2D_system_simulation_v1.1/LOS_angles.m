function [alpha_d,alpha_gs] = LOS_angles(x_drone,y_drone,theta_d,x_gs,y_gs,theta_gs)
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

% Transform theta to 0:2*pi for simpler calculations
if (theta_gs) < 0
    theta_gs = theta_gs + 2*pi;
end
if (theta_d) < 0 
    theta_d = theta_d + 2*pi;
end

%% Theta error calculations: ALPHA
% Optimal angles
if y_drone > y_gs
    opt_theta_gs = atan2(y_drone-y_gs,x_drone-x_gs);
    opt_theta_d = pi + opt_theta_gs;
else
    opt_theta_d = atan2(y_gs-y_drone,x_gs-x_drone);
    opt_theta_gs = pi + opt_theta_d;
end

% Error angles
alpha_gs = abs(theta_gs - opt_theta_gs);
alpha_gs = rad2deg(alpha_gs);
alpha_d = abs(theta_d - opt_theta_d);
alpha_d = rad2deg(alpha_d);

% Transform to -180:180
if alpha_gs > 180
    alpha_gs = 360 - alpha_gs;
end
if alpha_d > 180
    alpha_d = 360 - alpha_d;
end

% 
% beta = atan2(abs(y_drone-y_gs),abs(x_gs-x_drone));
% 
% % If the drone is lower than the Ground Station the angles are exchanged
% if x_gs < x_drone
%     alpha_gs = abs(abs(theta_gs) - beta);
%     alpha_gs = rad2deg(alpha_gs);
%     alpha_d = abs(pi - (beta + abs(theta_d)));
%     alpha_d = rad2deg(alpha_d);
% end
% if x_gs >= x_drone
%     alpha_gs = abs(pi - (beta + abs(theta_gs)));
%     alpha_gs = rad2deg(alpha_gs);
%     alpha_d = abs(abs(theta_d) - beta);
%     alpha_d = rad2deg(alpha_d);
% end


end

