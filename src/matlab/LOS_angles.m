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
% Transform theta to 0:2*pi for simpler calculations
if (theta_gs) < 0
    theta_gs = theta_gs + 2*pi;
end
if (theta_d) < 0 
    theta_d = theta_d + 2*pi;
end

% Transform phi to 0:pi for simpler calculations
if (phi_gs) > 0
    phi_gs = pi/2 - phi_gs;
else
    phi_gs = pi/2 + abs(phi_gs);
end
if (phi_d) > 0
    phi_d = pi/2 - phi_d;
else
    phi_d = pi/2 + abs(phi_d);
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
    alpha_d = 360 - alpha_gs;
end

%% Phi error calculations: GAMMA
%Optimal angles
if z_drone > z_gs
    opt_phi_gs = pi/2 - atan2(z_drone-z_gs,sqrt(abs(x_gs-x_drone)^2+...
        abs(y_gs-y_drone)^2));
    opt_phi_d = pi/2 + atan2(z_drone-z_gs,sqrt(abs(x_gs-x_drone)^2+...
        abs(y_gs-y_drone)^2));
else
    opt_phi_gs = pi/2 - atan2(z_drone-z_gs,sqrt(abs(x_gs-x_drone)^2+...
        abs(y_gs-y_drone)^2));
    opt_phi_gs = pi/2 + atan2(z_drone-z_gs,sqrt(abs(x_gs-x_drone)^2+...
        abs(y_gs-y_drone)^2));
end

gamma_gs = abs(phi_gs - opt_phi_gs);
gamma_gs = rad2deg(gamma_gs);
gamma_d = abs(phi_d- opt_phi_d);
gamma_d = rad2deg(gamma_d);

end

