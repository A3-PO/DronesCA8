function [phi_d,phi_gs] = angle_frames(x_drone,y_drone,angle_d,x_gs,y_gs,angle_gs)
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
% - angle_d = ANGLE of the drone with respect to the X world
% - angle_gs = ANGLE of the ground station with respect to the X world
%
% OUTPUTS:
% - phi_gs = Angle from the direction of the pointing antenna of the ground station 
%   to the LINE OF SIGHT that joins ground station and drone.
% - phi_d = Angle from the direction of the pointing antenna of the drone to the 
%   LINE OF SIGHT that joins ground station and drone.
%
%**************************************************************************
beta = atan(abs(y_drone-y_gs)/abs(x_gs-x_drone));
phi_gs = pi - angle_gs - beta;
phi_gs = phi_gs*180/pi;
phi_d = 2*pi-(beta + angle_d);
phi_d = phi_d*180/pi;

end

