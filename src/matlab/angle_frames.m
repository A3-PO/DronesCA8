function [phi_d,phi_gs] = angle_frames(x_drone,y_drone,angle_d,x_gs,y_gs,angle_gs)
%**************************************************************************
%
% CA8 - DRONES
%
%**************************************************************************
%
% Group 832 - Control and Automation Msc.
% Aalborg University
% February 2016
%
%**************************************************************************

beta = atan(abs(y_drone-y_gs)/abs(x_gs-x_drone));
phi_gs = pi - angle_gs - beta;
phi_gs = phi_gs*180/pi;
phi_d = 2*pi-(beta + angle_d);
phi_d = phi_d*180/pi;

end

