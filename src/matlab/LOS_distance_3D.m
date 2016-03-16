function [dxVector,dyVector,dzVector,los_d] = LOS_distance_3D(x_drone,y_drone,z_drone,x_gs,y_gs,z_gs,prec_d)
%**************************************************************************
%
% LOS_distance_3D.m - CA8 - DRONES
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
% Function to check and calculate the distance vector between Drone and
% Ground station in 3 DIMENSIONS.
% It checks weather the drone or the ground station is first in both X and
% Y axis, so that the direction of the vector is correctly calculated for
% the plot.
%
% INPUTS:
% - x_drone = Position of the drone in X
% - y_drone = Position of the drone in Y
% - z_drone = Position of the drone in Z
% - x_gs = Position of the ground station in X
% - y_gs = Position of the ground station in Y
% - z_gs = Position of the ground station in Z
% - prec_d = Precision of the output distance vector
%
% OUTPUTS:
% - dxVector = Distance vector in X
% - dyVector = Distance vector in Y
% - dzVector = Distance vector in Z
%
%**************************************************************************
% LOS distance vector construction

if x_gs > x_drone % Drone is first in the X axis
    dxVector = [x_drone:(x_gs - x_drone)/prec_d:x_gs];
elseif x_gs < x_drone % GS is first in the X axis
    dxVector = [x_drone:-(x_drone - x_gs)/prec_d:x_gs];
else
    dxVector = x_gs*ones(1,prec_d+1);
end

if y_gs > y_drone % Drone is first in the Y axis
    dyVector = [y_drone:(y_gs - y_drone)/prec_d:y_gs];
elseif y_gs < y_drone
    dyVector = [y_drone:-(y_drone - y_gs)/prec_d:y_gs];
else
    dyVector = y_gs*ones(1,prec_d+1);
end

if z_gs > z_drone % Drone is first in the Z axis
    dzVector = [z_drone:(z_gs - z_drone)/prec_d:z_gs];
elseif z_gs < z_drone
    dzVector = [z_drone:-(z_drone - z_gs)/prec_d:z_gs];
else
    dzVector = z_gs*ones(1,prec_d+1);
end

los_d = sqrt(abs(dzVector(end)-dzVector(1))^2 + abs(dyVector(end) - ...
    dyVector(1))^2 + abs(dxVector(end)-dxVector(1))^2);

end

