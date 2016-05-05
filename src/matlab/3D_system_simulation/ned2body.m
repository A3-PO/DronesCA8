function [bodyPos] = ned2body(nedPos,yaw,pitch,roll)
%**************************************************************************
%
% ned2body.m - CA8 - DRONES
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
%   Transform from the Vehicle-Carried NED to the Body Coordinate System
%
% INPUTS:
%
% OUTPUTS:
%
%
%**************************************************************************

xNED = nedPos(1);
yNED = nedPos(2);
zNED = nedPos(3);

% PSI = YAW = Rotate on the Z axis
% THETA = PITCH = Rotate on the Y axis
% PHI = ROLL = Rotate on the X axis
psi = yaw;
theta = pitch;
phi = roll;

% Rotation Matrix from ECEF to local NED
Rbnv = [             cosd(theta)*cosd(psi)                                       cosd(theta)*sind(psi)                                -sind(theta);
        sind(phi)*sind(theta)*cosd(psi) - cosd(phi)*sind(psi)       sind(phi)*sind(theta)*sind(psi) + cosd(phi)*cosd(psi)     sind(phi)*cosd(theta);
        cosd(phi)*sind(theta)*cosd(psi) + sind(phi)*sind(psi)       cosd(phi)*sind(theta)*sind(psi) - sind(phi)*cosd(psi)     cosd(phi)*cosd(theta)];

% Transformation
bodyPos = Rbnv*nedPos;

end

