function [ecefPos] = geo2ecef(geoPos)
%**************************************************************************
%
% geo2ecef.m - CA8 - DRONES
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
%   This matlab function would translate automatically from LLA (Geodetic 
%   Coordinate system) to ECEF but MATLAB wants me to pay for the Aerospace
%   Toolbox so it has to be done manually...
%
%   ecefPos = lla2ecef(geoPos, 'WGS84');
%
%   WE ASSUME WSG84 earth model
%
% INPUTS:
% - geoPos = [Latitude; Longitude; Altitude]
%   Column Vector with the Geodetic coordinates
% OUTPUTS:
% - ecefPos = [xECEF; yECEF; zECEF]
%   Column Vector with the Earth-Centered Earth-Fixed coordiantes
%
%**************************************************************************

lat = geoPos(1);
long = geoPos(2);
alt = geoPos(3);

% WGS84 Parameters
Rea = 6378137;                  % Semi-major axis [Meters]
f = 1/298.257223563;            % Flattening factor
Reb = Rea*(1-f);                 % Semi-minor axis [Meters]
e = sqrt(Rea^2 - Reb^2)/Rea;    % First eccentricity
Me = (Rea*(1-e^2))/((1-e^2*sind(lat)^2)^(3/2));          % Meridian radius of curvature
Ne = Rea/(sqrt(1 - e^2*sind(lat)^2));        % Prime vertical radius of curvature

% Transformation
xECEF = (Ne + alt)*cosd(lat)*cosd(long);
yECEF = (Ne + alt)*cosd(lat)*sind(long);
zECEF = (Ne*(1-e^2) + alt)*sind(lat);

ecefPos = [xECEF; yECEF; zECEF];

end

