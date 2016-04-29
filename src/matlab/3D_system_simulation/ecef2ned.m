function [nedPos] = ecef2ned(ecefPos,geoRef)
%**************************************************************************
%
% ecef2ned.m - CA8 - DRONES
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
%   Function to translate from Earth-Centered Earth-Fixed coordinates to
%   North-East-Down with the origen in ecefOg
%
% INPUTS:
% - ecefPos = [xECEF; yECEF; zECEF]
%   Column Vector with the Earth-Centered Earth-Fixed coordinates
% - geoRef = [Latitude Reference; Longitude Reference; Altitude Reference]
%   Origen of the NED frame given in Geodetic Coordinates
% OUTPUTS:
% - nedPos = [xNED; yNED; zNED]
%   Column Vector with the NED coordinates
%
%**************************************************************************

lat = ecefPos(1);
long = ecefPos(2);
alt = ecefPos(3);

latRef = geoRef(1);
longRef = geoRef(2);
altRef = geoRef(3);

% Rotation Matrix from ECEF to local NED
Rne = [ -sind(latRef)*cosd(longRef)  -sind(latRef)*sind(longRef)    cosd(latRef);
        -sind(longRef)                  cosd(longRef)               0;
        -cosd(latRef)*cosd(longRef)  -cosd(latRef)*sind(longRef)    -sind(latRef)];

ecefRef = geo2ecef(geoRef);

% Transformation
nedPos = Rne*(ecefPos - ecefRef);
    
end

