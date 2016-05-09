%% 832 Group Project - LOS Coverage Map
clear all; close all; clc;
% Input parameters
h_g = input('Input ground station altitude: ');   % [m] GS altitude 
h_d = input('Input drone altitude: ');            % [m] DRONE altitude
latDK = [53 58];    % map latitude limits
lonDK = [8 13];     % map longitude limits

%% 1. Import map as a Web Map Service (WMS)
layers = wmsfind('esa.int', 'SearchField', 'serverurl');
gtopo30Layer = refine(layers, 'gtopo30');
gtopo30Layer = wmsupdate(gtopo30Layer);
gtopo30Layer.Latlim = latDK;    
gtopo30Layer.Lonlim = lonDK;   
oceanColor = [0 170 255];
[A, R] = wmsread(gtopo30Layer, 'BackgroundColor', oceanColor);

%% 2. LOS Distance from GS to DRONE (2 points)
figure
worldmap(latDK, lonDK) % this doesnt work anymore...
geoshow(A, R, 'DisplayType', 'texturemap')
title('Denmark Topographic Map')
[lat lon] = inputm(2);  % Input 2 points on map to get lat and lon
Z = double(A(:,:,1));   
e_g = ltln2val(Z, R, lat(1), lon(1)) + h_g; % [m] terrain + station height
e_d = ltln2val(Z, R, lat(2), lon(2)) + h_d; % [m] terrain + drone height
D = 3.57*(sqrt(e_g) + sqrt(e_d))            % [km] theoretical LOS distance 
los2(Z, R, lat(1), lon(1), lat(2), lon(2), h_g, h_d)

%% 3. LOS coverage map
figure
worldmap(latDK, lonDK)
geoshow(A, R, 'DisplayType', 'texturemap')
demcmap(double(A))   
title('Denmark Topographic Map')
[latG lonG] = inputm(1);              % Input GS location
Re = earthRadius('meters');
[vmap, vmapl] = viewshed(Z, R, latG(1), lonG(1), h_g, h_d, ...
    'AGL', 'AGL', Re, 4/3*Re);
plotm(latG,lonG,'ro','LineWidth',4);  % GS point on map
contourm(vmap,R,'LineColor','w');     % LOS area from the GS
title('LOS COVERAGE MAP')
