%% 832 Group Project - Signal Coverage Map
clear all; close all; clc;
% Input parameters
h_g = 20;     % [m] groundstation altitude
h_d = 100;    % [m] drone altitude

%% 1. Import map as a Web Map Service (WMS)
latDK = [53 58];
lonDK = [8 13];
layers = wmsfind('esa.int', 'SearchField', 'serverurl');
gtopo30Layer = refine(layers, 'gtopo30');
gtopo30Layer = wmsupdate(gtopo30Layer);
gtopo30Layer.Latlim = latDK;    
gtopo30Layer.Lonlim = lonDK;   
oceanColor = [0 170 255];
[A, R] = wmsread(gtopo30Layer, 'BackgroundColor', oceanColor);

%% 2. LOS from ground station to drone
figure
worldmap(latDK, lonDK)
geoshow(A, R, 'DisplayType', 'texturemap')
title({'GTOPO30 Elevation Model',gtopo30Layer.LayerTitle})
[lat lon] = inputm(2);  % input 2 points on map to get lat and lon
Z = double(A(:,:,1));   % taking one layer of the map
e_g = ltln2val(Z, R, lat(1), lon(1)) + h_g; % [m] terrain + station height
e_d = ltln2val(Z, R, lat(2), lon(2)) + h_d; % [m] terrain + drone height
D = 3.57*(sqrt(e_g) + sqrt(e_d))            % [km] horizon (LOS) distance 
los2(Z, R, lat(1), lon(1), lat(2), lon(2), h_g, h_d)

%% 3. LOS coverage map
figure
worldmap(latDK, lonDK)
geoshow(A, R, 'DisplayType', 'texturemap')
demcmap(double(A))   
title({'GTOPO30 Elevation Model',gtopo30Layer.LayerTitle})
[latG lonG] = inputm(1);  % input groundstation location
Re = earthRadius('meters');
[vmap, vmapl] = viewshed(Z, R, latG(1), lonG(1), h_g, h_d, ...
    'AGL', 'AGL', Re, 4/3*Re);
for i=1:3
    vis(:,:,i) = uint8(vmap*500);
end
map = A + vis;
geoshow(map, R, 'DisplayType', 'texturemap')
title({'LOS COVERAGE MAP',gtopo30Layer.LayerTitle})
plotm(latG,lonG,'bo');


