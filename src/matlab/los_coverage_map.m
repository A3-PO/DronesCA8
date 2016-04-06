%% 832 Group Project - LOS Coverage Map
clear all; close all; clc;

%% 1. Import map as a Web Map Service (WMS)
% Denmark latitudes and longitudes
latDK = [53 58];
lonDK = [8 13];
layers = wmsfind('esa.int', 'SearchField', 'serverurl');
gtopo30Layer = refine(layers, 'gtopo30');
gtopo30Layer = wmsupdate(gtopo30Layer);
gtopo30Layer.Latlim = latDK;    
gtopo30Layer.Lonlim = lonDK;   
oceanColor = [0 170 255];
[A, R] = wmsread(gtopo30Layer, 'BackgroundColor', oceanColor);

%% 2. LOS distance from ground station to drone (2 points)
h_g = input('Input ground station altitude: ');   % [m] station altitude 
h_d = input('Input drone altitude: ');            % [m] drone altitude
figure
worldmap(latDK, lonDK)
geoshow(A, R, 'DisplayType', 'texturemap')
title({'GTOPO30 Elevation Model',gtopo30Layer.LayerTitle})
[lat lon] = inputm(2);      % input 2 points on map to get lat and lon
Z = double(A(:,:,1));       % map layer
e_g = ltln2val(Z, R, lat(1), lon(1)) + h_g; % [m] terrain + station height
e_d = ltln2val(Z, R, lat(2), lon(2)) + h_d; % [m] terrain + drone height
H = 3.57*(sqrt(e_g) + sqrt(e_d)) % [km] theoretical horizon (LOS) distance 
los2(Z, R, lat(1), lon(1), lat(2), lon(2), h_g, h_d)

%% 3. LOS Coverage Map (Area)
figure
worldmap(latDK, lonDK)
geoshow(A, R, 'DisplayType', 'texturemap')
demcmap(double(A))   
title({'GTOPO30 Elevation Model',gtopo30Layer.LayerTitle})
[latG lonG] = inputm(1);  % input ground station location
Re = earthRadius('meters');
[vmap, vmapl] = viewshed(Z, R, latG(1), lonG(1), h_g, h_d, ...
    'AGL', 'AGL', Re, 4/3*Re);
for i=1:3
    vis(:,:,i) = uint8(vmap*500);
end
map = A + vis;
geoshow(map, R, 'DisplayType', 'texturemap')
title({'LOS COVERAGE MAP',gtopo30Layer.LayerTitle})
plotm(latG,lonG,'bo')


