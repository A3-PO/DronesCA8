%% 832 Group Project - LOS Coverage Map (version 2)
clear all; close all; clc;
% Input parameters
h_g = input('Input ground station altitude: ');   % [m] GS altitude 
h_d = input('Input drone altitude: ');            % [m] DRONE altitude
latlim = [43 58];   % map latitude limits
lonlim = [3 18];    % map longitude limits

%% 1. Import map as a Web Map Service (WMS)
layers = wmsfind('nasa.network*elev', 'SearchField', 'serverurl');
layers = wmsupdate(layers);
% disp(layers,'Properties',{'LayerTitle','LayerName'})   % map info
aster = layers.refine('earthaster', 'SearchField', 'layername');
cellSize = dms2degrees([0,1,0]);
[ZA, RA] = wmsread(aster, 'Latlim', latlim, 'Lonlim', lonlim, ...
   'CellSize', cellSize, 'ImageFormat', 'image/bil');
Z = double(ZA);

%% 2. LOS Distance from GS to DRONE (2 points)
figure
worldmap(latlim, lonlim)
geoshow(ZA, RA, 'DisplayType', 'texturemap')
demcmap(double(ZA))
title({'Central Europe Topographic Map'});
[lat lon] = inputm(2);                       % Input GS and UAV locations
los2(Z, RA, lat(1), lon(1), lat(2), lon(2), h_g, h_d)
e_g = ltln2val(Z, RA, lat(1), lon(1)) + h_g; % [m] terrain + station height
e_d = ltln2val(Z, RA, lat(2), lon(2)) + h_d; % [m] terrain + drone height
D = 3.57*(sqrt(e_g) + sqrt(e_d))             % [km] theoretical LOS distance

%% 3. LOS Coverage Map
figure
worldmap(latlim, lonlim)
geoshow(ZA, RA, 'DisplayType', 'texturemap')
demcmap(double(ZA))   
title('Central Europe Topographic Map')
[latG lonG] = inputm(1);                % Input GS location
Re = earthRadius('meters');
[vmap, vmapl] = viewshed(Z, RA, latG(1), lonG(1), h_g, h_d, ...
    'AGL', 'AGL', Re, 4/3*Re);
plotm(latG,lonG,'ro','LineWidth',4);    % GS point on map
contourm(vmap,RA,'LineColor','w');      % LOS area from the GS
title('LOS COVERAGE MAP');
