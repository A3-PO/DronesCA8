%% 832 Group Project - LOS Coverage Map (version 2)
clear all; close all; clc;
% Input parameters
H_gs = input('Input ground station altitude: ');   % [m] GS altitude 
H_ua = input('Input aircraft altitude: ');         % [m] UA altitude
latlim = [45 55];                                  % Map latitude limits
lonlim = [3 18];                                   % Map longitude limits
R_e = earthRadius('meters');                       % Earth radius

%% 1. Import map as a Web Map Service (WMS)
layers = wmsfind('nasa.network*elev', 'SearchField', 'serverurl');
layers = wmsupdate(layers);
% disp(layers,'Properties',{'LayerTitle','LayerName'})   % map info
aster = layers.refine('earthaster', 'SearchField', 'layername');
cellSize = dms2degrees([0,1,0]);
[ZA, RA] = wmsread(aster, 'Latlim', latlim, 'Lonlim', lonlim, ...
   'CellSize', cellSize, 'ImageFormat', 'image/bil');
Z = double(ZA);
R = RA;

%% 2. LOS distance from GS to UA (2 points)
figure
worldmap(latlim, lonlim)
geoshow(ZA, RA, 'DisplayType', 'texturemap')
demcmap(double(ZA))
title({'Central Europe - Topographic Map'});
[lat lon] = inputm(2);                         % Input GS and UA locations
plotm(lat(1),lon(1),'ro','LineWidth',3);textm(lat(1),lon(1),'GS');
plotm(lat(2),lon(2),'bo','LineWidth',3);textm(lat(2),lon(2),'UA');

% Personalized LOS function (LOCAL NED GS FRAME):
uas_los(Z, R, lat(1), lon(1), lat(2), lon(2), H_gs, H_ua, 'AGL', ...
                                                    'AGL', R_e, 4/3*R_e)
                                                
% Matlab LOS function:                                                
% los2(Z, R, lat(1), lon(1), lat(2), lon(2), H_gs, H_ua, 'AGL', 'AGL', ...
%                                                            R_e, 4/3*R_e)

% Theoretical LOS distance 
E_gs = ltln2val(Z, R, lat(1), lon(1)) + H_gs; % [m] terrain + GS altitude
E_ua = ltln2val(Z, R, lat(2), lon(2)) + H_ua; % [m] terrain + UA altitude
D_los = 3.57*(sqrt(E_gs) + sqrt(E_ua))     % [km] LOS distance

%% 3. LOS Coverage Map
figure
worldmap(latlim, lonlim)
geoshow(ZA, RA, 'DisplayType', 'texturemap')
demcmap(double(ZA))   
title('Central Europe - Topographic Map')
[latGS lonGS] = inputm(1);                % Input GS location  
[vmap, vmapl] = viewshed(Z, R, latGS(1), lonGS(1), H_gs, H_ua, ...
    'AGL', 'AGL', R_e, 4/3*R_e);
plotm(latGS,lonGS,'ro','LineWidth',3);    % GS point on map
textm(latGS,lonGS,'GS');
contourm(vmap,RA,'LineColor','w');        % LOS area from the GS
title('LOS COVERAGE MAP');
