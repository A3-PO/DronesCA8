%% 832 Group Project - LOS Coverage Map (version 2)
clear all; close all; clc;
% Input parameters
H_gs = input('Input ground station altitude: ');   % [m] GS altitude 
H_ua = input('Input aircraft altitude: ');         % [m] UA altitude
latlim = [54 58];                                  % Map latitude limits
lonlim = [8 16];                                   % Map longitude limits
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
title({'Denmark - Topographic Map'});
[lat lon] = inputm(2);                         % INPUT GS & UA LOCATIONS
plotm(lat(1),lon(1),'o','color',[1 .5 .2],'LineWidth',3);
textm(lat(1),lon(1),'  GS','FontWeight','bold');
plotm(lat(2),lon(2),'bo','LineWidth',3);
textm(lat(2),lon(2),'  UA','FontWeight','bold');

% Personalized LOS function (LOCAL NED GS FRAME):
uas_los(Z, R, lat(1), lon(1), lat(2), lon(2), H_gs, H_ua, 'AGL', ...
                                                    'AGL', R_e, 4/3*R_e)
                                                
% Matlab LOS function:                                                
% los2(Z, R, lat(1), lon(1), lat(2), lon(2), H_gs, H_ua, 'AGL', 'AGL', ...
%                                                            R_e, 4/3*R_e)

% Theoretical LOS distance 
E_gs = ltln2val(Z, R, lat(1), lon(1)) + H_gs; % [m] TERRAIN + GS ALTITUDE
E_ua = ltln2val(Z, R, lat(2), lon(2)) + H_ua; % [m] TERRAIN + UA ALTITUDE
D_los = 3.57*(sqrt(E_gs) + sqrt(E_ua))     % [km] LOS DISTANCE

%% 3. LOS Coverage Map
figure
worldmap(latlim, lonlim)
geoshow(ZA, RA, 'DisplayType', 'texturemap')
demcmap(double(ZA))   
title('Denmark - Topographic Map')
[latGS lonGS] = inputm(1);                              % INPUT GS LOCATION  
[vmap, vmapl] = viewshed(Z, R, latGS(1), lonGS(1), H_gs, H_ua, ...
    'AGL', 'AGL', R_e, 4/3*R_e);
plotm(latGS,lonGS,'o','color',[1 .5 .2],'LineWidth',3); % GS POINT ON MAP
textm(latGS,lonGS,'  GS','FontWeight','bold');
contourm(vmap,RA,'LineColor',[1 .5 .2]);                % LOS AREA FROM GS
title('LOS COVERAGE MAP');
