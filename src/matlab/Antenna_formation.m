%**************************************************************************
%
% Antenna_formation.m - CA8 - DRONES
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
% Code to test and check out different types of antennas so they can be
% implemented in GSantenna function after.
%
%**************************************************************************

close all;
clear all;
clc;

% Parameters
precision = 200;    % [samples]
aprox = 40;         % [dB]
ticks = cellstr(['-30';'-20';'-10';'  0']); % Ticks in mmpolar
f = 5;              % + freq = + secondary lobes = - HPBW

% Angles
theta = [-pi:2*pi/precision:pi-(2*pi/precision)];
phi = [-pi:2*pi/precision:pi-(2*pi/precision)];

% 2D: sinc
U2d = sin(f*theta)./(f*theta);
U2d(precision/2 + 1) = 1;
U2d= abs(U2d);
% 3D: Surface formation
[THETA, PHI] = meshgrid(theta,phi);
R = sqrt(THETA.^2 + PHI.^2) + eps;
U3d = sin(f*R)./(f*R);
U3d = abs(U3d);

% Transform to dB:
% - Define in terms of dB
% - We add the scale we want to plot
% - Set whatever is less than the scale to 0
% 2D
U2db = 20*log10(abs(U2d));
U2db = U2db + aprox;
U2db(U2db<0) = 0;
% 3D
U3db = 20*log10(U3d);
U3db = U3db + aprox;
U3db(U3db<0) = 0;

% Transform to SPHERICAL COORDINATES
xp = U2d.*cos(theta);
yp = U2d.*sin(theta);
xf = repmat(xp',size(phi));
yf = yp'*cos(phi);
zf = yp'*sin(phi);

figure();
% 2D PLOT
% mmpolar(theta,U2db,'TGridColor',[0 0 0],'RGridColor',[0 0 0],'RTickLabel',ticks);
% 3D PLOT
% mesh(THETA,PHI,U3db);
mesh(xf,yf,zf);
grid on;
grid minor;
str = sprintf('Radiation Intensity [dB]');
title(str);