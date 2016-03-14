function [GSgain,angle3db] = GSantenna3(thetaGS,phiGS,plotting)
%**************************************************************************
%
% GSantenna3.m - CA8 - DRONES
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
% Function including the Radiation Intensity of the GROUND STATION ANTENNA.
% It plots different figures both in natural units and decibels.
% Gives back the Gain of the antenna at the input angle and the angle at
% which the Radiation Intensity decays 3dB.
%
% INPUTS:
% - thetaGS = Angle on which the gain of the antenna wants to be checked.
% - plotting = Variable to chose if display plot or not. 1 -> YES. 0 -> NO
%
% OUTPUTS:
% - GSgain = Gain of the antenna at the angle thetaGS..
% - angle3db = Angle in degrees at which the gain of the antenna decays 3db
%
%**************************************************************************

% Parameters
precision = 200;    % [samples]
aprox = 40;         % [dB]
ticks = cellstr(['-30';'-20';'-10';'  0']); % Ticks in mmpolar
f = 5;              % + freq = + secondary lobes = - HPBW

% Angle for ploting
theta = [-pi:2*pi/precision:pi-(2*pi/precision)];
phi = [0:pi/precision:pi];
[THETA,PHI] = meshgrid(theta,phi);

% Transforming the input angle to degrees
thetaGS = deg2rad(thetaGS);
% Maping from 0:2pi to -pi:pi
if thetaGS > pi
    thetaGS = thetaGS - 2*pi;
end
% Aproximating to the closet value of theta in our vector
temp = abs(theta - thetaGS);
[ind, ind] = min(temp);
tvalue = theta(ind);

% Radiation Intensity
R = sqrt(THETA.^2 + PHI.^2) + eps;
U = sin(f*R)./(f*R);
U = abs(U);

% Transform to dB
Udb = 20*log10(abs(U));
GSgain = Udb(ind);
[ind3db ind3db] = find(Udb==-3);
angle3db = rad2deg(theta(ind));
Udb = Udb + aprox;
Udb(find(Udb<0)) = 0;

if plotting == 1  
    figure();
    subplot(211);
    plot(theta,U);
    axis([-pi pi 0 1]);
    grid on;
    grid minor;
    str = sprintf('Radiation Intensity');
    title(str);
    subplot(212);
    plot(theta,Udb);
    axis([-pi pi 0 aprox]);
    grid on;
    grid minor;
    str = sprintf('Radiation Intensity [dB]');
    title(str);
    
    figure();
    polar(theta,U);
    grid on;
    grid minor;
    str = sprintf('Radiation Intensity');
    title(str);
    
    figure();
    mmpolar(theta,Udb,'TGridColor',[0 0 0],'RGridColor',[0 0 0],'RTickLabel',ticks);
    grid on;
    grid minor;
    str = sprintf('Radiation Intensity [dB]');
    title(str);
end

end

