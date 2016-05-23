function [GSgain,angle3db] = GSantenna(thetaGS,plotting)
%**************************************************************************
%
% GSantenna.m - CA8 - DRONES
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
U = sin(f*theta)./(f*theta);
U(precision/2 + 1) = 1;
U = abs(U);

% Transform to dB:
% - Define in terms of dB
Udb = 20*log10(U);

% Calculate gain of the given angles before transforming to plot
GSgain = Udb(ind);

% Calculate the angle in which decays 3dB
temp = abs(Udb + 3);
[~, ind3db] = min(temp);
angle3db = rad2deg(theta(ind3db));

% Transform to dB:
% - We add the scale we want to plot
% - Set whatever is less than the scale to 0
Udb = Udb + aprox;
Udb(Udb<0) = 0;

if plotting == 1  
    f1 = figure();
    plot(theta,U);
    axis([-pi pi 0 1]);
    grid on;
    grid minor;
%     str = sprintf('Radiation Intensity');
%     title(str);
%     saveas(f1, '../../doc/report/figures/sinc1.eps');
    f2 = figure();
    plot(theta,Udb);
    axis([-pi pi 0 aprox]);
    grid on;
    grid minor;
%     str = sprintf('Radiation Intensity [dB]');
%     title(str);
%     saveas(f2, '../../doc/report/figures/sinc2.eps');
    
    f3 = figure();
    polar(theta,U);
    grid on;
    grid minor;
    str = sprintf('Radiation Intensity');
    title(str);
%      print('../../doc/report/figures/RadPattern.eps','-depsc');
    
    f4 = figure();
    mmpolar(theta,Udb,'TGridColor',[0 0 0],'RGridColor',[0 0 0],'RTickLabel',ticks);
    grid on;
    grid minor;
%     str = sprintf('Radiation Intensity [dB]');
%     title(str);
    print('../../doc/report/figures/RadPattern.eps','-depsc');
end

end

