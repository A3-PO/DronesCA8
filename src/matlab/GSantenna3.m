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
% - phiGS = Angle on which the gain of the antenna wants to be checked.
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
phi = [-pi:2*pi/precision:pi-(2*pi/precision)];
[THETA,PHI] = meshgrid(theta,phi);

% Transforming the input angle to degrees
thetaGS = deg2rad(thetaGS);
phiGS = deg2rad(phiGS);
% Maping from 0:2pi to -pi:pi
if thetaGS > pi
    thetaGS = thetaGS - 2*pi;
end
if phiGS > pi
    phiGS = phiGS - 2*pi;
end
% Aproximating to the closet value of angles in our vector
temp_theta = abs(theta - thetaGS);
[minval_theta, ind_theta] = min(temp_theta);
tvalue_theta = theta(ind_theta);
temp_phi = abs(phi - phiGS);
[minval_phi, ind_phi] = min(temp_phi);
tvalue_phi = phi(ind_phi);

% Radiation Intensity
R = sqrt(THETA.^2 + PHI.^2) + eps;
U = sin(f*R)./(f*R);
U = abs(U);

% Transform to dB:
% - Define in terms of dB
Udb = 20*log10(U);

% Calculate gain of the given angles before transforming to plot
GSgain = Udb(ind_phi,ind_theta);

% NOT WORKING ATM
% Calculate the angle in which decays 3dB
% temp = abs(Udb + 3);
% [~, ind3db] = min(temp);
% angle3db = rad2deg(theta(ind3db));
angle3db = 0;

% Transform to dB:
% - We add the scale we want to plot ("aprox" variable)
% - Set whatever is less than the scale to 0
Udb = Udb + aprox;
Udb(Udb<0) = 0;

if plotting == 1  
    % 3D PLOT
    figure();
    % mesh(THETA,PHI,U3db);
    % mesh(xf,yf,zf);
    contourf(Udb,'ShowText','off');
    grid on;
    grid minor;
    str = sprintf('Radiation Intensity(Theta)(Phi)\n[dB]');
    title(str);
    ax = gca;
    ax.XTick = [1 (precision/4+1) (precision/2+1) (3*precision/4+1) (precision)];
    ax.YTick = [1 (precision/4+1) (precision/2+1) (3*precision/4+1) (precision)];
    ax.XTickLabel = {'-180','-90','0','90','180'};
    ax.YTickLabel = {'-180','-90','0','90','180'};
    xlabel('Theta');
    ylabel('Phi');
    
    % Radiation patterns 3D
    figure();
    subplot(121);
    mmpolar(theta,Udb(:,size(phi,2)/2 +1),'TGridColor',[0 0 0],'RGridColor',[0 0 0],'RTickLabel',ticks);
    grid on;
    grid minor;
    str = sprintf('Radiation Intensity(Theta)\nPhi = 0\n[dB]');
    title(str);
    subplot(122);
    mmpolar(theta,Udb(size(theta,2)/2 +1,:),'TGridColor',[0 0 0],'RGridColor',[0 0 0],'RTickLabel',ticks);
    grid on;
    grid minor;
    str = sprintf('Radiation Intensity(Phi)\nTheta = 0\n[dB]');
    title(str);

end

end

