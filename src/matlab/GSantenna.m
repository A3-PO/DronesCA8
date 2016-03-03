function [GSgain,angle3db] = GSantenna(thetaGS,plotting)
%********************************************************************************
%
%	GSAntenna
%
%********************************************************************************
%
%   CA8 - DRONES
%
%********************************************************************************
%
% Alvaro Perez Ortega
% Aalborg University
% February 2016
%
%********************************************************************************

precision = 200;    % [samples]
aprox = 40;         % [dB]
ticks = cellstr(['-30';'-20';'-10';'  0']); % Ticks in mmpolar
f = 5;              % + freq = + secondary lobes = - HPBW

% Angle for ploting
theta = [-pi:2*pi/precision:pi-(2*pi/precision)];


thetaGS = deg2rad(thetaGS);
if thetaGS > pi
    thetaGS = thetaGS - 2*pi;
end
temp = abs(theta - thetaGS);
[ind, ind] = min(temp);
tvalue = theta(ind);

% Radiation Intensity
U = sin(f*theta)./(f*theta);
U(precision/2 + 1) = 1;
% U = cos(f*theta).*exp(-f*abs(theta));
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

