%********************************************************************************
%
% CA8 - DRONES
%
%********************************************************************************
%
% Alvaro Perez Ortega
% Aalborg University
% February 2016
%
%********************************************************************************

clc; clear all; close all;

%% Horizon and Line-Of-Sight

H_GS_MAX = 20;
H_DR_MAX = 120;
H_GS_MIN = 0;
H_DR_MIN = 20;
prec = 100;

h_gs = [H_GS_MIN:(H_GS_MAX-H_GS_MIN)/prec:H_GS_MAX];      % Height of the ground station [m]
h_dr = [H_DR_MIN:(H_DR_MAX-H_DR_MIN)/prec:H_DR_MAX];      % Heigth of the drone [m]

%********************************************************************************
%
%   Ddb[km] = 3.57 * (sqrt(h_dr[m]) + sqrt(h_gs[m]))
%
%********************************************************************************

[h_gs_m,h_dr_m] = meshgrid(h_gs,h_dr);
D_horizon = 3.57 * (sqrt(h_gs_m) + sqrt(h_dr_m));

figure();
% surf(X,Y,D_horizon);
contourf(h_gs_m,h_dr_m,D_horizon,20,'ShowText','on');
grid on;
grid minor;
str = sprintf('Horizon Distance Analysis: \n Distance to Horizon [km]');
title(str);
xlabel('Height of the ground station [m]');
ylabel('Height of the drone [m]');
% zlabel('Distance to Horizon [km]');

%% Radio Link propagation

freq = 2.4 * 10^9;      % Frequency [GHz]
lambda = 3*10^8/freq;   % Wavelength [m]
d = 52;                 % Distance [km]

%********************************************************************************
%
%   Received Power [dBm] = Transmitted Power[dBm] + Gains[dB] − Losses[dB]
%
%   Prx = Ptx + Gtx + Grx - Ltx - Lfs - Lm + Grx - Lrx 
%
%   Prx = received power [dBm]
%   Ptx = transmitter output power [dBm]
%   Gtx = transmitter antenna gain [dBi]
%   Ltx = transmitter losses (coax, connectors...) [dB]
%   Lfs = path loss, usually free space loss [dB]
%   Lm = miscellaneous losses (fading margin, body loss, polarization mismatch, other losses...) [dB]
%   Grx = receiver antenna gain [dBi]
%   Lrx = receiver losses (coax, connectors...) [dB]
%
%********************************************************************************

Lfs = 20*log10(4*pi*d*10^3/lambda);
Ptx = 10*log10(1/(10^-3));
Grx_m = [5:0.5:30];
Gtx_m = [5:0.5:30];

[Gtx_m,Grx_m] = meshgrid(Gtx_m,Grx_m);
Prx = Ptx + Gtx_m + Grx_m - Lfs;

figure();
%surf(X,Y,Prx);
contourf(Gtx_m,Grx_m,Prx,20,'ShowText','on');
grid on;
grid minor;
str = sprintf('Link Budget Analysis: \nPrx[dB]');
title(str);
xlabel('Grx [dB]');
ylabel('Gtx [dB]');
% zlabel('Prx [dB]');

%% Loss through distance

d_m = [0:0.5:max(max(D_horizon))]; 

Lfs = 20*log10(4*pi*d_m*10^3/lambda);
Grx = 14;
Gtx = 24;

Prx = Ptx + Gtx + Grx - Lfs;

figure();
%surf(X,Y,Prx);
plot(d_m,Lfs)
grid on;
grid minor;
str = sprintf('Path Loss Analysis: \nLfs[dB]');
title(str);
xlabel('Distance [km]');
ylabel('Lfs [dB]');
% zlabel('Prx [dB]');