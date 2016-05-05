%**************************************************************************
%
% testFrames.m - CA8 - DRONES
%
%**************************************************************************
%
% Group 832 - Control and Automation Msc.
% Aalborg University
% May 2016
%
%**************************************************************************
%
% DESCRIPTION:
%
% INPUTS:
%
% OUTPUTS:
%
%**************************************************************************
clear all; close all; clc;

%% GEODETIC TO ECEF WGS84

% GS
lat_gs = 57.01487412;      % Degrees
long_gs = 9.98563731;      % Degrees
alt_gs = 11;               % Meters

% Drone
% lat_drone = 57.09746826;       % Degrees
% long_drone = 9.8708725;       % Degrees
% alt_drone = 2;                % Meters
lat_drone = 57.09746826;       % Degrees
long_drone = 9.86503601;       % Degrees
alt_drone = 100;                % Meters

[alpha_gs, gamma_gs, opt_theta_gs,opt_phi_gs] = optAnglesGS(lat_drone,long_drone,alt_drone,lat_gs,long_gs,...
    alt_gs,0,0)

[alpha_d, gamma_d, opt_theta_d,opt_phi_d] = optAnglesD(lat_drone,long_drone,alt_drone,lat_gs,long_gs,...
    alt_gs,0,0)

