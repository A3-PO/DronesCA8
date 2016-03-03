%********************************************************************************
%
% CA8 - DRONES
%
%********************************************************************************
%
% Alvaro Perez Ortega
% Linneuniversitet, Växjö, Sweden
% January 2014
%
% Description:         3-D Radiation Pattern of Dipole Antenna
%
%********************************************************************************

clc; clear all; close all;

% Defining variables in spherical coordinates
theta=[0:0.12:2*pi]; %theta vector
phi=[0:0.12:2*pi];   %phi vector

l_lamda1=1/100;      %length of antenna in terms of wavelengths
I0=1;                %max current in antenna structure
n=120*pi;            %eta
 
% Evaluating radiation intensity(U)
U1=( n*( I0^2 )*( ( cos(l_lamda1*cos(theta-(pi/2))/2) - cos(l_lamda1/2) )./ sin(theta-(pi/2)) ).^2 )/(8*(pi)^2);

% Converting to dB scale
U1_1=10*log10(U1);

% Normalizing in order to make U vector positive
min1=min(U1_1);
U=U1_1-min1;
 
% Expanding theta to span entire space
U(1,1)=0;
for n=1:length(phi)
    theta(n,:)=theta(1,:);
end
% Expanding phi to span entire space
phi=phi';
for m=1:length(phi)
    phi(:,m)=phi(:,1);
end
% Expanding U to span entire space
for k=1:length(U)
    U(k,:)=U(1,:);
end
 
% Converting to spherical coordinates 
[x,y,z]=sph2cart(phi,theta,U);

% Plotting routine
surf(x,y,z)
colormap(winter)
title('Radition Pattern for Dipole Antenna (length=1.5lamda)')
xlabel('x-axis--->')
ylabel('y-axis--->')
zlabel('z-axis--->')

