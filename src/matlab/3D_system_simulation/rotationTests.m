%**************************************************************************
%
% sim_3D_controller.m - CA8 - DRONES
%
%**************************************************************************
%
% Group 832 - Control and Automation Msc.
% Aalborg University
% April 2016
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

%% Drone Body Rotation

% Rotation Directions
yaw = [0,0,1];
pitch = [0,1,0];
roll = [1,0,0];
prec = 180;

% Rotation Angles
yawIni = 0;
yawEnd = 90;
pitchIni = 0;
pitchEnd = 1;
rollIni = 0;
rollEnd = 360;

yawAngle = 0;
pitchAngle = 0;
rollAngle = 0;
for i = 2:prec
    yawAngle = [yawAngle yawAngle(end)+round(-1 + 2*rand(1))];
    pitchAngle = [pitchAngle pitchAngle(end)+round(-1 + 2*rand(1))];
    rollAngle = [rollAngle rollAngle(end)+round(-1 + 2*rand(1))];
end

% yawAngle = yawIni:(yawEnd-yawIni)/prec:yawEnd;
% pitchAngle = pitchIni:(pitchEnd-pitchIni)/prec:pitchEnd;
% rollAngle = rollIni:(rollEnd-rollIni)/prec:rollEnd;

% Define arrow
figure(2);
arrow = arrow3D([-1,0,0] ,[2,0,0]);
view(45, 25); 
axis ([-1.5 1.5 -1.5 1.5 -1.5 1.5]);
grid on;
grid minor;
set(gca,'TickLength',[ 0 0 ]);
set(gca,'XTickLabel',[],'YTickLabel',[],'ZTickLabel',[]);
xlabel('East Axis'); ylabel('North Axis'); zlabel('Down Axis');
title('Drone Body Rotation - Local NED Frame');
set(gca,'xticklabel',[])

for i = 2:length(yawAngle)
    yawDif = yawAngle(i)-yawAngle(i-1);
    pitchDif = pitchAngle(i)-pitchAngle(i-1);
    rollDif = rollAngle(i)-rollAngle(i-1);
    
    rotate(arrow,yaw,yawDif);
    rotate(arrow,pitch,pitchDif);
    rotate(arrow,roll,rollDif);
    drawnow
end