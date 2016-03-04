function [dxVector,dyVector] = LOS_distance(x_drone,y_drone,x_gs,y_gs,prec_d)
%**************************************************************************
%
% CA8 - DRONES
%
%**************************************************************************
%
% Group 832 - Control and Automation Msc.
% Aalborg University
% February 2016
%
%**************************************************************************
% LOS distance vector construction
if x_gs > x_drone % Drone is first in the X axis
    dxVector = [x_drone:(x_gs - x_drone)/prec_d:x_gs];
    if y_gs > y_drone % Drone is first in the Y axis
        dyVector = [y_drone:(y_gs - y_drone)/prec_d:y_gs];
    elseif y_gs < y_drone
        dyVector = [y_drone:-(y_drone - y_gs)/prec_d:y_gs];
    else
        dyVector = y_gs*ones(1,prec_d+1);
    end
elseif x_gs < x_drone % GS is first in the X axis
    dxVector = [x_drone:-(x_drone - x_gs)/prec_d:x_gs];
    if y_gs > y_drone % Drone is first in the Y axis
        dyVector = [y_drone:(y_gs - y_drone)/prec_d:y_gs];
    elseif y_gs < y_drone
        dyVector = [y_drone:-(y_drone - y_gs)/prec_d:y_gs];
    else
        dyVector = y_gs*ones(1,prec_d+1);
    end
else
    dxVector = x_gs*ones(1,prec_d+1);
     if y_gs > y_drone % Drone is first in the Y axis
        dyVector = [y_drone:(y_gs - y_drone)/prec_d:y_gs];
    elseif y_gs < y_drone
        dyVector = [y_drone:-(y_drone - y_gs)/prec_d:y_gs];
    else
        dyVector = y_gs*ones(1,prec_d+1);
     end
end

end

