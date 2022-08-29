function [vehicle,Parameters] = Non_Linear_Constraints(vehicle,Parameters)
%% Function checks if non-linear constraints are fulfilled

% Author:   Daniel Telschow, Adrian König
% Date:     January 2021
% Update:   July 2021

%% Near-axle gearbox requires coaxial
if strcmp('2G2M_X',vehicle.Input.topology) && (strcmp(vehicle.Input.gear_type_f,'Parallel')||strcmp(vehicle.Input.gear_type_f,'Coaxial-layshaft'))
    vehicle.iteration=Parameters.settings.max_iteration+1;
    vehicle.Error=2;
elseif strcmp('2G2M_GM',vehicle.Input.topology) && (strcmp(vehicle.Input.gear_type_f,'Parallel')||strcmp(vehicle.Input.gear_type_f,'Coaxial-layshaft'))
    vehicle.iteration=Parameters.settings.max_iteration+1;
    vehicle.Error=2;
elseif strcmp('2G2M_2G2M',vehicle.Input.topology) && ((strcmp(vehicle.Input.gear_type_f,'Parallel') || strcmp(vehicle.Input.gear_type_r,'Parallel'))||(strcmp(vehicle.Input.gear_type_f,'Coaxial-layshaft')||strcmp(vehicle.Input.gear_type_r,'Coaxial-layshaft')))
    vehicle.iteration=Parameters.settings.max_iteration+1;
    vehicle.Error=2;
elseif strcmp('GM_2G2M',vehicle.Input.topology) && (strcmp(vehicle.Input.gear_type_r,'Parallel')||strcmp(vehicle.Input.gear_type_r,'Coaxial-layshaft'))
    vehicle.iteration=Parameters.settings.max_iteration+1;
    vehicle.Error=2;
elseif strcmp('X_2G2M',vehicle.Input.topology) && (strcmp(vehicle.Input.gear_type_r,'Parallel')||strcmp(vehicle.Input.gear_type_r,'Coaxial-layshaft'))
    vehicle.iteration=Parameters.settings.max_iteration+1;
    vehicle.Error=2;
end

%% Interior height under ground clearance
if vehicle.dimensions.GZ.H156>vehicle.Input.int_height
    vehicle.Error=9;
end

end



