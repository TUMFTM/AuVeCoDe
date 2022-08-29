function [vehicle] = CALCULATE_mass(vehicle,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.03.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates the mass of every component
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - varargin: if optimization is active, optimization variables
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------

%% 1) Preperations to avoid leaving boundaries
%Read Parameters
wheelbase_temp  = vehicle.dimensions.GX.wheelbase;         % in mm
width_temp      = vehicle.dimensions.GY.vehicle_width;     % in mm
height_temp     = vehicle.dimensions.GZ.vehicle_height;    % in mm
length_temp     = vehicle.dimensions.GX.vehicle_length;    % in mm

%Check if Parameter out of Boundaries
if wheelbase_temp<2200
    vehicle=errorlog(vehicle,'Wheelbase is out of boundary and is set to 2200 mm for weight calculation. Results could be distorted!');
    vehicle.dimensions.GX.wheelbase=2200;
end

if width_temp<1800
    vehicle=errorlog(vehicle,'Width is out of boundary and is set to 1800 mm for weight calculation. Results could be distorted!');
    vehicle.dimensions.GY.vehicle_width=1800;
end

if height_temp<1300
    vehicle=errorlog(vehicle,'Height is out of boundary and is set to 1300 mm for weight calculation. Results could be distorted!');
    vehicle.dimensions.GZ.vehicle_height=1300;
end

if length_temp<2500
    vehicle=errorlog(vehicle,'Length is out of boundary and is set to 2500 mm for weight calculation. Results could be distorted!');
    vehicle.dimensions.GX.vehicle_length=2500;
end


%% 2) Calculate weight of component groups
vehicle=Mass_chassis(vehicle,Parameters);
vehicle=Mass_powertrain(vehicle,Parameters);
vehicle=Mass_exterior(vehicle,Parameters);
vehicle=Mass_EE(vehicle,Parameters);
vehicle=Mass_frame(vehicle,Parameters);
vehicle=Mass_accessories(vehicle,Parameters);
vehicle=Mass_interior(vehicle,Parameters);


%% 3) Write back original dimensions
%Read Parameters
vehicle.dimensions.GX.wheelbase=wheelbase_temp;                              % in mm
vehicle.dimensions.GY.vehicle_width= width_temp;                            % in mm
vehicle.dimensions.GZ.vehicle_height=height_temp;                              % in mm
vehicle.dimensions.GX.vehicle_length=length_temp;                                % in mm


%% 4) Recalculate the new total vehicle weight
vehicle=update_weight(vehicle,Parameters);
end


