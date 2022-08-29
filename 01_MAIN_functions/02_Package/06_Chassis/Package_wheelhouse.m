function [vehicle] = Package_wheelhouse(vehicle,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow (Technical University of Munich)
%-------------
% Created on: 01.01.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates the dimension of the wheels, wheelhouses and axle height
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------

%% Parameter initialization
vehicle.topology.topology = vehicle.Input.topology;
vehicle.topology.ground_clearance = vehicle.Input.ground_clearance;

% Identify drive topology
if strcmp(vehicle.LDS.settings.drive,'front_wheel')
    vehicle.topology.drive = 'FWD';
elseif strcmp(vehicle.LDS.settings.drive,'rear_wheel')
    vehicle.topology.drive = 'RWD';
else
    vehicle.topology.drive = 'AWD';
end

%% Tire sizing
vehicle = Package_tire_dimensions(vehicle,Parameters);
if vehicle.Error>0
    return
end

%% Front wheelhouse sizing
vehicle = Package_wheelhouse_front_dim(vehicle,Parameters);

%% Rear wheelhouse sizing
vehicle = Package_wheelhouse_rear_dim(vehicle,Parameters);

%% Assign Outputs

%%%%%%%%%%%%%%%%%%% Front Wheelhouse %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Length (in y)
vehicle.dimensions.wheelhouse_length(1)         = vehicle.dimensions.CY.wheelhouse_f_width;

% Radius (x and z)
vehicle.dimensions.wheelhouse_width(1)          = vehicle.dimensions.CX.wheelhouse_f_length;

% Trackwidth
vehicle.dimensions.wheelhouse_vehicle_center(1) = vehicle.dimensions.GY.track_width_r;

% Assign values to package objects
vehicle.dimensions.p_wheelhouse_left{1}=[0 0 0;...
    vehicle.dimensions.wheelhouse_width(1)/2 vehicle.dimensions.wheelhouse_length(1) 4];

vehicle.dimensions.p_wheelhouse_right{1}=[0 0 0;...
    vehicle.dimensions.wheelhouse_width(1)/2 vehicle.dimensions.wheelhouse_length(1) 4];

%%%%%%%%%%%%%%%%%%% Rear Wheelhouse %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Length (in y)
vehicle.dimensions.wheelhouse_length(2)         = vehicle.dimensions.CY.wheelhouse_r_width;

% Radius (x and z)
vehicle.dimensions.wheelhouse_width(2)          = vehicle.dimensions.CX.wheelhouse_r_length;

% Trackwidth
vehicle.dimensions.wheelhouse_vehicle_center(2) = vehicle.dimensions.GY.track_width_r;

% Assign values to package objects
vehicle.dimensions.p_wheelhouse_left{2}=[0 0 0;...
    vehicle.dimensions.wheelhouse_width(2)/2 vehicle.dimensions.wheelhouse_length(2) 4];

vehicle.dimensions.p_wheelhouse_right{2}=[0 0 0;...
    vehicle.dimensions.wheelhouse_width(2)/2 vehicle.dimensions.wheelhouse_length(2) 4];

%%%%%%%%%%%%%%%%%%% Update axle height %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vehicle.dimensions.p_axle_height=vehicle.dimensions.CX.wheel_f_diameter/2;

end