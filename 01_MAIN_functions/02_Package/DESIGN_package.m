function [vehicle] = DESIGN_package(vehicle,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow
%-------------
% Created on: 01.01.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the vehicle package and all the component
%               dimensions
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle

%% Implementation
%1) Identify drivetrain and gearbox type
%2) Calculate interior measurements
%3) Calculate wheelhouse and chassis dimensions
%4) Calculate powertrain dimensions
%5) Calculate front wagon measurements
%6) Calculate rear wagon measurements
%7) Calculate exterior measurements

%% 1) Identify drivetrain and gearbox type
[drivetrain_type,vehicle] = Package_assign_topology(vehicle); % Subfunction to decrease complexity

%% 2) Calculate interior measurements
[vehicle] = Package_Interior_Calc(vehicle,Parameters);
if vehicle.Error>0
    return
end
[vehicle] = Package_Interior(vehicle);
if vehicle.Error>0
    return
end

%% 3) Calculate wheelhouse and chassis dimensions
%Calculate wheels and wheelhouse
[vehicle] = Package_wheelhouse(vehicle,Parameters);
if vehicle.Error>0
    return
end
%Calculate shockabsorber of rear axle
[vehicle] = Package_Shock_absorber_dim(vehicle,Parameters);
[vehicle] = Package_Shock_absorber_pos(vehicle);

%% 4) Calculate powertrain dimensions
[vehicle] = Package_Powertrain(vehicle,Parameters,drivetrain_type);
if vehicle.Error>0
    return
end

%% 5) Calculate front wagon measurements
[vehicle] = Package_Front(vehicle,drivetrain_type,Parameters);
if vehicle.Error>0
    return
end

%% 6) Calculate rear wagon measurements
[vehicle] = Package_Rear(vehicle,drivetrain_type,Parameters);
if vehicle.Error>0
    return
end

%% 7) Calculate exterior measurements
[vehicle] = Package_Exterior(vehicle,Parameters);
end