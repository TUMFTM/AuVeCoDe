function [vehicle] = store_temp_data(vehicle)
%% Description:
% Designed by:  Daniel Telschow (Technical University of Munich), Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function stores the temporary vehicle parameters in order to check the discrepancies later on
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct including the data of the vehicle
% ------------
% Output:   - vehicle: struct including the data of the vehicle

%% Store temporary vehicle dimensions
    vehicle.dimensions.temp.height=vehicle.dimensions.GZ.vehicle_height;
    vehicle.dimensions.temp.wheelbase=vehicle.dimensions.GX.wheelbase;
    vehicle.dimensions.temp.overhang_r=vehicle.dimensions.GX.vehicle_overhang_r;
    vehicle.dimensions.temp.overhang_f=vehicle.dimensions.GX.vehicle_overhang_f;
    vehicle.dimensions.temp.width=vehicle.dimensions.GY.vehicle_width;
    vehicle.dimensions.temp.length=vehicle.dimensions.GX.wheelbase+...
        vehicle.dimensions.GX.vehicle_overhang_r+vehicle.dimensions.GX.vehicle_overhang_f;
    vehicle.masses.temp_empty=vehicle.masses.vehicle_empty_weight_EU;
    
    
%% Store temporary torque, power and battery capacity   
    if vehicle.LDS.settings.filled_axles(1)
        vehicle.LDS.temp.T_max_1=vehicle.LDS.MOTOR{1, 1}.T_max;
        vehicle.LDS.temp.P_max_1=vehicle.LDS.MOTOR{1, 1}.P_max_mech;
    else
        vehicle.LDS.temp.T_max_1=0;
        vehicle.LDS.temp.P_max_1=0;
    end
    

    if vehicle.LDS.settings.filled_axles(2)
        vehicle.LDS.temp.T_max_2=vehicle.LDS.MOTOR{1, 2}.T_max;
        vehicle.LDS.temp.P_max_2=vehicle.LDS.MOTOR{1, 2}.P_max_mech;
    else
        vehicle.LDS.temp.T_max_2=0;
        vehicle.LDS.temp.P_max_2=0;
    end
    
    vehicle.battery.temp.C_batt=vehicle.battery.energy_is_gross_in_kWh;
end