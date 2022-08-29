function [vehicle]=check_LDS_change(vehicle,Parameters)
%% Description:
% Designed by:  Adrian König (FTM, Technical University of Munich), Daniel Telschow (Technical University of Munich)
%-------------
% Created on: 01.05.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function checks the discrepancies in powertrain and dimension results between iteration loops
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - vehicle: struct including the data of the vehicle
% ------------
% Output:   - vehicle: struct including the data of the vehicle
% ------------

%% Switch for convergence plot (1=on, 0=off)
status_cp=0;

%% Calculate actual range and deviation
vehicle.LDS.range=((vehicle.battery.energy_is_gross_in_kWh*Parameters.battery.net_to_gross_capacity_factor)/...
    vehicle.LDS.sim_cons.consumption100km)*100; % updated vehicle range [km]

%Calculate range deviation only if range is no optimization goal
if Parameters.optimization.range_opt==0
    delta_range=abs((Parameters.input.range-vehicle.LDS.range)/vehicle.LDS.range); %Calculate deviation
elseif Parameters.optimization.range_opt==1
    delta_range=0;
end

%% Calculate Deviation of desired and reched acceleration time

delta_acc_time = (vehicle.LDS.sim_acc.acc_time_is - vehicle.LDS.sim_acc.acc_time_req);

%% Convergenceplot (optional)
if status_cp
    [vehicle]=plot_convergence(vehicle,Parameters);
end

%% Calculate deviation of powertrain and dimensions
delta_powertrain=engine_delta(vehicle); % calculate pow. delta between iterations [%]
delta_dimensions=dim_delta(vehicle); % calculate dim. delta between iterationsy [%]

%% Calculate if deviation is within accepted range
if delta_powertrain > Parameters.settings.check_LDS_delta % pow. delta is too large
    vehicle.iteration=vehicle.iteration+1;
    
elseif delta_acc_time > 0.2
    vehicle.iteration=vehicle.iteration+1;
    vehicle.Error = 10;
    
elseif delta_range>(Parameters.settings.check_LDS_delta/100) % range delta too big
    vehicle.iteration=vehicle.iteration+1;
    vehicle.Error_Detail = 'Exact range could not be achieved. Reduce or increase range.';
elseif delta_dimensions > Parameters.settings.check_LDS_delta % dim. delta is too large
    vehicle.iteration=vehicle.iteration+1;
    
elseif max(vehicle.wheels.steering_angle_f,vehicle.wheels.steering_angle_r)==36 %Check if max. steering angle is reached => max. steering radius
    vehicle.iteration=vehicle.iteration+1;
    vehicle.Error_Detail = 'Required steering angle to large. Increase turning diameter.';
elseif vehicle.LDS.range == 0  %if vehicle range is 0 in the range optimization
    vehicle.iteration=vehicle.iteration+1;
    vehicle.Error_Detail = 'Vehicle not feasible';
else
    vehicle.feasible=1; % vehicle satisfies all criteria --> concept is feasible
    %Plot convergence
    if status_cp
        plot_convergence(vehicle,Parameters);
    end
end

%% save current data for comparison in next iteration
[vehicle] = store_temp_data(vehicle); 
end

function percentage_delta=engine_delta(vehicle)
% Subfunction checks if stored and new defined engine(s) and battery
% capacities

if vehicle.LDS.settings.filled_axles(1)
    d1=100*abs((vehicle.LDS.temp.T_max_1-vehicle.LDS.MOTOR{1, 1}.T_max)/...
        vehicle.LDS.MOTOR{1, 1}.T_max); % torque discrepancy front axle
    d3=100*abs((vehicle.LDS.temp.P_max_1-vehicle.LDS.MOTOR{1, 1}.P_max_mech)/...
        vehicle.LDS.MOTOR{1, 1}.P_max_mech); % power discrepancy front axle
    
else % no front engine(s)
    d1=0;
    d3=0;
end

if vehicle.LDS.settings.filled_axles(2)
    d2=100*abs((vehicle.LDS.temp.T_max_2-vehicle.LDS.MOTOR{1, 2}.T_max)/...
        vehicle.LDS.MOTOR{1, 2}.T_max); % torque discrepancy rear axle
    d4=100*abs((vehicle.LDS.temp.P_max_2-vehicle.LDS.MOTOR{1, 2}.P_max_mech)/...
        vehicle.LDS.MOTOR{1, 2}.P_max_mech); % power discrepancy rear axle
    
else % no rear engine(s)
    d2=0; 
    d4=0;
end

% Check Battery Capacity
d5=abs((vehicle.battery.temp.C_batt-vehicle.battery.energy_is_gross_in_kWh)/vehicle.battery.temp.C_batt);

percentage_delta=max([d1 d2 d3 d4 d5]);
end

function percentage_delta=dim_delta(vehicle)
% Subfunction checks if stored and new dimensions are comparable

d1=abs((vehicle.dimensions.temp.height-vehicle.dimensions.GZ.vehicle_height)/vehicle.dimensions.temp.height);
d2=abs((vehicle.dimensions.temp.wheelbase-vehicle.dimensions.GX.wheelbase)/vehicle.dimensions.temp.wheelbase);
d3=abs((vehicle.dimensions.temp.overhang_r-vehicle.dimensions.GX.vehicle_overhang_r)/vehicle.dimensions.temp.overhang_r);
d4=abs((vehicle.dimensions.temp.overhang_f-vehicle.dimensions.GX.vehicle_overhang_f)/vehicle.dimensions.temp.overhang_f);
d5=abs((vehicle.dimensions.temp.width-vehicle.dimensions.GY.vehicle_width)/vehicle.dimensions.temp.width);
d6=abs((vehicle.masses.temp_empty-vehicle.masses.vehicle_empty_weight_EU)/vehicle.masses.temp_empty);
percentage_delta=100*max([d1 d2 d3 d4 d5 d5 d6]);
end

