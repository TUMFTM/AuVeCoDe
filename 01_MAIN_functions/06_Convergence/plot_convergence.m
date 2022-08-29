function [vehicle]=plot_convergence(vehicle,Parameters)
%% Description:
% Designed by:  Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This plots values for every iteration to check convergence
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - vehicle: struct including the data of the vehicle
% ------------
% Output:   - vehicle: struct including the data of the vehicle
%           - plot of convergence
%
% ------------

%% Initialize parameters in first loop

if vehicle.iteration==1
    if isempty(vehicle.LDS.MOTOR{1, 2}) %Check if no front motor
        vehicle.convergence.motor=vehicle.LDS.MOTOR{1, 1}.T_max;
    else
        vehicle.convergence.motor=vehicle.LDS.MOTOR{1, 2}.T_max;
    end
    vehicle.convergence.battery=vehicle.battery.energy_is_gross_in_kWh;
    vehicle.convergence.range=vehicle.LDS.range;
    vehicle.convergence.mass=vehicle.masses.vehicle_empty_weight  ;
    vehicle.convergence.length=vehicle.dimensions.GX.vehicle_length;
    return
end



%% If vehicle is not feasible: read parameters, if feasible plot parameters
if ~isempty(vehicle.feasible) || (vehicle.iteration+1) >= Parameters.settings.max_iteration
    %Check why vehicle is finished
    if (vehicle.iteration+1) >= Parameters.settings.max_iteration %if cause is max. iteration
        iteration=vehicle.iteration-1; %reduce iteration with one (see check_LDS_change)
    else
        iteration=vehicle.iteration;
    end
    
    x=1:1:iteration; %create x vector
    figure('Name','Convergence','NumberTitle','off'); %plot figure
    %create y vectors
    y1=vehicle.convergence.motor;
    y2=vehicle.convergence.battery;
    y3=vehicle.convergence.range;
    y4= vehicle.convergence.mass;
    y5= vehicle.convergence.length;
    %plot battery, range and motor left
    yyaxis left
    plot(x,y1,'b-o',x,y2,'b--o',x,y3,'b:o')
    %plot length and weight right
    yyaxis right
    plot(x,y4,'r-o',x,y5,'r-.o')
    legend('motor','battery','range','mass','length')
    
else
    % Motor
    if isempty(vehicle.LDS.MOTOR{1, 1}) % Check if no front motor
        vehicle.convergence.motor=[vehicle.convergence.motor vehicle.LDS.MOTOR{1, 2}.T_max];
    elseif isempty(vehicle.LDS.MOTOR{1, 2}) %Check if no rear motor
        vehicle.convergence.motor=[vehicle.convergence.motor vehicle.LDS.MOTOR{1, 1}.T_max];
    else %allwheeldrive, choose only rear motor
        vehicle.convergence.motor=[vehicle.convergence.motor vehicle.LDS.MOTOR{1, 2}.T_max];
    end
    
    % Battery
    vehicle.convergence.battery=[vehicle.convergence.battery vehicle.battery.energy_is_gross_in_kWh];
    
    %Range
    vehicle.convergence.range=[vehicle.convergence.range vehicle.LDS.range];
    
    %Mass
    vehicle.convergence.mass=[vehicle.convergence.mass vehicle.masses.vehicle_empty_weight];
    
    %Length
    vehicle.convergence.length= [vehicle.convergence.length vehicle.dimensions.GX.vehicle_length];
end
end

