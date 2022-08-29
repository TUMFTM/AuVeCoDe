function [Fitness,c] = Objfunction(pop,Parameters)
%% Description:
% Function evaluates the fitness of the vehicle concept based on the
% optimization parameters

% Author:   Daniel Telschow
% Date:     January 2021

%% Inputs:
% Population (vehicle parameters), User input (from GUI) and
% constraint matrix

%% Outputs:
% Vehicle fitness based on the specified objectives
% Constraint satisfaction for directed mutation of next generation

%% Implementation
% 1. Initialization and preallocation of variables
% 2. Constraints satisfaction
% 3. Vehicle Design loop

%% 1. Initialization and preallocation of variables
Fitness = zeros(1,2);                                                % Fitness matrix
%c = zeros(1,length(Parameters.LinCon.b));                            % Constraint matrix

%% 2. Constraints satisfaction (based only on optimization parameters)
lin_RB = (Parameters.LinCon.A * pop') - Parameters.LinCon.b;      % Linear constraints
c = (lin_RB .* (lin_RB>0))';
    
%% 3. Vehicle Design loop
if c(:,1)==0 % jump if any constraint is already broken
    [vehicle] = MAIN_AuVeCoDe(Parameters,pop);
    if vehicle.feasible==1
        Fitness(1,1) = round(eval(Parameters.optimization.goal_1),5,'significant');
        Fitness(1,2) = round(eval(Parameters.optimization.goal_2),5,'significant');

    else % Punishment of non-feasible designs (error inside loop --> weaker punishment)
        %Differentiate punishment depending on objective. Take either punishment value or double the current value (except range is always to 0)
        switch Parameters.optimization.goal_1_name
            case 'Production Costs in Euro'                
                Fitness(1,1)=Parameters.punishment.cost;                
            case 'Battery Capacity in kWh'
                Fitness(1,1)=Parameters.punishment.battery;
            case 'Consumption in kWh/100km'
                Fitness(1,1)=Parameters.punishment.consumption;
            case 'Empty Weight in kg'
                Fitness(1,1)=Parameters.punishment.weight;
            case 'Range in km'
                Fitness(1,1)=Parameters.punishment.range;
        end
        
        switch Parameters.optimization.goal_2_name
            case 'Production Costs in Euro'                
                Fitness(1,2)=Parameters.punishment.cost;                
            case 'Battery Capacity in kWh'
                Fitness(1,2)=Parameters.punishment.battery;
            case 'Consumption in kWh/100km'
                Fitness(1,2)=Parameters.punishment.consumption;
            case 'Empty Weight in kg'
                Fitness(1,2)=Parameters.punishment.weight;
            case 'Range in km'
                Fitness(1,2)=Parameters.punishment.range;
        end
        
    end

else % Punishment of non-feasible designs (error before loop --> stronger punishment)
    %Differentiate punishment depending on objective. Double the punishment (except range is always to 0)
        switch Parameters.optimization.goal_1_name
            case 'Production Costs in Euro'                
                Fitness(1,1)=Parameters.punishment.cost*10;                
            case 'Battery Capacity in kWh'
                Fitness(1,1)=Parameters.punishment.battery*10;
            case 'Consumption in kWh/100km'
                Fitness(1,1)=Parameters.punishment.consumption*10;
            case 'Empty Weight in kg'
                Fitness(1,1)=Parameters.punishment.weight*10;
            case 'Range in km'
                Fitness(1,1)=Parameters.punishment.range;
        end
        
        switch Parameters.optimization.goal_2_name
            case 'Production Costs in Euro'                
                Fitness(1,2)=Parameters.punishment.cost*10;                
            case 'Battery Capacity in kWh'
                Fitness(1,2)=Parameters.punishment.battery*10;
            case 'Consumption in kWh/100km'
                Fitness(1,2)=Parameters.punishment.consumption*10;
            case 'Empty Weight in kg'
                Fitness(1,2)=Parameters.punishment.weight*10;
            case 'Range in km'
                Fitness(1,2)=Parameters.punishment.range;
        end
end

end

