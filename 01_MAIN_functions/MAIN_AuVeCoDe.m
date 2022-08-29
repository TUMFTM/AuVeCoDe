function [vehicle] = MAIN_AuVeCoDe(Parameters,varargin)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Main function for calculation a vehicle with a given input
% ------------
% Sources:  [1] Adrian König, S. Mayer, L. Nicoletti, S. Tumphart, und M. Lienkamp, “The Impact of HVAC on the Development of Autonomous and Electric Vehicle Concepts,” Energies, Bd. 15, Rn. 2, S. 441, 2022.
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - varargin: if optimization is active, optimization variables
% ------------
% Output:   - vehicle: struct including the data of the vehicle
% ------------

%% Implementation
% 1) Initialize vehicle struct
%    Including a initial calculation of the needed battery capacity, mass
%    and powertrain size
% 2) While loop for iterative design of the vehicle concept
%   a) Package design
%   b) Battery design
%   c) Trunk/Frunk calculation
%   d) Silhouette calculation
%   e) Chassis calculation
%   f) Auxiliary users calculation
%   g) Mass calculation
%   h) Longitudinal dynamic simulation
%   i) Check for convergence
%   j) Calculate cost

%% 1) Initialize vehicle struct
%Read in of Input data and optional the optimization  variables and creates the vehicle struct
[vehicle] = INITIALIZE_vehicle(Parameters,varargin);

%% 2) Loop until min. user requirements are satisfied
%Iteratively calculate parts of the vehicle concept and check feasibility
%Exit while loop after max. iterations
while vehicle.iteration < Parameters.settings.max_iteration
    % Reset errorlogs
    vehicle.ErrorLog={}; % Error counter for packaging
    vehicle.LDS.ErrorLog={}; % Error counter for LDS
    vehicle.Error=0;
    
    %% a) Package design
    %Design package of interior, front and rear wagon
    [vehicle] = DESIGN_package(vehicle,Parameters);
    if vehicle.Error==1 % skip current iteration if error encountered
        vehicle.iteration=vehicle.iteration+1;
        continue
    elseif vehicle.Error>1 % vehicle concept not feasible
        break
    end
    
    %% b) Battery design
    %Design package and electric setup of battery 
    [vehicle] = DESIGN_battery(vehicle,Parameters);
    if vehicle.Error==1 % skip current iteration if error encountered
        vehicle.iteration=vehicle.iteration+1;
        continue
    elseif vehicle.Error>1 % vehicle concept not feasible
        break
    end
    
    %% c) Trunk/Frunk calculation
    %Design trunk
    [vehicle] = DESIGN_trunk_space(vehicle,Parameters);
    if vehicle.Error>1
        break
    end
    
    %% d) Silhouette calculation
    %Design silhouette of exterior
    [vehicle]=DESIGN_Silhouette(vehicle,Parameters);
    if vehicle.Error>1
        break
    end
    
    %% e) Chassis calculation
    %Calculate package of structure
    [vehicle]=Package_Structure(vehicle,Parameters);
    [vehicle]=Package_Structure_R(vehicle,Parameters);
    
    %% f) Auxiliary users calculation
    %Calculate consumption of auxiliary consumers
    [vehicle] = CALCULATE_auxiliary(vehicle,Parameters);
    
    %% g) Mass calculation
    %Estimate mass of all parts of current vehicle
    [vehicle] = CALCULATE_mass(vehicle,Parameters);
        
    %% h) Longitudinal dynamic simulation
    %Calculate motor size and consumption depending on the max. acceleration, max. speed and driving cycle
    [vehicle] = calc_longitudinal_simulation(vehicle,Parameters);
    
    %% i) Convergence check
    %Check if difference between current vehicle and last-loop-vehicle is smaller then tolerance
    [vehicle]=check_LDS_change(vehicle,Parameters);
    
    %% j) Calculate cost
    %Calculate cost of vehicle is feasible
    if vehicle.feasible==1
        [vehicle.Cost.Manufacturing_Cost,vehicle.Cost.Component_Groups] = CALCULATE_cost(vehicle,Parameters.timeframe,Parameters.Databank,0,0);
        return
    end
    
end

%% Unfeasible vehicle after maximal iterations
% Set errorlog to 99 (iteration-maxiumum achieved) for GUI-Msg
if vehicle.iteration >= Parameters.settings.max_iteration
    if vehicle.Error<2 
        vehicle.Error=99;
    end
end
vehicle.feasible=0; % set vehicle as not feasible

end

