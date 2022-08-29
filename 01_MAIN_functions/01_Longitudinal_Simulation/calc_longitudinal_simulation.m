function [vehicle] = calc_longitudinal_simulation(vehicle,Parameters)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 22.03.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function conducts a Longitudinal dynamic simulation, which is 
%              used for the sizing of the powertrain components (battery,
%              electric machines, gearbox). The simulation was created in
%              different theses, which are listed in the Sources section.
% ------------
% Sources: [1] Adrian König, "Methodik zur Auslegung von elektrischen und autonomen Fahrzeugkonzepten
%          [2] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022

%          The following theses developed and improved the longitudinal simulation
%          [3] K. Moller, „Validierung einer MATLAB Längsdynamiksimulation für die Auslegung von Elektrofahrzeugen,“ Bachelor thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2020. 
%          [4] K. Moller, „Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
%          [5] R. Hefele, „Implementierung einer MATLAB Längsdynamiksimulation für Elektrofahrzeuge,“Semester thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2019.

%          The simulation presented here is based on the following open-source repository
%          [6] A. König, L. Nicoletti and K. Moller. „Modular Quasi Static Longitudinal Simulation for BEV,“ 2020. [Online]. Available: https://github.com/TUMFTM/Modular-Quasi-Static- Longitudinal-Simulation-for-BEV [visited on 01/07/2020].

%          More information regarding the open-source repository is given at:
%          [7] A. König, L. Nicoletti, S. Kalt, K. Moller, A. Koch and M. Lienkamp, „An Open-Source Modular Quasi-Static Longitudinal Simulation for Full Electric Vehicles,“ in 15th International Conference on Ecological Vehicles and Renewable Energies, Monte-Carlo, Monaco, 2020, pp. 1–9, DOI: 10.1109/EVER48776.2020.9242981.
% ------------
% Input: vehicle: The vehicle structure -> Stores the calculated component volumes and masses
%        Parameters: The Parameters structure -> Stores the constant values and regressions for volume and mass models
% ------------
% Output: The vehicle structure updated with:
%         -the vehicle consumption
%         -the required battery energy
%         -the resistance forces
%         -the achievable range
%         -the machine torque, rotational speed etc.
%         -the results of acceleration and max speed simulation
% ------------
%% Implementation
%1) Preprocessing
%2) Load cycles, efficiency map, and calculate missing inputs
%3) Acceleration Simulation
%4) Top Speed simulation    
%5) Energy Consumption Simulation   
%6) Clear unused data

%% 1) Preprocessing
%Simulation settings
Parameters.LDS.char_path                   =   '02_Characteristics_Diagrams'; %Path where engine characteristics are stored within the folder /Mainfolder/03_LDS_Hefele/XXXXX
Parameters.LDS.max_diff_to_acc             =   0.1;                                                  %tolerance of acceleration time in s
Parameters.LDS.max_diff_to_max_speed       =   2;                                                    %tolerance of actual speed to max speed in km/h
Parameters.LDS.t_sim                       =   25;                                                   %maximum acceleration simulation time in s
Parameters.LDS.v_max_sim                   =   100;                                                  %max speed in acceleration simulation in km/h
Parameters.LDS.delta_t                     =   0.1;                                                  %step size in acceleration simulation in s
Parameters.LDS.delta_t_con                 =   0.1;                                                  %step size in consumption simulation in s
Parameters.LDS.axle_load_front             =   vehicle.masses.optional_extras.front_repartition/100; %axis load distribution on front axis 
Parameters.LDS.axle_load_rear              =   1-Parameters.LDS.axle_load_front;                     %axis load distribution on rear axis 
Parameters.LDS.slope_angle_sim_acc         =   0;                                                    %Slope angle for acceleration simulation in °
vehicle.masses.vehicle_sim_cons_weight     =   NaN; %Set only if the mass used for the consumption simulation (by default the empty vehicle mass) is different from the mass used for the acceleration simulation
vehicle.LDS.settings.suppress_LDS_warnings =   vehicle.settings.suppress_warnings; %Set to 1 to suppress the LDS warnings and to 0 to show them

%If there is a given torque ratio (front to total distribution), set the torque ratio requirements for the machine (only relevant for AWD)
if count(vehicle.Input.topology,'G') == 2
    topology = split(vehicle.Input.topology,'_');
    % number of engines front
    if topology(1) == "2G2M"
        num_f = 2;
    else
        num_f = 1;
    end
    % number of engines rear
    if topology(2) == "2G2M"
        num_r = 2;
    else
        num_r = 1;
    end
    % assign torque ratio
    vehicle.Input.T_max_Mot_f = vehicle.Input.torque_ratio/num_f;
    vehicle.Input.T_max_Mot_r = (1 - vehicle.Input.torque_ratio)/num_r;
end

%Assign a marker if this is the first loop/run of the LDS
if ~isfield(vehicle.LDS,'First_Run')
    vehicle.LDS.First_Run = 1;
else
    vehicle.LDS.First_Run = 0;
end

%Check that the inputs of the LDS are assigned correctly
check_LDS_inputs(vehicle);

%% 2) Load cycles, efficiency map, and calculate missing inputs
%Load the selected consumption cycle
vehicle = load_cycle(vehicle,Parameters);          

%Initialize the topology
vehicle = initialize_topology(vehicle,Parameters);   

%Calculate missing inputs (dependent on the given Inputs): n_max, gear ratio 
vehicle = calc_missing_inputs(vehicle,Parameters); 

%Load the efficiency map for the motor
vehicle.LDS = load_engine(vehicle.LDS);            

%% 3) Acceleration simulation
%-This simulation is necessary to scale the motor characteristic, which will be needed for the energy consumption simulation
%-The machine torque will be scaled to reach the given acceleration time
%-The output is a definition of the required motor characteristic (T_max, n_max and the gear ratios)
vehicle = acceleration_sim(vehicle,Parameters);

%% 4) Top speed simulation
vehicle = max_speed_sim(vehicle,Parameters);

%% 5) Energy consumption simulation
%-Also using the results of the acceleration simulation, an energy consumptuon calculate the cosumption for the choosen cycle
%-If the motor/motors do not have enough torque for the cycle, the torque will be scaled accordingly
vehicle = energy_consumption_sim(vehicle, Parameters);  

%% 6) Clear unused data
%Activate only if you want to run a big optimization and therefore reduce the size of the vehicle structure.
%Otherwise uncomment this line of code
vehicle = clear_struct(vehicle);
end