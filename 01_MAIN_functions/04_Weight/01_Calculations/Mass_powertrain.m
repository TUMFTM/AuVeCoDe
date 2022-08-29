function [v] = Mass_powertrain(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the total weight of the powertrain. The powertrain is
%               defined as the sum of battery, electric machines, gearboxes
% ------------
% Sources:  [1] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] A. Romano, „Data-based Analysis for Parametric Weight Estimation of new BEV Concepts,“Master thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
%           [3] L. Nicoletti, A. Romano, A. König, P. Köhler, M. Heinrich and M. Lienkamp, „An Estimation of the Lightweight Potential of Battery Electric Vehicles,“ Energies, vol. 14, no. 15, p. 4655, 2021, DOI: 10.3390/en14154655.
%           [4] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters (Par): struct with input and constant values
%           - vehicle(v): struct including the data of the vehicle
% ------------
% Output:   - vehicle(v): struct including the data of the vehicle
% ------------

%% Implementation
%1) Initialize the variables:
%2) Calculate the weight of the battery in kg
%3) Calculate the weight of the electric machines in kg
%4) Calculate the weight of the gearbox in kg (with oil)
%5) Calculate weight of cooling system in kg (with coolant fluid)

%% 1) Initialize the variables
%Regressions for the mass calculation of powertrain components
regr_ASM=Par.regr.mass.ASM.eq;
regr_PSM=Par.regr.mass.PSM.eq;
regr_pw_cooling=Par.regr.mass.powertrain_cooling.eq;

%% 2)  Calculate the weight of the battery in kg
v=Mass_battery(v,Par);

%% 3) Calculate the weight of the electric machines in kg

%Noise insulation for the electric machine
noise_insulation_weight = Par.masses.noise_insulation_powertrain;%1.30;  % [kg]

%Mounting structure for each machine (does not include the housing!)
EM_mounts_weight = Par.masses.EM_mounts;                  %11.6905;      % [kg]

%Calculate the electric machine weight. It contains the mass of stator,
%rotor, sensors, housing, electrical and cooling connectors.
for axle=1:2
    
    if ~isempty(v.gearbox{axle}) %-> axle=1 is the front axle; axle=2 is the rear axle
        
        e_machine=v.e_machine{axle};
        type=e_machine.type;
        T_max=e_machine.T_max; %in Nm
        quantity=e_machine.quantity;
        power(axle)=e_machine.P_max_mech;
        
        if strcmp(type,'ASM') %asyncronous
            EM_weight(axle) = (regr_ASM(T_max) + noise_insulation_weight + EM_mounts_weight)*quantity;  %[kg]
        elseif strcmp(type,'PSM')  %PSM, permanent magnet syncronous
            EM_weight(axle) = (regr_PSM(T_max) + noise_insulation_weight + EM_mounts_weight)*quantity;  %[kg]
        else
            disp('Error, the given machine type is not recognized. The possibilities are PSM or ASM');
        end   
        
        v.e_machine{axle}.weight= EM_weight(axle)/quantity;
        
    end    
end

%Calculate total combined power
power_tot=sum(power);

%% 4) Calculate the weight of the gearbox in kg (with oil)
%Mass of the gearbox, the shaft and the oil contained in it
gearbox_oil_weight = Par.masses.transmission_fluid;       % [kg]

%Multiply by the number of electric machines on each axle and distinguish
%gearbox type and construction
for axle=1:2
    
    if ~isempty(v.gearbox{axle}) %-> axle=1 is the front axle; axle=2 is the rear axle
        
        gearbox=v.gearbox{axle};
        quantity=v.e_machine{axle}.quantity;
        e_machine_length=v.e_machine{axle}.CY_e_machine_length;
        
        if (strcmp(gearbox.Input.type, 'lay-shaft'))

            if (strcmp(gearbox.Input.axles,'two-speed')==0)
                % Calculation of gearbox mass in kg (parallel and coaxial design)
                gearbox = calc_mass(gearbox);
                % Calculation of gearbox moment of inertia in kg*mm^2 (parallel and coaxial design)
                gearbox = calc_J(gearbox);
                gearbox.drive_shaft=set_drive_shaft(gearbox,e_machine_length);
            else
                % Calculation of gearbox mass in kg (two-speed design)
                gearbox = calc_mass_2sp(gearbox);
                % Calculation of gearbox moment of inertia in kg*mm^2 (two-speed design)
                gearbox = calc_J_2sp(gearbox);
                gearbox.drive_shaft=set_drive_shaft(gearbox,e_machine_length);
            end
        elseif (strcmp(gearbox.Input.type, 'planetary'))
            % Calculation of gearbox mass in kg
            gearbox = calc_mass_plan(gearbox);
            % Calculation of moment of inertia in kg*mm^2
            gearbox = calc_J_plan(gearbox);
            gearbox.drive_shaft=set_drive_shaft(gearbox,e_machine_length);
        end
               
        gearbox_weight(axle)=gearbox.masses.m_tot*quantity;
        %weight of two driveshafts per axle
        driveshafts_weight(axle) = gearbox.drive_shaft.mass_regression;
        %add oil weight
        oil_weight(axle)=gearbox_oil_weight*quantity;
        v.gearbox{axle}.masses=gearbox.masses;
       
    end
end

%% 5) Calculate weight of cooling system in kg (with coolant fluid)

%If the vehicle has an AWD topology, more coolant is needed, since more machine have to be cooled
if ~isempty(v.gearbox{1}) && ~isempty(v.gearbox{2}) %AWD
    pw_coolant=Par.masses.coolant.AWD;
else %RWD or FWD
    pw_coolant=Par.masses.coolant.FWDorRWD;
end

cooling_weight = regr_pw_cooling(power_tot)+pw_coolant;%9.89+0.056*power_tot+coolant_weight; %[kg]

%Check that the result is not negative
check_mass(cooling_weight);
%% Outputs 

%Output motor and transmission weights
if length(gearbox_weight)==2  %AWD=> output front and rear motor and gearbox
    v.masses.powertrain.gearbox_weight_front=gearbox_weight(1)+driveshafts_weight(1)+oil_weight(1);
    v.masses.powertrain.motor_weight_front=EM_weight(1);
    v.masses.powertrain.gearbox_weight_rear=gearbox_weight(2)+driveshafts_weight(2)+oil_weight(2);
    v.masses.powertrain.motor_weight_rear=EM_weight(2);
else %FWD or RWD
    v.masses.powertrain.gearbox_weight=gearbox_weight+driveshafts_weight+oil_weight;
    v.masses.powertrain.motor_weight=EM_weight; 
end

%Cooling system
v.masses.powertrain.cooling_weight=cooling_weight;
end

