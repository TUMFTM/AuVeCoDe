function v=update_weight(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function updates the vehicle weight at every iteration, calculating
%               the weight of each functional module
%               The output is the updated vehicle weight, along with a table representing
%               the weight variations at every iteration
% ------------
% Sources: [1] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%          [2] A. Romano, „Data-based Analysis for Parametric Weight Estimation of new BEV Concepts,“Master thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
%          [3] L. Nicoletti, A. Romano, A. König, P. Köhler, M. Heinrich and M. Lienkamp, „An Estimation of the Lightweight Potential of Battery Electric Vehicles,“ Energies, vol. 14, no. 15, p. 4655, 2021, DOI: 10.3390/en14154655.
%          [4] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters (Par): struct with input and constant values
%           - vehicle(v): struct including the data of the vehicle
% ------------
% Output:   - vehicle(v): struct including the data of the vehicle
% ------------

%% Implementation
% 1) Initialize variables
% 2) Sum elemnts of chassis
% 3) Sum elements of powertrain
% 4) Sum elements of interior
% 5) Sum elements of exterior
% 6) Sum elements of frame
% 7) Sum elements of EE
% 8) Mass of accessories
% 9) Create output table and update weight

%% 1) Initialize variables
weight_passenger = Par.masses.payload.weight_passenger;         %Mass of one passenger in kg accoding to 2007/46/EC
weight_suitcase = Par.masses.payload.weight_luggage;            %Mass of his suitcase in kg accoding to 2007/46/EC
payload = v.masses.payload.vehicle_payload;                     %Payload of the vehicle (i.e. passenger weight + maximum luggage load)
weight_extra_equipment = v.masses.payload.extra_equipement;     %If there are extra equipement in the vehicle they can be assigned here

%% 2) Sum elements of chassis
%This module is affected by secondary mass changes 
chassis=v.masses.chassis;
el_chassis=fieldnames(chassis);

mass_chassis=0;
for i=1:numel(el_chassis)
    mass_chassis=mass_chassis+chassis.(el_chassis{i});  
end

%Table chassis Create a row for every iteration, displaying the weight of each chassis component
if ~isfield(v.masses.iterations,'chassis')
    iteration_number=1;
    T=table(iteration_number,chassis.wheel_brakes_weight,chassis.additional_brake_components_weight,chassis.rims_weight,chassis.tires_weight,chassis.rear_axle_weight,chassis.front_axle_weight,chassis.steering_weight,chassis.air_susp_weight);
    T.Properties.VariableNames(2:end)=el_chassis;    
    v.masses.iterations.chassis=T;
else
    iteration_number=size(v.masses.iterations.chassis,1)+1;
    T=table(iteration_number,chassis.wheel_brakes_weight,chassis.additional_brake_components_weight,chassis.rims_weight,chassis.tires_weight,chassis.rear_axle_weight,chassis.front_axle_weight,chassis.steering_weight,chassis.air_susp_weight);
    T.Properties.VariableNames(2:end)=el_chassis;
    v.masses.iterations.chassis=[v.masses.iterations.chassis;T];
end

%% 3) Sum elements of powertrain
%This module is affected by secondary mass changes 
powertrain=v.masses.powertrain;
el_powertrain=fieldnames(powertrain);

mass_powertrain=0;
for i=1:numel(el_powertrain)
    mass_powertrain=mass_powertrain+powertrain.(el_powertrain{i});
end

%Table powertrain: Create a row for every iteration, displaying the weight of each powertrain component
battery_mass = powertrain.battery_structural_components+powertrain.battery_cells+powertrain.battery_electrical;
GM_mass = mass_powertrain-battery_mass-powertrain.cooling_weight;
cooling_mass=powertrain.cooling_weight;

if ~isfield(v.masses.iterations,'powertrain')
    iteration_number=1;
    T=table(iteration_number,battery_mass,cooling_mass,GM_mass);
    v.masses.iterations.powertrain=T;
else
    iteration_number=size(v.masses.iterations.powertrain,1)+1;
    T=table(iteration_number,battery_mass,cooling_mass,GM_mass);
    v.masses.iterations.powertrain=[v.masses.iterations.powertrain;T];
end

%% 4) Sum elements of interior
%This module is NOT affected by secondary mass changes -> No need to document it in the iteration table
interior=v.masses.interior;
el_interior=fieldnames(interior);

mass_interior=0;
for i=1:numel(el_interior)
    mass_interior=mass_interior+interior.(el_interior{i});   
end

%% 5) Sum elements of exterior
%This module is NOT affected by secondary mass changes -> No need to document it in the iteration table
exterior=v.masses.exterior;
el_exterior=fieldnames(exterior);

mass_exterior=0;
for i=1:numel(el_exterior)
    mass_exterior=mass_exterior+exterior.(el_exterior{i});   
end


%% 6) Sum elements of frame
%This module is affected by secondary mass changes 
frame=v.masses.frame;
el_frame=fieldnames(frame);

mass_frame=0;
for i=1:numel(el_frame)
    mass_frame=mass_frame+frame.(el_frame{i});   
end

%Table frame: create a row for every iteration, displaying the weight of each frame component
if ~isfield(v.masses.iterations,'frame')
    iteration_number=1;
    T=table(iteration_number,frame.BIW_weight,frame.other_frame_components_weight);
    T.Properties.VariableNames(2:end)=el_frame;
    v.masses.iterations.frame=T;
else
    iteration_number=size(v.masses.iterations.frame,1)+1;
    T=table(iteration_number,frame.BIW_weight,frame.other_frame_components_weight);
    T.Properties.VariableNames(2:end)=el_frame;
    v.masses.iterations.frame=[v.masses.iterations.frame;T];
end
%% 7) Sum elements of EE
%This module is affected by secondary mass changes 
EE=v.masses.EE;
el_EE=fieldnames(EE);

mass_EE=0;
for i=1:numel(el_EE)
    mass_EE=mass_EE+EE.(el_EE{i});   
end

%Table EE: Create a row for every iteration, displaying the weight of each EE component
if ~isfield(v.masses.iterations,'EE')
    iteration_number=1;
    T=table(iteration_number,EE.LV_weight,EE.HV_weight,EE.plugin_sys_weight);
    T.Properties.VariableNames(2:end)=el_EE;
    v.masses.iterations.EE=T;
else
    iteration_number=size(v.masses.iterations.EE,1)+1;
    T=table(iteration_number,EE.LV_weight,EE.HV_weight,EE.plugin_sys_weight);
    T.Properties.VariableNames(2:end)=el_EE;
    v.masses.iterations.EE=[v.masses.iterations.EE;T];
end

%Read mass of ECU and Sensors (See Auxiliary_SensECU)
mass_SensECU=v.masses.SensECU;


%% 8) Mass of accessories
%This module is NOT affected by secondary mass changes -> No need to document it in the iteration table
mass_accessories=v.masses.accessories;

%% 9) Create output table and update weight
if ~isfield(v.masses.iterations,'vehicle')
    iteration_number=1;
    T=table(iteration_number,mass_chassis,mass_powertrain,mass_interior,mass_exterior,mass_frame,mass_EE,mass_accessories);
    v.masses.iterations.vehicle=T;
else
    iteration_number=size(v.masses.iterations.vehicle,1)+1;
    T=table(iteration_number,mass_chassis,mass_powertrain,mass_interior,mass_exterior,mass_frame,mass_EE,mass_accessories);   
    v.masses.iterations.vehicle=[v.masses.iterations.vehicle;T];
end

%Calculate empty weight EU (with driver) and the max weight in kg
v.masses.vehicle_empty_weight=mass_chassis+mass_powertrain+mass_interior+mass_exterior+mass_frame+mass_EE+mass_accessories+weight_extra_equipment+mass_SensECU;           
v.masses.vehicle_empty_weight_EU=v.masses.vehicle_empty_weight +weight_passenger + weight_suitcase;  

%Add payload = gross weight - curb weight EU. It includes passengers (driver already in curb weight EU) and cargo
v.masses.vehicle_max_weight=v.masses.vehicle_empty_weight_EU+payload;

%Check if the given weight does not surpass the 3.5t limit for passenger cars
if v.masses.vehicle_max_weight>3500
    
    %If the vehicle reaches the maximum weight for licensing, the weight has to
    %be reduced. This is achieved by reducing payload and extra equipement weight.
    v = errorlog( v, 'The actual vehicle weights above 3500 kg, extra equipment and payload will be reduced' );

    %Weight to reduce in kg
    to_reduce=(v.masses.vehicle_max_weight-3500);
    
    %Reduce extra equipment weight:
    weight_extra_equipment_new=max(0,weight_extra_equipment-to_reduce);
    to_reduce=to_reduce-(weight_extra_equipment-weight_extra_equipment_new);

    %Reassign new extra equipement weight
    v.masses.payload.extra_equipement=weight_extra_equipment_new;  
    
    %If it is not sufficient, reduce the payload
    if to_reduce>0
        
        payload_new=max(0,payload-to_reduce);
        to_reduce=to_reduce-(payload-payload_new);
        
        %Reassign new payload
        v.masses.payload.payload=payload_new;
            
        %if it is not sufficient, this vehicle cannot be licenced with the actual weight:
        if to_reduce>0           
            v = errorlog( v, 'The actual vehicle weights above 3500 kg and is therefore not suitable for licencing as a passenger car' );            
        end
    end
    
    %Recalculate maximum weight
    v.masses.vehicle_max_weight=v.masses.vehicle_empty_weight_EU+payload_new+weight_extra_equipment_new;
end

end

