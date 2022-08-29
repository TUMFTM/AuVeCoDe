function v=Mass_chassis(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the total weight of the chassis. The chassis is
%               defined as the sum of brakes, brake calipers and pads, rims, tires, shock
%               absorber, coil springs and axles weight.
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
%1) Initialize the variables:
%2) Weight of brake system
%3) Weight of rims and tires
%4) Front Axle weight complete
%5) Rear Axle weight complete
%6) Additional components for air suspensions (if any)
%7) Steering system weight

%% 1) Initialize the variables:
rim_diameter_in_inches=v.dimensions.CX.rim_diameter_f_inch;         % in inches
tire_diameter=v.dimensions.CX.wheel_f_diameter;                     % in mm
tire_width=v.dimensions.CY.wheel_f_width;                           % in mm
wheelbase = v.dimensions.GX.wheelbase;                              % in mm
width = v.dimensions.GY.vehicle_width;                              % in mm
gross_weight = v.masses.vehicle_max_weight;                         % in kg
gross_load_front_axle = v.masses.axle_load_f_max;                   % in kg
gross_load_rear_axle = gross_weight-gross_load_front_axle;          % in kg

%Optional extras of the vehicle:
air_susp=v.masses.optional_extras.AS;                               % 1: air suspension; 0: no air suspensions
rear_steering=v.masses.optional_extras.AWS;                         % 1: rear steering; 0: no rear steering

%Regression and constant values for the mass calculation of chassis components
regr_rim=Par.regr.mass.rim.eq;
regr_tire=Par.regr.mass.tire.eq;
regr_brakes_front=Par.regr.mass.front_wheel_brakes.eq;
regr_brakes_rear=Par.regr.mass.rear_wheel_brakes.eq;
regr_axles_front_links=Par.regr.mass.front_axle_links.eq;
regr_axles_front_SD=Par.regr.mass.front_axle_SD.eq;
regr_axles_rear_ML_links=Par.regr.mass.rear_axle_links_ML.eq;
regr_axles_rear_SD=Par.regr.mass.rear_axle_SD.eq;
regr_axles_rear_TB=Par.regr.mass.rear_axle_TB.eq;
regr_steering_system=Par.regr.mass.steering.eq;

%% 2) Weight of brake system
%Calculate the mass of the front wheel brakes(brake discs, calipers and
%pads)from the vehicle's gross weight
brakes_front_weight = regr_brakes_front(gross_weight);%-4.857+0.01543*gross_weight;

%Calculate the mass of the rear wheel brakes(brake discs, calipers and pads)from the mass of the front wheel brakes
brakes_rear_weight = regr_brakes_rear(brakes_front_weight);%2.05494+0.58549*brakes_front_weight;

%Total mass of brake components in the wheel (contribute to unsprung mass)
wheel_brakes_weight = brakes_front_weight+brakes_rear_weight;

%ABS system as constant value
ABS_system_weight = Par.masses.ABS_system;  %2.701                   % [kg]

%Brake_lines as constant value
brake_lines_weight = Par.masses.brake_lines_system;   %1.395         % [kg]

%Master cylinder as constant value
master_cylinder_weight = Par.masses.master_cylinder; %5.148;         % [kg]

%Parking brake actuators as constant value
parking_actuators_weight = Par.masses.parking_brake_actuators; %1.126;   % [kg]

%Brake fluid as constant value
brake_fluid_weight = Par.masses.brake_fluid; %0.59;                  % [kg]

%Brake hoses as constant value (for all four brakes)
brake_hoses_weight = 2*Par.masses.brake_hose; %2*0.30;               % [kg]

%Brake disc covers as constant value (for all four brakes)
disc_covers_weight = 2*Par.masses.brake_disc_cover; %2*0.38;         % [kg]

%Check that the result is not negative
check_mass(brakes_front_weight);
check_mass(brakes_rear_weight);

%Sum the additional brake components
additional_brake_components = ABS_system_weight+...
    brake_lines_weight+master_cylinder_weight+parking_actuators_weight+...
    brake_fluid_weight+brake_hoses_weight+disc_covers_weight;        % [kg]

%% 3) Weight of rims and tires

%Weight of the four rims in kg
rims_weight=regr_rim(rim_diameter_in_inches)*4;

%Weight of the four tires in kg
tires_weight=regr_tire(tire_width,tire_diameter)*4;

%Check that the result is not negative
check_mass(rims_weight);
check_mass(tires_weight);

%% 4) Front Axle weight complete
%The front axle is computed separately from its spring-damper assembly.
%This approach allows distinguishing between vehicles with and without air
%suspension. The spring-damper assembly is calculated differently. 
%Spring-damper assembly NO AS: steel spring and damper. 
%Spring-damper assembly AS: air spring and damper. 
%The weight of the axle includes suspension arms, anti-roll bar, subframe and wheel hubs.
%No geometry distinction

if air_susp == 0  %no air suspension
    weight_front_axle_arms =  regr_axles_front_links(gross_load_front_axle,width); %-84.715 + 0.02782*gross_load_front_axle + 0.05359*width;   % [kg]
    weight_front_axle_shockabs = regr_axles_front_SD(gross_load_front_axle);           %2.3178+0.00991*gross_load_front_axle;                  % [kg]
else  %with air suspension
    weight_front_axle_arms =  regr_axles_front_links(gross_load_front_axle,width);%-84.715 + 0.02782*gross_load_front_axle + 0.05359*width;    % [kg]
    weight_front_axle_shockabs = Par.masses.front_AS; %16.68;                                                                                  %[kg]
end

%Check that the result is not negative
check_mass(weight_front_axle_arms);
check_mass(weight_front_axle_shockabs);

%% 5) Rear Axle weight complete
%We need to distinguish between Torsion beam and multilink axles
%The mass of the torsion beam is calculated along with its spring-damper assembly
%The mass of the axle comprises control arms, subframe, anti-roll bar and wheel hubs.
%Multilink suspensions can have air suspension. As for the front axle, the
%mass of the spring-damper assembly is calculated differently based on this
%distinction.

if strcmp(v.Input.axis_type_r,'torsion_beam')
    weight_rear_axle = regr_axles_rear_TB(gross_weight);%-0.7229+0.021897*gross_weight;    % [kg]
else % Multi link 
    if air_susp == 0 %without air suspensions
        weight_rear_axle = regr_axles_rear_ML_links(gross_weight,width) + regr_axles_rear_SD(gross_load_rear_axle);  %-66.1215+0.00876*gross_weight+0.054*width + (1.63429+0.008688*(gross_weight-gross_load_front_axle));   % [kg]
    else
        weight_rear_axle = regr_axles_rear_ML_links(gross_weight,width) + Par.masses.rear_AS; %-66.1215+0.00876*gross_weight+0.054*width + 13.836;    %[kg]
    end  
end

%Check that the result is not negative
check_mass(weight_rear_axle); 

%% 6) Additional components for air suspensions (if any)
%This section calculates the weight of the on-board components of the
%suspension system. These are air bottles, compressor, distributor.
%Masses include supports and protections

if air_susp == 1
   %add weight of additional components
   air_bottle_weight = Par.masses.AS_air_bottle;                      % [kg]
   compressor_weight = Par.masses.AS_compressor; %4.4825;             % [kg]
   distributor_weight = Par.masses.AS_distributor; %0.443;            % [kg]
  
else
    air_bottle_weight= 0;
    compressor_weight = 0;
    distributor_weight = 0;
   
end

additional_weight_AS = air_bottle_weight+ ...
       compressor_weight+distributor_weight;       % [kg]
   
%% 7) Steering system weight
%The mass of the steering system includes the steering shaft assembly, the
%steering wheel (without airbag), the steering rack assembly and the tie rods.
%If the vehicle is equipped with all-wheel-steering, the weight of the
%rear actuation motor and tie rods is included.

weight_steering = regr_steering_system(gross_weight,wheelbase); % -11.01 + 0.002313*gross_weight + 0.010*wheelbase;   % [kg] 
if rear_steering == 1  %add weight of rear axle steering
    weight_steering = weight_steering + Par.masses.rear_axle_steering; %11.454; % [kg]
end

%Check that the result is not negative
check_mass(weight_steering); 

%% Assign outputs
v.masses.chassis.wheel_brakes_weight = wheel_brakes_weight;
v.masses.chassis.additional_brake_components_weight = additional_brake_components;
v.masses.chassis.rims_weight = rims_weight;
v.masses.chassis.tires_weight = tires_weight;
v.masses.chassis.rear_axle_weight = weight_rear_axle;
v.masses.chassis.front_axle_weight = weight_front_axle_arms + weight_front_axle_shockabs;
v.masses.chassis.steering_weight = weight_steering;
v.masses.chassis.air_susp_weight = additional_weight_AS;

end