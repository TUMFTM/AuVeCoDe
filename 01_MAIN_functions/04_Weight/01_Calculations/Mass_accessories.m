function v=Mass_accessories(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates the weight of the accessories of the vehicle.
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
%1) Initialize the variables
%2) Emergency equipment
%3) Comfort and ADAS
%4) Tow hitch
%5) Assign outputs

%% 1) Initialize the variables
tire_diameter=v.dimensions.CX.wheel_f_diameter;
tire_width=v.dimensions.CY.wheel_f_width;
rim_diameter_in_inches=v.dimensions.CX.rim_diameter_f_inch;

%Optional extras (1= Optional extra is present; 0=Optional extra is not present)
ACC=v.masses.optional_extras.ACC;               %Active Cruise Control
BMS=v.masses.optional_extras.BSM;               %Blind Spot Monitor
PA=v.masses.optional_extras.park_assist;        %Park Assist System
ST=v.masses.optional_extras.spare_tire;         %Spare Tire (if built)
NV=v.masses.optional_extras.night_vision;       %Night vision
LKS=v.masses.optional_extras.LKS;               %Lane Keeping Support Camera
KL=v.masses.optional_extras.keyless;            %Keyless opening
TA=v.masses.optional_extras.trunk_assist;       %Trunk Assist System
PC=v.masses.optional_extras.phone_connectivity; %Phone connectivity
TH=v.masses.optional_extras.tow_hitch;          %Tow hitch

%Regressions for the mass calculation of EE components
regr_tire=Par.regr.mass.tire.eq;
regr_rim=Par.regr.mass.rim.eq;
regr_spare_wheel=Par.regr.mass.spare_wheel.eq;

%% 2) Emergency equipment
%mass of standard vehicle wheel
standard_wheel_weight=regr_tire(tire_width,tire_diameter)+regr_rim(rim_diameter_in_inches);  %[kg]
%spare wheel (if present) and toolbox: warning triangle, first aid, equipment to change/repair tire
if ST == 1
    spare_wheel_weight = regr_spare_wheel(standard_wheel_weight);
    toolbox_weight = Par.masses.toolbox.car_jack + Par.masses.toolbox.container +...
        Par.masses.toolbox.first_aid + Par.masses.toolbox.jack_crank +...
        Par.masses.toolbox.nut_tool + Par.masses.toolbox.screwdriver +...
        Par.masses.toolbox.storage_bag + Par.masses.toolbox.tire_compressor +...
        Par.masses.toolbox.tow_hook + Par.masses.toolbox.wheel_lock;%5.648;   % [kg] kit with car jack to change wheel
else
    spare_wheel_weight = 0;
    toolbox_weight = Par.masses.toolbox.container + Par.masses.toolbox.first_aid +...
        Par.masses.toolbox.nut_tool + Par.masses.toolbox.screwdriver +...
        Par.masses.toolbox.storage_bag + Par.masses.toolbox.tire_compressor +...
        Par.masses.toolbox.tire_sealant + Par.masses.toolbox.tow_hook +...
        Par.masses.toolbox.wheel_lock; %3.950;% [kg] kit with sealant and no jack
end

%% 3) Comfort and ADAS

%pedestrian warning, always present on BEVs. Constant value for speakers and support.
pedestrian_warning = Par.masses.AVAS;  %0.73+0.274; % [kg]

%park assist with cameras f/r (if present)
if PA == 1
    park_assist_weight = Par.masses.park_assist;%0.522; % [kg]
else
    park_assist_weight = 0;
end

%ADAS control unit (if ADAS are present)
if ACC == 1 || BMS == 1 || LKS == 1 || NV ==1
    ADAS_ECU = Par.masses.ADAS_ECU; %1.358;  % [kg]
else 
    ADAS_ECU =0;
end

%Adaptive cruise control (if present): radar+supports
if ACC == 1
    ACC_weight = Par.masses.ACC_radar; %0.331; % [kg]
else
    ACC_weight = 0;
end

%Blind spot monitoring (if present): 2 short range radars 
if BMS == 1
    BSM_weight = Par.masses.BSM; %0.483; % [kg]
else
    BSM_weight = 0;
end

%Lane Keeping Support Camera (if present)
if LKS == 1
    LKS_weight = Par.masses.lane_assist; %0.194;
else
    LKS_weight = 0;
end

%Control Unit for Keyless entry (if present)
if KL == 1
    Keyless_weight = Par.masses.keyless_system; %0.24;
else
    Keyless_weight = 0;
end

%Night vision camera (if present)
if NV == 1
    night_vision_weight = Par.masses.night_vision; %1.059;
else
    night_vision_weight = 0;
end

%Sensors for kick to open tailgate (if present)
if TA == 1
    trunk_assist_weight = Par.masses.trunk_opening_sensor; %0.225+0.148;
else
    trunk_assist_weight = 0;
end

%Wireless charge pad and wifi hotspot (if present)
if PC == 1
    phone_connectivity_weight = Par.masses.phone_wifi+Par.masses.phone_wireless_charge; %0.26+0.377;
else
    phone_connectivity_weight = 0;
end

%% 4) Tow hitch
% Includes hitch and frame reinforcements

if TH == 1
    tow_hitch_weight = Par.masses.tow_system; %20.85;  % [kg]
else
    tow_hitch_weight = 0;
end

%% 5) Assign output
%Total the mass of the accessories. this value corresponds to the mass of the entire module
v.masses.accessories = pedestrian_warning + spare_wheel_weight + park_assist_weight+ toolbox_weight +...
    ACC_weight + BSM_weight + LKS_weight + ADAS_ECU + Keyless_weight + night_vision_weight +...
    trunk_assist_weight + phone_connectivity_weight+tow_hitch_weight;
end