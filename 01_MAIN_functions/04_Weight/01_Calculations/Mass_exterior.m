function v=Mass_exterior(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the total weight of the exterior. The exterior is
%               defined as the sum of Closures, Bumpers, Lights, Windshield and windows.               
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
%2) Weight of Closures
%3) Weight of Bumpers
%4) Weight of Fenders
%5) Weight of Lights
%6) Weight of Windshield and wipers

%% 1) Initialize the variables:
wheelbase = v.dimensions.GX.wheelbase;                              % in mm
width = v.dimensions.GY.vehicle_width;                              % in mm
height = v.dimensions.GZ.vehicle_height;                            % in mm
vehicle_form = v.topology.frameform;                                % SUV, Hatchback or Sedan    
overhang_r=v.dimensions.GX.vehicle_overhang_r;                      % in mm

%Further Parameters
hood_material = v.masses.optional_extras.hood_material;             %steel/alu
doors_material = v.masses.optional_extras.doors_material;           %steel/alu
tailgate_material = v.masses.optional_extras.tailgate_material;     %steel/alu
fenders_material=v.masses.optional_extras.fenders_material;         %steel/alu
headlights_tech = v.masses.optional_extras.headlights_tech;         %LED/Xenon and Halogen
doors_number = v.masses.optional_extras.doors_number;               %4/2 
panoramic_roof = v.masses.optional_extras.panorama_roof;            %1=yes; 0=no
sliding_roof =v.masses.optional_extras.sliding_roof;                %1=yes; 0=no

%Regressions for the mass calculation of exterior components
regr_hood_steel=Par.regr.mass.hood.steel.eq;
regr_hood_aluminum=Par.regr.mass.hood.alu.eq;
regr_front_door_steel=Par.regr.mass.door_driver.steel.eq;
regr_front_door_aluminum=Par.regr.mass.door_driver.alu.eq;
regr_rear_door_steel=Par.regr.mass.door_rear.steel.eq;
regr_rear_door_aluminum=Par.regr.mass.door_rear.alu.eq;
regr_door_2door=Par.regr.mass.door_2door.eq;
regr_tailgate_steel=Par.regr.mass.tailgate.steel.eq;
regr_tailgate_aluminum=Par.regr.mass.tailgate.alu.eq;
regr_front_bumper=Par.regr.mass.front_bumper.eq;
regr_rear_bumper=Par.regr.mass.rear_bumper.eq;
regr_headlights_LED=Par.regr.mass.headlights.LED.eq;
regr_headlights_xenon=Par.regr.mass.headlights.xenon.eq;
regr_headlights_halogen=Par.regr.mass.headlights.halogen.eq;
regr_taillights=Par.regr.mass.taillights.eq;
regr_rear_quarter_glasses=Par.regr.mass.quarter_glasses_4door.eq;


%% 2) Closures
%In this section, the mass of hood, doors, and tailgate is calculated.

% Hood. We distinguish between steel and aluminum
if strcmp(hood_material,'steel')
    hood_weight = regr_hood_steel(width);%-80.956+0.055*width;         % [kg]
else %aluminum
    hood_weight = regr_hood_aluminum(width);%-72.5413+0.047*width;     % [kg]
end

%Check that the result is not negative
check_mass(hood_weight);

% Doors (2 or 4); Doors weight includes inner panels and windows.
if doors_number == 4 % 4 door vehicles. 
    if strcmp(doors_material,'steel')
        front_doors_weight = 2*regr_front_door_steel(height,wheelbase);%2*(-23.759+0.015*wheelbase+0.011*height);             % [kg]
        rear_doors_weight = 2*regr_rear_door_steel(height,wheelbase);%2*(-46.265+0.018*wheelbase+0.016*height);              % [kg]
    else  %aluminum
        front_doors_weight = 2*regr_front_door_aluminum(height,wheelbase);%2*(-32.204+0.012*wheelbase+0.020*height);             % [kg] 
        rear_doors_weight = 2*regr_rear_door_aluminum(height,wheelbase);%2*(-84.819+0.027*wheelbase+0.022*height);              % [kg]
    end
else % 2 doors, no material distinction due to little available data
   front_doors_weight = 2*regr_door_2door(height,wheelbase);%2*(-65.208+0.021*wheelbase+0.035*height);                  % [kg]
   rear_doors_weight = 0;
end

%Check that the result is not negative
check_mass(front_doors_weight);
check_mass(rear_doors_weight);

% Tailgate (SUVs, Hatchbacks) or Trunk (Sedans). Tailgate includes rear window
if strcmp(vehicle_form,'Sedan')  %Trunk: constant values.
    if strcmp(tailgate_material,'steel')       %steel
        tailgate_weight = Par.masses.trunk_steel; %20.691;          % [kg]
    else                                       %aluminum
        tailgate_weight = Par.masses.trunk_alu; %18.661;            % [kg]
    end              
else  % SUV, SW, hatchback -> Tailgate
    if strcmp(tailgate_material,'steel')       %steel
        tailgate_weight = regr_tailgate_steel(width,height);%-102.182+0.058*width+0.016*height;        % [kg]
    else                                       %aluminum
        tailgate_weight = regr_tailgate_aluminum(width,height);%-85.181+0.053*width+0.010*height;      % [kg]
    end
    
end

%Check that the result is not negative
check_mass(tailgate_weight);

%Add hood, doors and tailgate
closures_weight = hood_weight + front_doors_weight + ...
    rear_doors_weight + tailgate_weight;

%% 3) Bumpers

% Front bumper
bumper_front_weight = regr_front_bumper(width);%-40.154 + 0.032*width;     % [kg]

% Rear bumper
bumper_rear_weight = regr_rear_bumper(width);%-40.470 + 0.030*width;       % [kg]

%Check that the result is not negative
check_mass(bumper_front_weight);
check_mass(bumper_rear_weight);

%Sum of front and rear bumper
bumpers_weight = bumper_front_weight + bumper_rear_weight;

%% 4) Fenders
% Mostly this weight accounts only for the front fenders, since the rear fenders are usually integratred in the BIW
if strcmp(fenders_material,'steel')
    fenders_weight = Par.masses.fenders.steel;%5.387;                % [kg]
else  % alu 
    fenders_weight = Par.masses.fenders.alu;%-14.259-0.013*overhang_f+0.016*width;     % [kg]
end

%% 5) Lights
% Headlights: 3 possibilities: LED, Xenon and Halogen
switch headlights_tech

    case 'LED'
        
        headlights_weight = regr_headlights_LED(width);%-25.237 + 0.018*width;                  % [kg]        
    case 'Xenon'
        
        headlights_weight = regr_headlights_xenon(width,height);%-18.231+0.008*width+0.007*height;       % [kg]
    otherwise %Halogen
        
        headlights_weight = regr_headlights_halogen(width);%-19.115+0.014*width;                    % [kg]      
end

%Taillights
taillights_weight = regr_taillights(width,height);%-11.958+0.006*width+0.003*height;               % [kg]

% 3rd stop light
stop_light_weight =Par.masses.stop_light;%0.1915;  % [kg]

% Foglights 
foglights_weight = Par.masses.fog_lights;%1.2625;  % [kg]

%Check that the result is not negative
check_mass(headlights_weight);
check_mass(taillights_weight);

%Sum al the lighting components
lights_weight = headlights_weight+taillights_weight+stop_light_weight +...
    foglights_weight;

%% 6) Windshield and wipers

% Front windshield wiper (only the wiper blades!) in kg
front_wipers_weight = Par.masses.wiper_front;

%Weight of the fluids for the wipers and the washing system (tank, hoses, nozzles)
wipers_fluid_weight=Par.masses.washer_fluid+Par.masses.washer_system; %4.95;                    % [kg]
% Front windshield % mean value based on wheelbase intervals (segment) --> see Eslam method
if wheelbase <= 2493.12                            % A Segment
    windshield_weight = Par.masses.windshield_A;%11.979;                                       % [kg]               
elseif 2493.12 < wheelbase && wheelbase <= 2640.04 % B Segment
    windshield_weight = Par.masses.windshield_B;%12.293;                                       % [kg]
elseif 2640.04 < wheelbase && wheelbase <= 2750.32 % C Segment
    windshield_weight = Par.masses.windshield_C;%13.5655;                                      % [kg]
elseif 2750.32 < wheelbase && wheelbase <= 2927.14 % D Segment
    windshield_weight = Par.masses.windshield_D;%12.714;                                       % [kg]
else                                               % E Segment
    windshield_weight = Par.masses.windshield_E_F;%13.566;                                     % [kg]
end

% Rear glass windshield and rear windshield wiper -> Sedans don't have the rear wiper
if strcmp(vehicle_form,'Sedan')  % rear glass not included in the trunk lid
    rear_wipers_weight = 0;                                          % [kg] 
    rear_window_weight = Par.masses.rear_window_sedans;%8.0825;      % [kg]
else % weight of rear glass already included in the tailgate. Wiper present.
    rear_wipers_weight = Par.masses.wiper_rear; %0.974;              % [kg]
    rear_window_weight = 0;                                          % [kg]
end

%Quarter glass: Glass between rear door and luggage compartement
if doors_number < 4 % 2 door vehicles
    rear_quarter_glasses_weight = Par.masses.quarter_glasses_2door; %3.93;           % [kg]
else 
    if strcmp(vehicle_form,'Sedan')
        rear_quarter_glasses_weight = Par.masses.quarter_glasses_sedans;%1.9072;     % [kg]
    else 
        rear_quarter_glasses_weight = regr_rear_quarter_glasses(overhang_r);%-3.845+0.007*overhang_r;    % [kg]CHECK
    end                                  
end

%Panoramic roof (if present) -> The panoramic roof is fixed and cannot be opened
if panoramic_roof == 1
    panoramic_roof_weight = Par.masses.fixed_glass_roof;         % [kg]
else
    panoramic_roof_weight = 0;                                   % [kg]
end

%Sliding roof -> differently from panoramic roof it can be opened
if sliding_roof==1
    sliding_roof_weight=Par.masses.sliding_glass_roof;           % [kg]
else
    sliding_roof_weight=0;                                       % [kg]
end

%Check that the result is not negative
check_mass(rear_quarter_glasses_weight);

%sum all components
windshield_windows_weight = windshield_weight + front_wipers_weight +...
    rear_wipers_weight +rear_window_weight+rear_quarter_glasses_weight + ...
    panoramic_roof_weight+sliding_roof_weight+wipers_fluid_weight;

%% Assign Output

v.masses.exterior.closures_weight=closures_weight;
v.masses.exterior.bumpers_weight=bumpers_weight;
v.masses.exterior.fenders_weight=fenders_weight;
v.masses.exterior.lights_weight=lights_weight;
v.masses.exterior.windshield_windows_weight=windshield_windows_weight;
end