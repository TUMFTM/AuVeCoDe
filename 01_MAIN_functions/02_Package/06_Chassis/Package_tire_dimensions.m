function [v] = Package_tire_dimensions(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the tire dimensions. In a first step, from the
%               given Inputs, the maximum tire load is calculated. After that a required brake diameter is calculated.
%               Adding a mean offset the rim diameter is derived. Finally from the max tire load the
%               tire volume is calculated. From the tire volume the width and the diameter
%               of the tire are derived, thus fully describing the wheel dimensions
% ------------
% Sources:  [1] G.Leister, Passenger Car Tires and Wheels. Springer International Publishing 2018
%           [2] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [3] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters (Par): struct with input and constant values
%           - vehicle(v): struct with data of the vehicle
% ------------
% Output:   - vehicle(v): struct with data of the vehicle
% ------------


%% Implementation
%1) Local input declaration
%2) Calculate load case for the 100% rule
%3) Calculate load case for the 88% rule
%4) Calculate brake and rim diameter
%5) Calculate tire dimensions
%6) Assign Outputs
%7) Subfunction calc tire width

%% 1) Local Input declaration
front_repartition=Par.masses.loads.axle_load_front.(v.topology.drive);    %Repartition in % on the front axle
rear_repartition = 100-front_repartition;                                 %Repartition in % on the rear axle
wb=v.dimensions.GX.wheelbase;                                             %Wheelbase in mm
o_r=v.dimensions.GX.vehicle_overhang_r;                                   %Rear overhang in mm
width=v.dimensions.GY.vehicle_width;                                      %Vehicle width in mm

%Masses in kg:
m_empty=v.masses.vehicle_empty_weight;              %empty vehicle weight without payload, extra equipement and without driver in kg   
m_max=v.masses.vehicle_max_weight;                  %max vehicle weight in kg with payload, extra equipement and passengers in kg
m_pass=Par.masses.payload.weight_passenger;         %standard weight passenger/driver in kg
m_lugg=Par.masses.payload.weight_luggage;           %standard weight luggage in kg
m_ex_eq=v.masses.payload.extra_equipement;          %weight extra equipement in kg
m_payload=v.masses.payload.vehicle_payload;         %weight of the vehicle payload in kg. Payload = m_max-m_empty = n_pass*weight_passengers+luggage.

X_sr1=v.manikin.SgRP_X_front_axle;                  %distance in mm between front axle and SgRP of first row of seats (L114)
X_sr2=v.manikin.SgRP2_X_front_axle;                 %distance in mm between front axle and SgRP of first row of seats (L114+L50_2)
X_lg=X_sr2+o_r / 2;                                 %distance in mm between front axle and centre of luggage compartment

%Number of passenger 
n_pass=sum(Par.input.n_passengers);
n_pass_f=v.Input.n_seat(1);
n_pass_r=v.Input.n_seat(2);

%Other parameters for the tire and brake disc calculation:
acceleration=v.Input.acceleration_time_req;                     %Required acceleration time 0-100 in s
rim_diameter=v.dimensions.CX.rim_diameter_r_inch;               %Desired/required rim diameter in inches
tire_diameter_should=2*v.Input.r_tire;                          %Desired/required tire diameter in mm
reserve_100=Par.masses.reserve.tyre_reserve_100;                %Extra tolerance for the 100% rule in kg
reserve_88=Par.masses.reserve.tyre_reserve_88;                  %Extra tolerance for the 88% rule in kg

%Regressions, constant values, and catalogs:
regr_normal_load=Par.regr.wheel.tire_volume_normal_load.eq; %Load vs required volume for normal load
regr_extra_load=Par.regr.wheel.tire_volume_extra_load.eq;   %Load vs required volume for extra load
regr_brake_disc=Par.regr.wheel.brake_disc_f_diameter.eq;    %Brake disc diameter vs acceleration time and gross mass
Tires=Par.wheels.tires;                                     %Tire catalogue 
EX_min_offset_front=Par.dimensions.EX.offset_brake_to_rim;  %Minimum clearance between rim and brake disc expressed as difference between rim and brake diameter (in mm)

%% 2) Calculate load case for the 100% rule
%Distance between COG and front axle (case empty vehicle with extra equipement) in mm
X_COG_basis = wb*rear_repartition/100;

%Calculate mass according to the 100% rule definition (in kg)
m_100_rule = m_empty+m_ex_eq+m_payload;

%Calculate the new X_COG with the 100% Rule. We suppose that the extra equipment is distributed front and rear axle
%With the same values as the front and rear repartition.
X_COG_100_rule = (X_COG_basis*(m_empty+m_ex_eq) + n_pass_f*m_pass*X_sr1 + n_pass_r*m_pass*X_sr2+(m_payload-n_pass*m_pass)*X_lg)./ m_100_rule;    

%Calculate the resulting loads at front and rear axle (in kg)                    
tire_load_f_100 = (m_100_rule*(wb-X_COG_100_rule)/wb)/2 + reserve_100;                                 
tire_load_r_100 = (m_100_rule*X_COG_100_rule/wb)/2 + reserve_100;

%Take the maximum load as the relevant load (in kg)
tire_load_100 = max(tire_load_f_100, tire_load_r_100);

%% 3) Calculate load case for the 88% rule

%Calculate mass according to the 88% rule definition (in kg).
if n_pass <=4
    m_88_rule = m_empty+2*m_pass+m_ex_eq+2*m_lugg;
    X_COG_88_rule = (X_COG_basis*(m_empty+m_ex_eq) + 2*m_pass*X_sr1+2*m_lugg*X_lg)/ m_88_rule; 
else
    m_88_rule = m_empty+3*m_pass+m_ex_eq+3*m_lugg;  
    X_COG_88_rule = (X_COG_basis*(m_empty+m_ex_eq)+2*m_pass*X_sr1+ m_pass*X_sr2+3*m_lugg*X_lg)./m_88_rule;    
end       

%Tire load front and rear according to the 88% rule
tire_load_f_88 = (m_88_rule*(wb-X_COG_88_rule)/wb)/(2*0.88) + reserve_88;
tire_load_r_88 = (m_88_rule*X_COG_88_rule/wb)/(2*0.88) + reserve_88;

%Take the maximum load as the relevant load (in kg)
tire_load_88 = max(tire_load_f_88, tire_load_r_88); %in kg

%Take the maximum load as the relevant load (in kg)
tire_load_max = max(tire_load_88, tire_load_100);

%% 4) Calculate brake and rim diameter min
% Regression brake disc diameter front axle in mm
CX_brake_disc_f_diameter=regr_brake_disc(m_max,acceleration);

%calculate rim dimension without collision with brake system
rim_diameter_min=round((EX_min_offset_front+CX_brake_disc_f_diameter)/25.4);

if rim_diameter_min>rim_diameter
    
    v = errorlog(v,'The given rim diameter is not sufficient to fit the brake mechanism, therefore it will be increased');
    rim_diameter=rim_diameter_min;
    rim_rad_mm=rim_diameter*25.4/2;
    if rim_rad_mm > v.Input.r_tire
            v.Error=5;
            return
    else
        v.dimensions.CX.rim_diameter_f_inch = rim_diameter;
        v.dimensions.CX.rim_diameter_r_inch = rim_diameter;
    end    
end

%% 5) Calculate tire volume and minimum width

%Calculate the tire volume from the tire load. Distinguish between normal and extra load tires
if strcmp(v.Input.tire_type,'NL')  %Normal load tire    
    tire_volume = regr_normal_load(tire_load_max);
    
else   % Extra load tire    
    tire_volume = regr_extra_load(tire_load_max);
    
end

[tire_diameter,tire_volume,tire_width,side_wall] =calc_tire_width(Tires,rim_diameter,tire_volume,tire_diameter_should,v.Input.tire_type);
   
%% 6) Assign Outputs
%Assign the calucated dimensions as output. The front and rear tire are coinsidered as equal size

v.dimensions.CX.rim_f_diameter=rim_diameter;
v.dimensions.CX.brake_disc_f_diameter=CX_brake_disc_f_diameter;
v.dimensions.CX.brake_disc_r_diameter=CX_brake_disc_f_diameter;
v.dimensions.CX.wheel_f_diameter=tire_diameter;
v.dimensions.CY.wheel_f_width=tire_width;
v.dimensions.CX.rim_r_diameter=rim_diameter;
v.dimensions.CX.wheel_r_diameter=tire_diameter;
v.dimensions.CY.wheel_r_width=tire_width;
v.dimensions.GY.track_width_r=width-tire_width;
v.dimensions.GY.track_width_f=width-tire_width;

%Tire and axle loads:
v.wheels.tire_load_88_f=tire_load_f_88;
v.wheels.tire_load_88_r=tire_load_r_88;
v.wheels.tire_load_100_f=tire_load_f_100;
v.wheels.tire_load_100_r=tire_load_r_100;

%Axle loads
v.masses.axle_load_r_max=(rear_repartition)*0.01*m_max;               %in kg
v.masses.axle_load_f_max=(front_repartition)*0.01*m_max;              %in kg
v.masses.axle_load_r_empty=(rear_repartition)*0.01*m_empty;           %in kg
v.masses.axle_load_f_empty=(front_repartition)*0.01*m_empty;          %in kg


v.wheels.tire_side_wall=side_wall;
v.wheels.tire_volume=tire_volume;

end

%% 7) Subfunction: calc tire width

function [diameter_final,volume_final,width,sidewall] =calc_tire_width(Tires,rim_diameter_in_inches,volume_min,tire_diameter_in_mm,tire_type)
%% Description:
%This function calculates from the calculated volume, rim and wheel
%diameter the wheel dimensions. The functions generates staring from the
%rim diameter all the possible combinations (from all possible sidewalls
%and wheel width) and identifies the configuration which is the closest to
%the calculated volume and the searched rim diameter.
%Author: Adrian König, Lorenzo Nicoletti
%Date: 01.05.2022
%% Input:
%Tires: Catalog containing the set of possible tires
%rim_diameter_in_inches: Required rim diameter. This may be a user input or otherwise calculated from the brake disc dimensions (in inches)
%volume_min: Minimum required tire volume to carry the tire load in mm^3
%tire_diameter_in_mm: Desired tire diamettire_diameter_in_mmer in mm. input of the tool.
%tire_type: The type of tire used (EL or NL)

%% Output
%The final wheel dimensions

%% Implementation
%1) Calculate all the possible wheel dimensions from all possible sidewall factor and the width (for the given rim size)
%2) Identify the wheel dimension with the smallest diameter and volume deviation

%Filter out the tires which are not compatible with the given rim diameter
Tires_filt=Tires(Tires.rim_diameter_in_inches==rim_diameter_in_inches,:);

%Keep only the tires compatible with the chosen tire type (EL or NL)
if strcmp(tire_type,'EL')
    Tires_filt=Tires_filt(contains(Tires_filt.extra_load_tire,'Yes'),:);  
else
    Tires_filt=Tires_filt(contains(Tires_filt.normal_load_tire,'Yes'),:);
end

%For each of the remaining tire, calculate the volume and its percentual deviation from the required volume
volume_dev=((Tires_filt.tire_volume_calculated_in_mm3-volume_min)./volume_min)*100;

%For each of the remaining tire, calculate the percentual deviation from the required outer diameter
diameter_dev=(abs(Tires_filt.calculated_diameter_in_mm-tire_diameter_in_mm)./tire_diameter_in_mm)*100;

%For the first loop, accept a deviation of 1% from the Input diameter
deviation=1; %in %
check=0;

while check==0
      
    %Find the corresponding volumes and consider only the ones bigger than
    %the Input volume. I.e. the volume which have a positive deviation form
    %the Input volume
    corresponding_volumes=volume_dev(diameter_dev<deviation); %only use volumes where diameter deviation is smaller then 1%
    corresponding_volumes=corresponding_volumes(corresponding_volumes>=0);
    
    if isempty(corresponding_volumes) %None of the corresponing volumes is bigger than the input volume
        
        %Make the acceptable diameter deviation higher and start a new loop
        deviation=deviation+0.5; % in %
    else
        
        %Find the volume with the smallest deviation and retrieve its position in the volume matrix
        best_volume=min(corresponding_volumes);
        position=find(volume_dev==best_volume);
        check=1; %Exit the loop
    end
    
    if deviation>50
        error('The closest tire in the catalog having the desired tire and rim diameters has a volume which is 50% higher than the required one! Try to select more realistic values for tire and rim diameter')
    end         
end

%If two tires are found, select one (size is same)
position=position(1);

%Assign outputs
diameter_final=Tires_filt.calculated_diameter_in_mm(position);
volume_final=Tires_filt.tire_volume_calculated_in_mm3(position);
width=Tires_filt.width_in_mm(position);
sidewall=Tires_filt.aspect_ratio(position);
end

