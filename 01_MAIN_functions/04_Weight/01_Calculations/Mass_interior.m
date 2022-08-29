function v=Mass_interior(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates weight changed for automous vehicles of the interior. The interior is
%               defined as the sum of Airbags, center console, door panels, HVAC,
%               instrument panel, noise insulation material, seatbelts, seats, speakers,
%               trim parts
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
%2) Weight of safety equipment (EU spec)
%3) Weight of seats
%4) Weight of HVAC
%5) Weight of center console
%6) Weight of instrument panel
%7) Weight of noise insulation material
%8) Weight of trim parts 
%9) Weight of speakers 

%% 1) Initialize the variables:
wheelbase = v.dimensions.GX.wheelbase;                              % [mm]
width = v.dimensions.GY.vehicle_width;                              % [mm]
veh_length = v.dimensions.GX.vehicle_length;                        % [mm]
seating_capacity = v.Input.n_seat;                                  % [/]
sing_armrest_betw = v.Input.single_ar_between;                      % Single armrest between seats
doub_armrest_betw = v.Input.double_ar;                              % Double armrest between seats

%Count number of rows
if seating_capacity(2)==0
    num_rows=1;
else
    num_rows=2;
end

%Reset num of single seats/bench seats
num_frontseat=0;
num_rearseat=0;

%Seat type per row (capacity 2 or 1=single seats, capacity 3 and more =bench)
for i=1:2
    if (seating_capacity(i)<=2 && seating_capacity(i)>0) || sing_armrest_betw(i)==1 || doub_armrest_betw(i)==1
        seat_typ(i)=1;  %1= single seat
        num_frontseat=num_frontseat+seating_capacity(i);
    elseif seating_capacity(i)>2 && seating_capacity(i)>0
        seat_typ(i)=2;  %2= bench
        num_rearseat=num_rearseat+seating_capacity(i);
    else
        seat_typ(i)=0;  %0= no seats
    end
end

%Options
cluster_type = v.masses.optional_extras.cluster_type;
infotainment = v.masses.optional_extras.infotainment;
HUD = v.masses.optional_extras.HUD;
subwoofer = v.masses.optional_extras.subwoofer;

%Regressions for the mass calculation of interior components
regr_front_seat=Par.regr.mass.front_seat.eq;
mass_rear_bench=Par.masses.rear_seat_bench; %constant value instead of regression
regr_heating_system_cabin=Par.regr.mass.heating_system_cabin.eq;
regr_center_console=Par.regr.mass.center_console.eq;
regr_trim_parts = Par.regr.mass.trims.eq;

%% 2) Weight of safety equipment (EU spec)
% Weight of Airbags
% The weight of the airbags is divided into: driver's airbag, passenger's airbag,
% knee airbag, curtain airbag and rear side airbag.

%accelerometers that trigger airbags
airbag_sensors_weight = Par.masses.airbag_sensors;%0.128;                  % [kg]

% Driver's airbag
airbag_driver_weight = Par.masses.airbag_driver;%1.10;                     % [kg]

% Passenger's airbag
airbag_passenger_weight = Par.masses.airbag_passenger;%1.668;              % [kg]

% Knee airbag
airbag_knee_weight = Par.masses.airbag_knee;%1.33;                         % [kg]

% Curtain airbag
airbag_curtain_weight = Par.masses.airbag_curtain;%2.412;                  % [kg]

% Rear side airbags
if num_rows == 2
    airbag_rear_side_weight = Par.masses.airbag_rear_side;%0.588;          % [kg]
else 
    airbag_rear_side_weight = 0;
end

% Airbag control unit
airbag_CU_weight = Par.masses.airbag_CU;%0.343;                            % [kg]

% total airbag_weight
airbag_weight = airbag_sensors_weight+airbag_driver_weight+airbag_passenger_weight+...
    2*airbag_knee_weight+airbag_curtain_weight+airbag_rear_side_weight+...
    airbag_CU_weight;

%Weight of seatbelts 
seatbelt_front_weight = num_frontseat*Par.masses.seatbelt_driver;%2.239*2;             % [kg]

seatbelt_rear_weight = num_rearseat*Par.masses.seatbelt_rear;      % [kg]

seatbelts_weight = seatbelt_front_weight + seatbelt_rear_weight;           % [kg]

%Horn system
horn_weight = Par.masses.horn_system;%0.54;                                % [kg]

%Add up all safety equipment
safety_equipment_weight=airbag_weight+seatbelts_weight+horn_weight;

%% 3) Weight of seats
% Driver and passenger seats
front_seats_weight = num_frontseat*regr_front_seat(width,wheelbase);%2*(-60.6 + 0.045 * width);           % [kg]

%Calculate rear rows weight
rear_bench_weight = mass_rear_bench;%-27.9kg average value

%Rear seats
rear_seats_weight=0;
for i=1:2
    if seat_typ(i)==2
        rear_seats_weight=rear_seats_weight+rear_bench_weight; %Add weight of a seat bench 
    end
end

%Check that the result is not negative
check_mass(front_seats_weight);
check_mass(rear_seats_weight);

%Mode for 3 seat rows
if isfield(v.Input,'int_3rows')
    if v.Input.int_3rows==1
        if strcmp(v.Input.int_3rows_type,'3vavcon')
            front_seats_weight=front_seats_weight*2;
        else
            rear_seats_weight=rear_seats_weight*2;
        end
    end
end

%Calculate total seat weight
seats_weight = front_seats_weight + rear_seats_weight;                                                   % [kg]



%% 4) Weight of HVAC
%Heating system and A/C
%A/C: Condenser, comporessor, A/C lines anfd valves+refrigerant.
AC_weight = Par.masses.AC_system + Par.masses.AC_refrigerant;%16.539+0.52;   % [kg]

%Heating system: Air vents, blowing unit system, radiator, evaporator,
%temp. sensors, filters, air intakes, resistors, defrosters
heating_weight = regr_heating_system_cabin(wheelbase,width); %-11.578 + 0.010*wheelbase;  % [kg]

%Check that the result is not negative
check_mass(heating_weight);

HVAC_weight = AC_weight+heating_weight;

%% 5) Weight of center console
%Center console: the console between the front seats:
%Structure, cupholders, parking brake switch, storage compartments
center_console_weight = regr_center_console(wheelbase); %-25.827+0.012*wheelbase;   % [kg]

%Check that the result is not negative
check_mass(center_console_weight);

%% 6) Weight of instrument panel
% Dashboard and dashboard covers
dashboard_weight = Par.masses.dashboard;%7.4790;                           % [kg]

% Cluster system (if present): instrument cluster in front of the driver
if cluster_type == 1  % digital instrument cluster
    cluster_weight = Par.masses.cluster_system.digital;%1.675              % [kg]
else %analog
    cluster_weight = Par.masses.cluster_system.analog;%1.27                % [kg]
end 

% Cross Car beam  
cross_car_beam_weight = Par.masses.cross_car_beam; %6.1685;                % [kg]

% LCD screen (if present) in the center of the dashboard. Includes screen and Hardware weight
if infotainment == 1  
    infotainment_weight = Par.masses.infotainment;%1.03;                   % [kg]
else 
    infotainment_weight = 0;
end

% Head up display (if present) in the driver's field of view
if HUD == 1
    HUD_weight = Par.masses.HUD;%1.375;                                    % [kg]
else
    HUD_weight = 0;                                                
end

%glovebox storage compartment with lid.
glovebox_weight=Par.masses.glovebox; %2.32;                                % [kg]

instrument_panel_weight = dashboard_weight+cluster_weight+...
    cross_car_beam_weight+infotainment_weight+HUD_weight+glovebox_weight;

%% 7) Weight of noise insulation material
% mean value based based on the segment (the segment division is made based on the wheelbase)
if wheelbase <=2750 %segments A to C
   noise_ins_weight = Par.masses.noise_insulation.A_to_C;
else %larger wheelbase-->segments D to F
   noise_ins_weight = Par.masses.noise_insulation.D_to_F;
end

%% 8) Weight of trim parts
%interior trim covers on pillars, roof, floor
trim_parts_weight = regr_trim_parts(width,veh_length);%-231.566 + 0.094*width + 0.016*veh_length + 0.015*height;  % [kg] 

%Check that the result is not negative
check_mass(trim_parts_weight);

%% 9) Weight of speakers 
speakers_weight = Par.masses.speaker.dashboard + 2*Par.masses.speaker.front + 2*Par.masses.speaker.rear; % [kg]

%if a subwoofer is present, a constant value is added to the speaker's weight.
if subwoofer == 1
   speakers_weight =  speakers_weight + Par.masses.speaker.subwoofer; %2.70;     % [kg]    
end

%% Assign Output

v.masses.interior.safety_equipment_weight=safety_equipment_weight;
v.masses.interior.center_console_weight=center_console_weight;
v.masses.interior.HVAC_weight=HVAC_weight;
v.masses.interior.instrument_panel_weight=instrument_panel_weight;
v.masses.interior.noise_insulation_weight=noise_ins_weight;
v.masses.interior.seats_weight=seats_weight;
v.masses.interior.speakers_weight=speakers_weight;
v.masses.interior.trim_parts_weight=trim_parts_weight;

end