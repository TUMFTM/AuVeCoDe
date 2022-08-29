function [technical_data,mot_data] = READ_techdata(vehicle)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Dila Akguel (Technical University of Munich), Fabian Liemawan Adji (Technical University of Munich)
%-------------
% Created on: 19.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function estimates the manufacturing cost of a vehicle based its technical data generated with function "cost". 
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] Dila Akguel, "Development of a Tool for the Cost Estimation of Autonomous Vehicles", Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2020
% ------------
% Input:    - vehicle:struct "Vehicle"
%           
% ------------
% Output:   - technical_data:technical data vector of the vehicle
% ------------

%% Implementation
% 1) Adaptation for weight model notation
% 2) Read in technical data

%% 1) Adaptation for weight model notation
if ~isfield(vehicle.masses.powertrain,'gearbox_weight_front')
    gearbox_weight=vehicle.masses.powertrain.gearbox_weight;
else
    gearbox_weight=vehicle.masses.powertrain.gearbox_weight_front + vehicle.masses.powertrain.gearbox_weight_rear;
end
%% 2) Read in technical data
%Initialize data vectors
technical_data=zeros(72,1); %vector with all data except motor
mot_data=zeros(4,2);        %vector with motor data (4 rows for 2x motortypes and 2x axles, 2 columns for up to 2 motors per axle)

%Powertrain
for i=1:2%Front and rear axle
    if ~isempty(vehicle.LDS.MOTOR{i})
        if strcmp(vehicle.Input.machine_type_f,'ASM')
            mot_data(i*2-1,1) = vehicle.LDS.MOTOR{1, i}.P_max_mech; %first motor
            if vehicle.LDS.MOTOR{1, i}.quantity>1 %only if second motor per axle
                mot_data(i*2-1,2) = vehicle.LDS.MOTOR{1, i}.P_max_mech;
            end
        else
            mot_data(i*2,1) = vehicle.LDS.MOTOR{1, i}.P_max_mech; %first motor
            if vehicle.LDS.MOTOR{1, i}.quantity>1 %only if second motor per axle
                mot_data(i*2,2) = vehicle.LDS.MOTOR{1, i}.P_max_mech;
            end
        end
    end
end

technical_data(1,:) = sum(mot_data,'all'); %Calculate power of complete vehicle

technical_data(2,1)=vehicle.battery.energy_is_gross_in_kWh;                          %Battery capacity (in kWh)
technical_data(3,1)=0;                                                      %no use for  reluctance motor
technical_data(4,1)=vehicle.Equipment.Powertrain.on_board_charger;          % is there an onboard charger? (1-0)
technical_data(5,1)=vehicle.Equipment.Powertrain.driveshaft.driveshaft_mass;
technical_data(6,1)=vehicle.Equipment.Powertrain.differential.mass_differential*vehicle.Equipment.Powertrain.differential.material_distribution.steel;
technical_data(7,1)=(vehicle.Equipment.Powertrain.differential.mass_differential)*(vehicle.Equipment.Powertrain.differential.material_distribution.alu);
technical_data(8,1)=(gearbox_weight)*vehicle.masses.powertrain.perc_gearbox_steel;
technical_data(9,1)=(gearbox_weight)*vehicle.masses.powertrain.perc_gearbox_alu;

%Chassis
technical_data(10,1)=vehicle.masses.chassis.rear_axle_weight+vehicle.masses.chassis.front_axle_weight;
technical_data(11,1)=vehicle.Equipment.Powertrain.electromech_steering;     %is the steering electromechanical (1-0)
technical_data(12,1)=vehicle.Equipment.Powertrain.steer_by_wire;            %Steer by Wire (1-0)
technical_data(13,1)=vehicle.Equipment.Powertrain.electromech_braking;      %conventional braking(1-0)
technical_data(14,1)=1;                                                     %the price of shock absorber is standard
technical_data(15,1)=vehicle.masses.chassis.rims_weight*(vehicle.Equipment.Powertrain.wheels.Alu_perc_rim);
technical_data(16,1)=vehicle.masses.chassis.rims_weight*(vehicle.Equipment.Powertrain.wheels.Steel_perc_rim);
technical_data(17,1)=vehicle.masses.chassis.tires_weight;                   

%Electronics
technical_data(18,1)=vehicle.Equipment.Electronics.front_headlight;
technical_data(19,1)=vehicle.Equipment.Electronics.back_headlight;
technical_data(20,1)=vehicle.Equipment.Electronics.back_up_battery;
technical_data(21,1)=0;                                                     %alternative to onboard charger (not standard)
technical_data(22,1)=1;                                                     %Wiring Harness is standard -

%Interior
if strcmp(vehicle.Equipment.Interior.seat_type,'aut')                              %are seats automatic or manual adjustable (aut-mech)
  technical_data(23,1)=0;
  technical_data(24,1)=vehicle.Input.n_seat(1,1);                           %number of front seats 
elseif strcmp(vehicle.Equipment.Interior.seat_type,'mech')
  technical_data(23,1)=vehicle.Input.n_seat(1,1);  
  technical_data(24,1)=0;
end

technical_data(25,1)=vehicle.Input.n_seat(1,2);                            %number of back seats
technical_data(26,1)=vehicle.Equipment.Interior.n_seat_warmers;            %number of seat warmers
technical_data(27,1)=vehicle.Equipment.Interior.n_airbags;                 %standard airbag SET 
technical_data(28,1)=vehicle.Equipment.Interior.n_side_airbags;            %number of side airbags
technical_data(29,1)=vehicle.Input.n_seat(1,1);                            %fr_seatbelts
technical_data(30,1)=vehicle.Input.n_seat(1,2);                            %back seatbelts
technical_data(31,1)=vehicle.Equipment.Interior.n_displays;                %number of displays
technical_data(32,1)=vehicle.Equipment.Interior.HVAC;                      %is there HVAC?
technical_data(33,1)=vehicle.masses.interior.noise_insulation_weight;
technical_data(34,1)=vehicle.Equipment.Exterior.Pneumatic_doors;           %are the doors pneumatic? (1-0)
technical_data(35,1)=vehicle.masses.exterior.windshield_windows_weight;    %glass
technical_data(36,1)=vehicle.Equipment.Exterior.n_windows_lifter;          %number of windows lifters
technical_data(37,1)=vehicle.Equipment.Exterior.n_windshield_wiper;        %windshield wiper (2-1-0)
technical_data(38,1)=1;                                                    %Investment cost per vehicle closures is standard
technical_data(39,1)=1;                                                    %Energy cost per vehicle closures is standard

%Exterior
technical_data(40,1)=vehicle.masses.exterior.closures_weight*vehicle.Equipment.Exterior.Closures_Material_Distribution.alu_closures;
technical_data(41,1)=0;                                                    %Tool doesn't consider CRFP
technical_data(42,1)=vehicle.masses.exterior.closures_weight*vehicle.Equipment.Exterior.Closures_Material_Distribution.steel_closures;
technical_data(43,1)=vehicle.Equipment.Exterior.Interior_panelling_Weight*vehicle.Equipment.Exterior.Closures_Material_Distribution.Interior_panelling_abs;
technical_data(44,1)=vehicle.Equipment.Exterior.Interior_panelling_Weight*vehicle.Equipment.Exterior.Closures_Material_Distribution.Interior_panelling_pp;
technical_data(45,1)=-vehicle.masses.exterior.closures_weight*vehicle.Equipment.Exterior.Closures_Material_Distribution.steel_closures;
technical_data(46,1)=-vehicle.masses.exterior.closures_weight*vehicle.Equipment.Exterior.Closures_Material_Distribution.alu_closures;
technical_data(47,1)=0;                                                    %mix scrap

%Structure
technical_data(48,1)=(vehicle.masses.optional_extras.alu_perc_BIW/100)*(vehicle.masses.frame.BIW_weight);
technical_data(49,1)=0;
technical_data(50,1)=(1-(vehicle.masses.optional_extras.alu_perc_BIW/100))*(vehicle.masses.frame.BIW_weight); 
technical_data(51,1)=-(1-(vehicle.masses.optional_extras.alu_perc_BIW/100))*(vehicle.masses.frame.BIW_weight); %scrap
technical_data(52,1)=-(vehicle.masses.optional_extras.alu_perc_BIW/100)*(vehicle.masses.frame.BIW_weight);   
technical_data(53,1)=0;                                                    %mix scrap
technical_data(54,1)=0;                                                    %wage cost per vehicle -- choose of them--
technical_data(55,1)=vehicle.masses.frame.BIW_weight;                      %wage cost per kg
technical_data(56,1)=1;                                                    %energy costs per vehicle
technical_data(57,1)=0;                                                    %investment costs per vehicle (fix cost) 
technical_data(58,1)=vehicle.masses.frame.BIW_weight;                      %investment costs per kg

%Sensors
technical_data(59,1)=vehicle.Equipment.Sensors.Radar;                       %number of radars
technical_data(60,1)=vehicle.Equipment.Sensors.Sonar;                       %number of sonars
technical_data(61,1)=vehicle.Equipment.Sensors.Lidar.n_lidar_mems;          %number of mems lidar
technical_data(62,1)=vehicle.Equipment.Sensors.Lidar.n_lidar_mech;          %number of mechanic lidar
technical_data(63,1)=vehicle.Equipment.Sensors.Surround_Camera;             %number of surround cameras
technical_data(64,1)=vehicle.Equipment.Sensors.Front_Camera;                %number of front cameras
technical_data(65,1)=vehicle.Equipment.Sensors.Computer;                    %number of computers
technical_data(66,1)=vehicle.Equipment.Sensors.GPS.GPS_receiver;            %Is there an RTK GPS receiver?
technical_data(67,1)=vehicle.Equipment.Sensors.GPS.GPS_Correction_Service;  %Subscription for a GPS correction service? 
technical_data(68,1)=vehicle.Equipment.Sensors.FiveG;                       %5G
technical_data(69,1)=vehicle.Equipment.Sensors.C2X;                         %Car to X technology
technical_data(70,1)=0;                                                     %ESP and ACC already considered in sensors
technical_data(71,1)=0;                                                     %ESP and ACC already considered in sensors

%Assemlby
technical_data(72,1)=vehicle.Equipment.Assembly.h_assembly;                 %Assembly time of the vehicle

end

