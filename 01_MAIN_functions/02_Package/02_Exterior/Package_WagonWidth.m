function [vehicle]=Package_WagonWidth(vehicle,wagontype,drivetrain_type,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow (Technical University of Munich), Korbinian Moller (Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the width of the vehicle and positions the 
%               powertrain components
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
%           - Parameters: struct with input and constant values%           - 
%           - Drivertrain type (see Package_assign_topology)
%           - Wagon type (1=front, 2=rear)
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------

%% Calculate y-measurements and positions of powertrain
%% 1) Assign powertrain dimensions
drivetrain_type=drivetrain_type(wagontype);
if wagontype==1
    if drivetrain_type~=0 % motor on front axle available
        vehicle.dimensions.p_motor_paral{1}(2,:)=[vehicle.e_machine{1}.CX_e_machine_diameter/2 vehicle.e_machine{1}.CY_e_machine_length 2];
        vehicle.dimensions.p_motor_coax{1}(2,:)=[vehicle.e_machine{1}.CX_e_machine_diameter/2 vehicle.e_machine{1}.CY_e_machine_length 5];
        vehicle.dimensions.p_motor_nax{1}(2,:)=[vehicle.e_machine{1}.CX_e_machine_diameter/2 vehicle.e_machine{1}.CY_e_machine_length 5];
    end
elseif wagontype==2
    if drivetrain_type~=0 % motor on rear axle available
        vehicle.dimensions.p_motor_paral{2}(2,:)=[vehicle.e_machine{2}.CX_e_machine_diameter/2 vehicle.e_machine{2}.CY_e_machine_length 2];
        vehicle.dimensions.p_motor_coax{2}(2,:)=[vehicle.e_machine{2}.CX_e_machine_diameter/2 vehicle.e_machine{2}.CY_e_machine_length 5];
        vehicle.dimensions.p_motor_nax{2}(2,:)=[vehicle.e_machine{2}.CX_e_machine_diameter/2 vehicle.e_machine{2}.CY_e_machine_length 5];
    end
end

%% 2) Calculate powertrain width
switch drivetrain_type
    case 0         %no motor
        %Width defined later on
    vehicle.dimensions.p_drivetrain{wagontype}(2,2)=0;
    case 1        %parallel topology of motor
        %Width is sum of motor length and longest gear stage
        vehicle.dimensions.p_drivetrain{wagontype}(2,2)=vehicle.dimensions.p_motor_paral{wagontype}(2,2)+vehicle.dimensions.p_gear_paral{wagontype}(10,2);
    
    case 2         %coaxial planetary topology of motor
        %Width is sum of motor and gear (incl. differential)
        vehicle.dimensions.p_drivetrain{wagontype}(2,2)=vehicle.dimensions.p_motor_coax{wagontype}(2,2)+vehicle.dimensions.p_gear_coax{wagontype}(2,2);
    
    case 3       %near-wheel topology of motor
        %Width is sum of 2 motors and 2 gears (without differential)
        vehicle.dimensions.p_drivetrain{wagontype}(2,2)=2*(vehicle.dimensions.p_motor_nax{wagontype}(2,2)+vehicle.dimensions.p_gear_nax{wagontype}(2,2));     
        
    case 4      %coaxial lay-shaft topology of motor
        %Width is sum of motor and gear (incl. differential)
        vehicle.dimensions.p_drivetrain{wagontype}(2,2)=vehicle.dimensions.p_motor_coax{wagontype}(2,2)+vehicle.dimensions.p_gear_coax_ls{wagontype}(10,2);
        vehicle.dimensions.p_motor_coax{wagontype}(2,3)=6; %For Package_Layer_Calculate
end

%% 3) Check available width of interior to place motor/gear
% Left wheelhouse center position (half wheelhouse width)
vehicle.dimensions.p_wheelhouse_left{wagontype}(1,2)=vehicle.dimensions.wheelhouse_length(wagontype)/2;

% Right wheelhouse center position (veh. width - half wheelhouse width)
vehicle.dimensions.p_wheelhouse_right{wagontype}(1,2)=vehicle.dimensions.GY.vehicle_width-0.5*vehicle.dimensions.wheelhouse_length(wagontype);

% Reference points
if wagontype==1
    o_x1=vehicle.dimensions.wheelhouse_length(wagontype)+Parameters.body.F_Frame_w; % right bound of left wheelhouse (start of "free powertrain space")
elseif wagontype==2
    o_x1=vehicle.dimensions.wheelhouse_length(wagontype)+Parameters.body.R_Frame_w+vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber; % right bound of left wheelhouse (start of "free powertrain space")
end

% Available width between wheelhouses (veh. width - 2 wheelhouse widths)
if wagontype==1
    vehicle.dimensions.width_avail(wagontype) = vehicle.dimensions.GY.vehicle_width-2*vehicle.dimensions.wheelhouse_length(wagontype)-2*Parameters.body.F_Frame_w; %Wheelhouse and longitudinal beam
elseif wagontype==2
    vehicle.dimensions.width_avail(wagontype) = vehicle.dimensions.GY.vehicle_width-2*vehicle.dimensions.wheelhouse_length(wagontype)-2*Parameters.body.R_Frame_w-2*vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber; %additional shock absorber
end
%% 4) Positioning components

if vehicle.dimensions.width_avail(wagontype)>=vehicle.dimensions.p_drivetrain{wagontype}(2,2) %Ignore further calculation and increase width if available width is smaller than drivetrain width
    
    % Gap between powertrain and wheelhouses ((avaialble width - drivetrain)/2)
    gap=(vehicle.dimensions.width_avail(wagontype)-vehicle.dimensions.p_drivetrain{wagontype}(2,2))/2;
    
    % Calculate maximal offset of driveshaft due to the angle
    x_offset=gap*tand(vehicle.Input.alpha_axle(wagontype,1)); %x-Offset due to angle of driveshaft
    z_offset=gap*tand(vehicle.Input.alpha_axle(wagontype,2)); %z-Offset due to angle of driveshaft
    vehicle.dimensions.p_offset_driveshaft(wagontype,1)=round(x_offset); %round so that indexing is not influenced later on
    vehicle.dimensions.p_offset_driveshaft(wagontype,2)=round(z_offset); %round so that indexing is not influenced later on
    
    % Calculate position of powertrain components
    switch drivetrain_type
        case 0 %No motor

        case 1 %Parallel topology of motor
            
            %Calculate Motor center point position (Wheelhouse+gap+half length of motor)
            vehicle.dimensions.p_motor_paral{wagontype}(1,2)=o_x1+gap+0.5*vehicle.dimensions.p_motor_paral{wagontype}(2,2);
            
            %Set Powerelectronic center position (center of wagon)
            vehicle.dimensions.p_powerel{wagontype}(1,2)=vehicle.dimensions.GY.vehicle_width/2;
            
            %Calculate Gear center point y-position (offset (from Package_Powertrain)+motor center point+half length of motor+ half length of gear (max. gear stage))
            gearbox_motor_side = vehicle.dimensions.p_motor_paral{wagontype}(1,2)+0.5*vehicle.dimensions.p_motor_paral{wagontype}(2,2); %beginning of gearbox at motor side
            
            vehicle.dimensions.p_gear_paral{wagontype}(1,2)=gearbox_motor_side+vehicle.dimensions.p_gear_paral{wagontype}(1,2)+0.5*vehicle.dimensions.p_gear_paral{wagontype}(2,2);  %gearwheel 1
            vehicle.dimensions.p_gear_paral{wagontype}(3,2)=gearbox_motor_side+vehicle.dimensions.p_gear_paral{wagontype}(3,2)+0.5*vehicle.dimensions.p_gear_paral{wagontype}(2,2);  %gearwheel 2
            vehicle.dimensions.p_gear_paral{wagontype}(5,2)=gearbox_motor_side+vehicle.dimensions.p_gear_paral{wagontype}(5,2)+0.5*vehicle.dimensions.p_gear_paral{wagontype}(6,2);  %gearwheel 3
            vehicle.dimensions.p_gear_paral{wagontype}(7,2)=gearbox_motor_side+vehicle.dimensions.p_gear_paral{wagontype}(7,2)+0.5*vehicle.dimensions.p_gear_paral{wagontype}(8,2);  %gearwheel 3
            vehicle.dimensions.p_gear_paral{wagontype}(9,2)=vehicle.dimensions.p_motor_paral{wagontype}(1,2)+0.5*vehicle.dimensions.p_motor_paral{wagontype}(2,2)+0.5*vehicle.dimensions.p_gear_paral{wagontype}(10,2); %housing
            
            %Calculate Gear Auxiliary center point y-position (Motor offset + y_center point) -- see info in Package_Powertrain -- saved y_position of aux components is already center point
            for i = 1:2:size(vehicle.dimensions.p_gear_paral_aux{wagontype},1)
            vehicle.dimensions.p_gear_paral_aux{wagontype}(i,2) = gearbox_motor_side + vehicle.dimensions.p_gear_paral_aux{wagontype}(i,2);
            end
            

            
        case 2 %Coaxial planetary topology of motor
            
            %Calculate Motor center point position (Wheelhouse+gap+half length of motor)
            vehicle.dimensions.p_motor_coax{wagontype}(1,2)=o_x1+gap+0.5*vehicle.dimensions.p_motor_coax{wagontype}(2,2);
            
            %Set Powerelectronic center position (center of wagon)
            vehicle.dimensions.p_powerel{wagontype}(1,2)= vehicle.dimensions.GY.vehicle_width/2;
            
            %Calculate Gear center point position (motor center point+half length of motor+ half length of gear (max. gear stage))
            vehicle.dimensions.p_gear_coax{wagontype}(1,2)=vehicle.dimensions.p_motor_coax{wagontype}(1,2)+0.5*vehicle.dimensions.p_motor_coax{wagontype}(2,2)+0.5*vehicle.dimensions.p_gear_coax{wagontype}(2,2);
            
            %Calculate Gearbox components center position (Motor offset + y_center point)
            gearbox_motor_side = vehicle.dimensions.p_motor_coax{wagontype}(1,2) + 0.5*vehicle.dimensions.p_motor_coax{wagontype}(2,2); %beginning of gearbox at motor side
            for i = 1:size(vehicle.dimensions.comp_gear_plan{wagontype},1)
                vehicle.dimensions.comp_gear_plan{wagontype}(i,2) = gearbox_motor_side + vehicle.dimensions.comp_gear_plan{wagontype}(i,2);
            end
            
        case 3 %Near-wheel topology of motor
            
            %select corresponding wheelhouse width for calculation of gearbox center point
            if wagontype == 1  %front axle: longitudinal beam has to be added
                wheelhouse_width = vehicle.dimensions.CY.wheelhouse_f_width+Parameters.body.F_Frame_w;
            else % at rear axle --> rear axle: shock absorber width must be added and longitudinal beam has to be added
                wheelhouse_width = vehicle.dimensions.CY.wheelhouse_r_width + vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber + Parameters.body.R_Frame_w;
            end
            
            %Calculate Gear center point position
            %Gear 1 (wheelhouse + half width gearbox)
            vehicle.dimensions.p_gear_nax{wagontype}(1,2) = wheelhouse_width + 0.5*vehicle.dimensions.p_gear_nax{wagontype}(2,2);
            
            %Gear 2 (veh. width - wheelhouse - half width gearbox)
            vehicle.dimensions.p_gear_nax{wagontype}(3,2)=vehicle.dimensions.GY.vehicle_width - wheelhouse_width - 0.5*vehicle.dimensions.p_gear_nax{wagontype}(2,2);
            
            %Calculate Gearbox components center position (Motor offset + y_center point)
            gearbox_motor_side_1 = vehicle.dimensions.p_gear_nax{wagontype}(1,2) + 0.5*vehicle.dimensions.p_gear_nax{wagontype}(2,2); % left side of vehicle
            gearbox_motor_side_2 = vehicle.dimensions.p_gear_nax{wagontype}(3,2) - 0.5*vehicle.dimensions.p_gear_nax{wagontype}(2,2); % right side of vehicle
            
            for i = 1:size(vehicle.dimensions.comp_gear_nax_1{wagontype},1)
                vehicle.dimensions.comp_gear_nax_1{wagontype}(i,2) = gearbox_motor_side_1 + vehicle.dimensions.comp_gear_nax_1{wagontype}(i,2);
                vehicle.dimensions.comp_gear_nax_2{wagontype}(i,2) = gearbox_motor_side_2 + vehicle.dimensions.comp_gear_nax_2{wagontype}(i,2);
            end
            
            %Calculate Motors center point position
            %Motor 1 (gearbox pos. + half width gearbox + half width motor)
            vehicle.dimensions.p_motor_nax{wagontype}(1,2)=vehicle.dimensions.p_gear_nax{wagontype}(1,2)+0.5*vehicle.dimensions.p_gear_nax{wagontype}(2,2)+0.5*vehicle.dimensions.p_motor_nax{wagontype}(2,2);
            %Motor 2 (gearbox pos. - half width gearbox - half width motor)
            vehicle.dimensions.p_motor_nax{wagontype}(3,2)=vehicle.dimensions.p_gear_nax{wagontype}(3,2)-0.5*vehicle.dimensions.p_gear_nax{wagontype}(2,2)-0.5*vehicle.dimensions.p_motor_nax{wagontype}(2,2);
            
            %Calculate Powerelectronic center position (center of vehicle)
            vehicle.dimensions.p_powerel{wagontype}(1,2)=vehicle.dimensions.GY.vehicle_width/2;
            
        case 4 %Coaxial layshaft topology of motor
            %Calculate Motor center point position (Wheelhouse+gap+half length of motor)
            vehicle.dimensions.p_motor_coax{wagontype}(1,2)=o_x1+gap+0.5*vehicle.dimensions.p_motor_coax{wagontype}(2,2);
            
            %Set Powerelectronic center position (center of wagon)
            vehicle.dimensions.p_powerel{wagontype}(1,2)= vehicle.dimensions.GY.vehicle_width/2;
            
            %Calculate Gear center point position (motor center point+half length of motor+ half length of gear (max. gear stage))
            gearbox_motor_side = vehicle.dimensions.p_motor_coax{wagontype}(1,2)+0.5*vehicle.dimensions.p_motor_coax{wagontype}(2,2); %beginning of gearbox at motor side
            
            vehicle.dimensions.p_gear_coax_ls{wagontype}(1,2)=gearbox_motor_side+vehicle.dimensions.p_gear_coax_ls{wagontype}(1,2)+0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(2,2);  %gearwheel 1
            vehicle.dimensions.p_gear_coax_ls{wagontype}(3,2)=gearbox_motor_side+vehicle.dimensions.p_gear_coax_ls{wagontype}(3,2)+0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(4,2);  %gearwheel 2
            vehicle.dimensions.p_gear_coax_ls{wagontype}(5,2)=gearbox_motor_side+vehicle.dimensions.p_gear_coax_ls{wagontype}(5,2)+0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(6,2);  %gearwheel 3
            vehicle.dimensions.p_gear_coax_ls{wagontype}(7,2)=gearbox_motor_side+vehicle.dimensions.p_gear_coax_ls{wagontype}(7,2)+0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(8,2);  %gearwheel 4
            vehicle.dimensions.p_gear_coax_ls{wagontype}(9,2)=vehicle.dimensions.p_motor_coax{wagontype}(1,2)+0.5*vehicle.dimensions.p_motor_coax{wagontype}(2,2)+0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(10,2);
            
            %Calculate Gear Auxiliary center point y-position (Motor offset + y_center point) -- see info in Package_Powertrain -- saved y_position of aux components is already center point
            for i = 1:2:size(vehicle.dimensions.p_gear_coax_ls_aux{wagontype},1)
            vehicle.dimensions.p_gear_coax_ls_aux{wagontype}(i,2) = gearbox_motor_side + vehicle.dimensions.p_gear_coax_ls_aux{wagontype}(i,2);
            end
            
    end
    
else % Wagon width is wider then interior (new width = drivetrain width + wheelhouses)
    vehicle.Error=1;
    if wagontype==1
        vehicle.dimensions.GY.vehicle_width= ceil(2*vehicle.dimensions.wheelhouse_length(wagontype)+vehicle.dimensions.p_drivetrain{wagontype}(2,2)+2*Parameters.body.F_Frame_w);
    elseif wagontype==2
        vehicle.dimensions.GY.vehicle_width= ceil(2*vehicle.dimensions.wheelhouse_length(wagontype)+vehicle.dimensions.p_drivetrain{wagontype}(2,2)+2*Parameters.body.R_Frame_w+2*vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber);
    end
    return
end
end