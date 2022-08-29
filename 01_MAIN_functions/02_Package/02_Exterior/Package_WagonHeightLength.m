function [vehicle]=Package_WagonHeightLength(vehicle,wagontype,drivetrain_type,psi)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow (Technical University of Munich), Korbinian Moller (Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the height and length of the vehicle and
%               positions the powertrain components
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - vehicle: struct with includes the data of the vehicle
%           - Drivertrain type (see Package_assign_topology)
%           - Wagon type (1=front, 2=rear)
%           - Psi: Angle of Motor to Gear in Degree
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------

%% Calculate x- and z-measurements and positions of powertrain

%% 1) Parameter initialization
%Read motor type depending on front or rear wagon
if wagontype==1
    drivetrain_type=drivetrain_type(1);
elseif wagontype==2
    drivetrain_type=drivetrain_type(2);
end

%% 2) Calculate Distance of Motor and Gear (height, length)
switch drivetrain_type
    case 0 % No motor
        vehicle.dimensions.p_drivetrain{wagontype}(2,3)=0;
        vehicle.dimensions.p_drivetrain{wagontype}(1,3)=0;
        vehicle.dimensions.p_drivetrain{wagontype}(2,1)=0;
        vehicle.dimensions.p_drivetrain{wagontype}(1,1)=0;
    case 1 % Parallel topology of motor
        %% Height (z-direction)
        % 1) Gearbox and Motor
        % 1a) Read gear z-position and add driveshaft offset
        gear_z_1=vehicle.dimensions.p_gear_paral{wagontype}(1,3);
        gear_z_2=vehicle.dimensions.p_gear_paral{wagontype}(3,3);
        gear_z_3=vehicle.dimensions.p_gear_paral{wagontype}(5,3);
        gear_z_4=vehicle.dimensions.p_gear_paral{wagontype}(7,3);
        
        vehicle.dimensions.p_gear_paral{wagontype}(1,3)=gear_z_1+vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        vehicle.dimensions.p_gear_paral{wagontype}(3,3)=gear_z_2+vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        vehicle.dimensions.p_gear_paral{wagontype}(5,3)=gear_z_3+vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        vehicle.dimensions.p_gear_paral{wagontype}(7,3)=gear_z_4+vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        vehicle.dimensions.p_gear_paral{wagontype}(9,3)=vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        
        %Calculate updated center point z-position of Gear Auxiliary components
        for i = 1:2:size(vehicle.dimensions.p_gear_paral_aux{wagontype},1)
        vehicle.dimensions.p_gear_paral_aux{wagontype}(i,3) = vehicle.dimensions.p_gear_paral_aux{wagontype}(i,3) + vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        end
        
        % 1b) Read motor z-position (same as third gear stage)
        vehicle.dimensions.p_motor_paral{wagontype}(1,3)=round(vehicle.dimensions.p_gear_paral{wagontype}(1,3));
        
        % 2) Read parameters from calculation of gearbox
        d_flange=vehicle.gearbox{wagontype}.ConstDim.d_flange;
        t_housing=vehicle.gearbox{wagontype}.ConstDim.t_housing;
        d_housing=vehicle.gearbox{wagontype}.ConstDim.d_housing;
        d_diff=vehicle.gearbox{wagontype}.gears_34.d_a4;
        l_gearbox=vehicle.gearbox{wagontype}.results.l_gearbox;
        h_gearbox=vehicle.gearbox{wagontype}.results.h_gearbox;
        t_gearbox=vehicle.gearbox{wagontype}.results.t_gearbox;
        d_wheelmotor=vehicle.gearbox{wagontype}.shafts.d_1_max;
        l_park=vehicle.gearbox{wagontype}.ConstDim.l_park;
        a_13=vehicle.gearbox{wagontype}.results.a_13;
        %----Optional----
        % Calculate length of a_13
        check_length=(l_gearbox-(d_housing+t_housing+d_flange+0.5*d_diff)-(l_park+0.5*d_wheelmotor));
        
        % Check if a_13 is equal to check_length
        if abs(a_13-check_length)<0.001
        else
            error('There is an inconsistency in the gear calculation. Please check for errors!')
        end
        %----Optional/end---- 
        
        % 3) Calculate housing
        % Calculate offset of housing middle point to a4 middle point
        dist_a4_to_housing=d_housing+t_housing+d_flange+d_diff/2; %distance between mid-point of a4 and housing wall
        r_housing=h_gearbox/2; %radius of curve at the end of gearbox
        
        diff_a4_to_housing= r_housing-dist_a4_to_housing; %difference of two middle points
        diff_a4_to_housing_x=cosd(psi)*diff_a4_to_housing; %x-part (for length)
        diff_a4_to_housing_z=sind(psi)*diff_a4_to_housing; %z-part
        
        vehicle.dimensions.p_gear_paral{wagontype}(9,3)=diff_a4_to_housing_z;%offset between housing middle point and gear 
        
        % 4) Calculate powerelectronics and complete drivetrain measurements
        % 4a) Min. height of gear housing (mid point of lower housing circle - one radius (in z-direction))
        height_house_paral_min=vehicle.dimensions.p_gear_paral{wagontype}(7,3)+vehicle.dimensions.p_gear_paral{wagontype}(9,3)...
            -0.5*vehicle.dimensions.p_gear_paral{wagontype}(10,3);
        % 4b) Min. height of drivetrain (without powerelectronics)
        height_min=min(height_house_paral_min,(vehicle.dimensions.p_motor_paral{wagontype}(1,3)-vehicle.dimensions.p_motor_paral{wagontype}(2,1))); %minimal height is either due to gear or motor
        % 4c) Max. height of gear housing (mid point of lower housing circle + length to upper circle + one radius (in z-direction)
        height_house_paral_max=vehicle.dimensions.p_gear_paral{wagontype}(7,3)+vehicle.dimensions.p_gear_paral{wagontype}(9,3)...
            +(vehicle.dimensions.p_gear_paral{wagontype}(10,1)-vehicle.dimensions.p_gear_paral{wagontype}(10,3))*sind(psi)+0.5*vehicle.dimensions.p_gear_paral{wagontype}(10,3);
        height_max_temp=max(height_house_paral_max,(vehicle.dimensions.p_motor_paral{wagontype}(1,3)+vehicle.dimensions.p_motor_paral{wagontype}(2,1))); %maximal height is either due to gear or motor
        
        % 5) Powerelectronics z-pos. (max. height of drivertrain + half powerelectronics height)
        vehicle.dimensions.p_powerel{wagontype}(1,3)=height_max_temp+0.5*vehicle.dimensions.p_powerel{wagontype}(2,3); % Position of power electronics is direct above motor
        
        % 6) Dimensions of drivetrain
        % Max. height of drivetrain (with powerelectronics)
        height_max=vehicle.dimensions.p_powerel{wagontype}(1,3)+0.5*vehicle.dimensions.p_powerel{wagontype}(2,3); %new height max = power electronic
        vehicle.dimensions.p_drivetrain{wagontype}(2,3)=height_max-height_min;
        vehicle.dimensions.p_drivetrain{wagontype}(1,3)=height_max;
        
        
        %% Length (x-direction)
        % 1) Gearbox and Motor
        % 1a) Calculate Gear center point x-position (axle center + driveshaft offset)
        gear_x_1=vehicle.dimensions.p_gear_paral{wagontype}(1,1);
        gear_x_2=vehicle.dimensions.p_gear_paral{wagontype}(3,1);
        gear_x_3=vehicle.dimensions.p_gear_paral{wagontype}(5,1);
        gear_x_4=vehicle.dimensions.p_gear_paral{wagontype}(7,1);
        
        vehicle.dimensions.p_gear_paral{wagontype}(1,1)=gear_x_1+vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        vehicle.dimensions.p_gear_paral{wagontype}(3,1)=gear_x_2+vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        vehicle.dimensions.p_gear_paral{wagontype}(5,1)=gear_x_3+vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        vehicle.dimensions.p_gear_paral{wagontype}(7,1)=gear_x_4+vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        
        %Calculate updated center point x-position of Gear Auxiliary components
        for i = 1:2:size(vehicle.dimensions.p_gear_paral_aux{wagontype},1)
            vehicle.dimensions.p_gear_paral_aux{wagontype}(i,1) = vehicle.dimensions.p_gear_paral_aux{wagontype}(i,1) + vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        end
        
        % 1b) Motor x-pos = x-pos. 1st stage
        vehicle.dimensions.p_motor_paral{wagontype}(1,1)=round(vehicle.dimensions.p_gear_paral{wagontype}(1,1));
        
        % 2) Gearbox Housing
        % Calculate offset between housing and a4 middle points        
        vehicle.dimensions.p_gear_paral{wagontype}(9,1)=diff_a4_to_housing_x; %offset between housing middle point and gear        

        % 3) Powerelectronics = motor x-pos
        vehicle.dimensions.p_powerel{wagontype}(1,1)=vehicle.dimensions.p_motor_paral{wagontype}(1,1);
        
        % 4) Dimensions of drivetrain
        % 4a) Min. and max. length due gearbox housing
        if psi<90
            length_house_paral_min=vehicle.dimensions.p_gear_paral{wagontype}(7,1)+vehicle.dimensions.p_gear_paral{wagontype}(9,1)...
                -0.5*vehicle.dimensions.p_gear_paral{wagontype}(10,3);
            length_house_paral_max=vehicle.dimensions.p_gear_paral{wagontype}(7,1)+vehicle.dimensions.p_gear_paral{wagontype}(9,1)...
            +(vehicle.dimensions.p_gear_paral{wagontype}(10,1)-vehicle.dimensions.p_gear_paral{wagontype}(10,3))*cosd(psi)+0.5*vehicle.dimensions.p_gear_paral{wagontype}(10,3);
        elseif psi>90
            length_house_paral_max=vehicle.dimensions.p_gear_paral{wagontype}(7,1)+vehicle.dimensions.p_gear_paral{wagontype}(9,1)...
                +0.5*vehicle.dimensions.p_gear_paral{wagontype}(10,3);
            length_house_paral_min=vehicle.dimensions.p_gear_paral{wagontype}(7,1)+vehicle.dimensions.p_gear_paral{wagontype}(9,1)...
            +(vehicle.dimensions.p_gear_paral{wagontype}(10,1)-vehicle.dimensions.p_gear_paral{wagontype}(10,3))*cosd(psi)-0.5*vehicle.dimensions.p_gear_paral{wagontype}(10,3);
        elseif psi==90 %house is half height in both directions
            length_house_paral_min=vehicle.dimensions.p_gear_paral{wagontype}(7,1)+vehicle.dimensions.p_gear_paral{wagontype}(9,1)-0.5*vehicle.dimensions.p_gear_paral{wagontype}(10,3);
            length_house_paral_max=vehicle.dimensions.p_gear_paral{wagontype}(7,1)+vehicle.dimensions.p_gear_paral{wagontype}(9,1)+0.5*vehicle.dimensions.p_gear_paral{wagontype}(10,3);
        end
        % 4b) Min. and max. length due to gearbox housing, powerelectronics or motor
        % Maximal length due to powerelectronics, gearbox or motor
        length_min=min([(vehicle.dimensions.p_powerel{wagontype}(1,1)-0.5*vehicle.dimensions.p_powerel{wagontype}(2,1)),... %power electronics
            length_house_paral_min,... %geaxbox
            (vehicle.dimensions.p_motor_paral{wagontype}(1,1)-vehicle.dimensions.p_motor_paral{wagontype}(2,1))]); %motor
        % Maximal length due to powerelectronics, gearbox or motor
        length_max=max([(vehicle.dimensions.p_powerel{wagontype}(1,1)+0.5*vehicle.dimensions.p_powerel{wagontype}(2,1)),... %power electronics
            length_house_paral_max,... %geaxbox
            (vehicle.dimensions.p_motor_paral{wagontype}(1,1)+vehicle.dimensions.p_motor_paral{wagontype}(2,1))]); %motor
        
        % 5) Calculate values and write them into vehicle
        vehicle.dimensions.p_drivetrain{wagontype}(2,1)=length_max-length_min;
        vehicle.dimensions.p_drivetrain{wagontype}(1,1)=length_max;
        
    case 2 %Coaxial topology of motor
%%---------------------------Height----------------------------------------
        % Gear z-position (axle center + driveshaft offset)
        vehicle.dimensions.p_gear_coax{wagontype}(1,3)=vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        
        %Calculate updated center point z-position of Gearbox components
        for i = 1:size(vehicle.dimensions.comp_gear_plan{wagontype},1)
            vehicle.dimensions.comp_gear_plan{wagontype}(i,3) = vehicle.dimensions.comp_gear_plan{wagontype}(i,3) + vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        end
        
        % Motor z-position (axle center + driveshaft offset)
        vehicle.dimensions.p_motor_coax{wagontype}(1,3)=vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        
        height_min=min([(vehicle.dimensions.p_gear_coax{wagontype}(1,3)-vehicle.dimensions.p_gear_coax{wagontype}(2,1)),(vehicle.dimensions.p_motor_coax{wagontype}(1,3)-vehicle.dimensions.p_motor_coax{wagontype}(2,1))]); %minimal height is either due to gear or motor
        % Temporary height max (without powerelectronics)
        height_max_temp=max([(vehicle.dimensions.p_gear_coax{wagontype}(1,3)+vehicle.dimensions.p_gear_coax{wagontype}(2,1)),(vehicle.dimensions.p_motor_coax{wagontype}(1,3)+vehicle.dimensions.p_motor_coax{wagontype}(2,1))]); %minimal height is either due to gear or motor
        
        % Powerelectronics z-pos. (height max + half powerelectronics height)
        vehicle.dimensions.p_powerel{wagontype}(1,3)=height_max_temp+0.5*vehicle.dimensions.p_powerel{wagontype}(2,3); % Position of power electronics is direct above motor
        
        % Redefine height max. (with powerelectronics)
        height_max=max([(vehicle.dimensions.p_powerel{wagontype}(1,3)+0.5*vehicle.dimensions.p_powerel{wagontype}(2,3)),(vehicle.dimensions.p_gear_coax{wagontype}(1,3)+vehicle.dimensions.p_gear_coax{wagontype}(2,1)),(vehicle.dimensions.p_motor_coax{wagontype}(1,3)+vehicle.dimensions.p_motor_coax{wagontype}(2,1))]); %maximal height is either due to gear or motor
        vehicle.dimensions.p_drivetrain{wagontype}(2,3)=height_max-height_min;
        vehicle.dimensions.p_drivetrain{wagontype}(1,3)=height_max;
        
%%---------------------------Length----------------------------------------        
        %Calculate Gear center point x-position (axle center (0) + driveshaft offset)
        vehicle.dimensions.p_gear_coax{wagontype}(1,1)= vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        
        %Calculate updated center point z-position of Gearbox components
        for i = 1:size(vehicle.dimensions.comp_gear_plan{wagontype},1)
            vehicle.dimensions.comp_gear_plan{wagontype}(i,1) = vehicle.dimensions.comp_gear_plan{wagontype}(i,1) + vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        end
        
        % Motor x-pos = x-pos. gearbox
        vehicle.dimensions.p_motor_coax{wagontype}(1,1)=vehicle.dimensions.p_gear_coax{wagontype}(1,1);
        
        % Powerelectronics = motor x-pos
        vehicle.dimensions.p_powerel{wagontype}(1,1)=vehicle.dimensions.p_motor_coax{wagontype}(1,1);
                        
        length_min=min([(vehicle.dimensions.p_powerel{wagontype}(1,1)-0.5*vehicle.dimensions.p_powerel{wagontype}(2,1)),(vehicle.dimensions.p_gear_coax{wagontype}(1,1)-vehicle.dimensions.p_gear_coax{wagontype}(2,1)),(vehicle.dimensions.p_motor_coax{wagontype}(1,1)-vehicle.dimensions.p_motor_coax{wagontype}(2,1))]); %minimal length is either due to gear or motor
        length_max=max([(vehicle.dimensions.p_powerel{wagontype}(1,1)+0.5*vehicle.dimensions.p_powerel{wagontype}(2,1)),(vehicle.dimensions.p_gear_coax{wagontype}(1,1)+vehicle.dimensions.p_gear_coax{wagontype}(2,1)),(vehicle.dimensions.p_motor_coax{wagontype}(1,1)+vehicle.dimensions.p_motor_coax{wagontype}(2,1))]); %maximal length is either due to gear or motor
        vehicle.dimensions.p_drivetrain{wagontype}(2,1)=length_max-length_min;
        vehicle.dimensions.p_drivetrain{wagontype}(1,1)=length_max;
        
    case 3 %Near-wheel topology of motor
%%---------------------------Height----------------------------------------
        % Gear 1 z-position (axle center + driveshaft offset)
        vehicle.dimensions.p_gear_nax{wagontype}(1,3)=vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        
        % Gear 2 z-position (axle center + driveshaft offset)
        vehicle.dimensions.p_gear_nax{wagontype}(3,3)=vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        
        % Motor 1 z-position (axle center + driveshaft offset)
        vehicle.dimensions.p_motor_nax{wagontype}(1,3)=vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        
        % Motor 2 z-position (axle center + driveshaft offset)
        vehicle.dimensions.p_motor_nax{wagontype}(3,3)=vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        
        % Gearbox components x and z position
        for i = 1:size(vehicle.dimensions.comp_gear_nax_1{wagontype},1)
            vehicle.dimensions.comp_gear_nax_1{wagontype}(i,1) = vehicle.dimensions.p_offset_driveshaft(wagontype,1) + vehicle.dimensions.comp_gear_nax_1{wagontype}(i,1); %component gearbox 1 x position
            vehicle.dimensions.comp_gear_nax_1{wagontype}(i,3) = vehicle.dimensions.p_offset_driveshaft(wagontype,2) + vehicle.dimensions.comp_gear_nax_1{wagontype}(i,3); %component gearbox 1 z position
            vehicle.dimensions.comp_gear_nax_2{wagontype}(i,1) = vehicle.dimensions.p_offset_driveshaft(wagontype,1) + vehicle.dimensions.comp_gear_nax_2{wagontype}(i,1); %component gearbox 2 x position
            vehicle.dimensions.comp_gear_nax_2{wagontype}(i,3) = vehicle.dimensions.p_offset_driveshaft(wagontype,2) + vehicle.dimensions.comp_gear_nax_2{wagontype}(i,3); %component gearbox 2 z position
        end
        
        % temporary height max (without powerelectronics)
        height_max_temp=max([(vehicle.dimensions.p_gear_nax{wagontype}(1,3)+vehicle.dimensions.p_gear_nax{wagontype}(2,1)),(vehicle.dimensions.p_motor_nax{wagontype}(1,3)+vehicle.dimensions.p_motor_nax{wagontype}(2,1))]); %minimal height is either due to gear or motor

        % Powerelectronics z-pos. (height max + half powerelectronics height)
        vehicle.dimensions.p_powerel{wagontype}(1,3)=height_max_temp+0.5*vehicle.dimensions.p_powerel{wagontype}(2,3); % Position of power electronics is direct above motor
        
        height_min=min([(vehicle.dimensions.p_powerel{wagontype}(1,3)-0.5*vehicle.dimensions.p_powerel{wagontype}(2,3)),(vehicle.dimensions.p_gear_nax{wagontype}(1,3)-vehicle.dimensions.p_gear_nax{wagontype}(2,1)),(vehicle.dimensions.p_motor_nax{wagontype}(1,3)-vehicle.dimensions.p_motor_nax{wagontype}(2,1))]); %minimal height is either due to gear or motor
        % Redefine temp. height max (with powerelectronics)
        height_max=max([(vehicle.dimensions.p_powerel{wagontype}(1,3)+0.5*vehicle.dimensions.p_powerel{wagontype}(2,3)),(vehicle.dimensions.p_gear_nax{wagontype}(1,3)+vehicle.dimensions.p_gear_nax{wagontype}(2,1)),(vehicle.dimensions.p_motor_nax{wagontype}(1,3)+vehicle.dimensions.p_motor_nax{wagontype}(2,1))]); %maximal height is either due to gear or motor     
        vehicle.dimensions.p_drivetrain{wagontype}(2,3)=height_max-height_min;
        vehicle.dimensions.p_drivetrain{wagontype}(1,3)=height_max;
        
%%---------------------------Length----------------------------------------
        % Gear x-pos = center + driveshaft offset
        vehicle.dimensions.p_gear_nax{wagontype}(1,1)=vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        vehicle.dimensions.p_gear_nax{wagontype}(3,1)=vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        
        % Motor x-pos = center + driveshaft offset
        vehicle.dimensions.p_motor_nax{wagontype}(1,1)=vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        vehicle.dimensions.p_motor_nax{wagontype}(3,1)=vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        
        % Powerelectronics = motor x-pos
        vehicle.dimensions.p_powerel{wagontype}(1,1)=vehicle.dimensions.p_motor_nax{wagontype}(1,1);
                
        length_min=min([(vehicle.dimensions.p_powerel{wagontype}(1,1)-0.5*vehicle.dimensions.p_powerel{wagontype}(2,1)),(vehicle.dimensions.p_gear_nax{wagontype}(1,1)-vehicle.dimensions.p_gear_nax{wagontype}(2,1)),(vehicle.dimensions.p_motor_nax{wagontype}(1,1)-vehicle.dimensions.p_motor_nax{wagontype}(2,1))]); %minimal length is either due to gear or motor
        length_max=max([(vehicle.dimensions.p_powerel{wagontype}(1,1)+0.5*vehicle.dimensions.p_powerel{wagontype}(2,1)),(vehicle.dimensions.p_gear_nax{wagontype}(1,1)+vehicle.dimensions.p_gear_nax{wagontype}(2,1)),(vehicle.dimensions.p_motor_nax{wagontype}(1,1)+vehicle.dimensions.p_motor_nax{wagontype}(2,1))]); %maximal length is either due to gear or motor
        vehicle.dimensions.p_drivetrain{wagontype}(2,1)=length_max-length_min;
        vehicle.dimensions.p_drivetrain{wagontype}(1,1)=length_max;
        
    case 4 %Coaxial Layshaft       
        %% Height (z-direction)
        % 1) Gearbox and Motor
        % 1a) Read gear z-position and add driveshaft offset (2nd and 3rd gear are on parallel axle)
        gear_z_2=vehicle.dimensions.p_gear_coax_ls{wagontype}(3,3);
        gear_z_3=vehicle.dimensions.p_gear_coax_ls{wagontype}(5,3);
        
        vehicle.dimensions.p_gear_coax_ls{wagontype}(1,3)=vehicle.dimensions.p_offset_driveshaft(wagontype,2); %1st stage on same sahft then 4th (0+axle_offset)
        vehicle.dimensions.p_gear_coax_ls{wagontype}(3,3)=gear_z_2+vehicle.dimensions.p_offset_driveshaft(wagontype,2); %2nd stage on same shaft then 3rd
        vehicle.dimensions.p_gear_coax_ls{wagontype}(5,3)=gear_z_3+vehicle.dimensions.p_offset_driveshaft(wagontype,2); %3rd stage on same shaft then 2nd
        vehicle.dimensions.p_gear_coax_ls{wagontype}(7,3)=vehicle.dimensions.p_offset_driveshaft(wagontype,2); %4th stage on same shaft then 1st (0+axle_offset)
        vehicle.dimensions.p_gear_coax_ls{wagontype}(9,3)=vehicle.dimensions.p_offset_driveshaft(wagontype,2);          %housing offset
        
        %Calculate updated center point z-position of Gear Auxiliary components
        for i = 1:2:size(vehicle.dimensions.p_gear_coax_ls_aux{wagontype},1)
        vehicle.dimensions.p_gear_coax_ls_aux{wagontype}(i,3) = vehicle.dimensions.p_gear_coax_ls_aux{wagontype}(i,3) + vehicle.dimensions.p_offset_driveshaft(wagontype,2);
        end
                
        % 1b) Read motor z-position (same as 1st gear stage)
        vehicle.dimensions.p_motor_coax{wagontype}(1,3)=round(vehicle.dimensions.p_gear_coax_ls{wagontype}(1,3));
        
        % 2) Read parameters from calculation of gearbox
        d_flange=vehicle.gearbox{wagontype}.ConstDim.d_flange;
        t_housing=vehicle.gearbox{wagontype}.ConstDim.t_housing;
        d_housing=vehicle.gearbox{wagontype}.ConstDim.d_housing;
        d_diff=vehicle.gearbox{wagontype}.gears_34.d_a4;
        l_gearbox=vehicle.gearbox{wagontype}.results.l_gearbox;
        h_gearbox=vehicle.gearbox{wagontype}.results.h_gearbox;
        t_gearbox=vehicle.gearbox{wagontype}.results.t_gearbox;
        d_wheelmotor=vehicle.gearbox{wagontype}.shafts.d_1_max;
        l_park=vehicle.gearbox{wagontype}.ConstDim.l_park;
        a_12=vehicle.gearbox{wagontype}.results.a_12;        
        
        % 3) Calculate housing
        % Calculate offset of housing middle point to a4 middle point
        dist_a4_to_housing=d_housing+t_housing+d_flange+d_diff/2; %distance between mid-point of a4 and housing wall
        r_housing=h_gearbox/2; %radius of curve at the end of gearbox
        
        diff_a4_to_housing= r_housing-dist_a4_to_housing; %difference of two middle points
        diff_a4_to_housing_x=cosd(psi)*diff_a4_to_housing; %x-part (for length)
        diff_a4_to_housing_z=sind(psi)*diff_a4_to_housing; %z-part
        
        vehicle.dimensions.p_gear_coax_ls{wagontype}(9,3)=diff_a4_to_housing_z;%offset between housing middle point and gear 
        
        % 4) Calculate powerelectronics and complete drivetrain measurements
        % 4a) Min. height of gear housing (mid point of lower housing circle - one radius (in z-direction))
        height_house_coax_ls_min=vehicle.dimensions.p_gear_coax_ls{wagontype}(1,3)+vehicle.dimensions.p_gear_coax_ls{wagontype}(9,3)...
            -0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(10,3);
        % 4b) Min. height of drivetrain (without powerelectronics)
        height_min=min(height_house_coax_ls_min,(vehicle.dimensions.p_motor_coax{wagontype}(1,3)-vehicle.dimensions.p_motor_coax{wagontype}(2,1))); %minimal height is either due to gear or motor
        % 4c) Max. height of gear housing (mid point of lower housing circle + length to upper circle + one radius (in z-direction)
        height_house_coax_ls_max=vehicle.dimensions.p_gear_coax_ls{wagontype}(1,3)+vehicle.dimensions.p_gear_coax_ls{wagontype}(9,3)...
            +(vehicle.dimensions.p_gear_coax_ls{wagontype}(10,1)-vehicle.dimensions.p_gear_coax_ls{wagontype}(10,3))*sind(psi)+0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(10,3);
        height_max_temp=max(height_house_coax_ls_max,(vehicle.dimensions.p_motor_coax{wagontype}(1,3)+vehicle.dimensions.p_motor_coax{wagontype}(2,1))); %maximal height is either due to gear or motor
        
        % 5) Powerelectronics z-pos. (max. height of drivertrain + half powerelectronics height)
        vehicle.dimensions.p_powerel{wagontype}(1,3)=height_max_temp+0.5*vehicle.dimensions.p_powerel{wagontype}(2,3); % Position of power electronics is direct above motor
        
        % 6) Dimensions of drivetrain
        % Max. height of drivetrain (with powerelectronics)
        height_max=vehicle.dimensions.p_powerel{wagontype}(1,3)+0.5*vehicle.dimensions.p_powerel{wagontype}(2,3); %new height max = power electronic
        vehicle.dimensions.p_drivetrain{wagontype}(2,3)=height_max-height_min;
        vehicle.dimensions.p_drivetrain{wagontype}(1,3)=height_max;
        
        
        %% Length (x-direction)
        % 1) Gearbox and Motor
        % 1a) Calculate Gear center point x-position (axle center + driveshaft offset)
        gear_x_1=vehicle.dimensions.p_gear_coax_ls{wagontype}(1,1);
        gear_x_2=vehicle.dimensions.p_gear_coax_ls{wagontype}(3,1);
        gear_x_3=vehicle.dimensions.p_gear_coax_ls{wagontype}(5,1);
        gear_x_4=vehicle.dimensions.p_gear_coax_ls{wagontype}(7,1);
        
        vehicle.dimensions.p_gear_coax_ls{wagontype}(1,1)=gear_x_1+vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        vehicle.dimensions.p_gear_coax_ls{wagontype}(3,1)=gear_x_2+vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        vehicle.dimensions.p_gear_coax_ls{wagontype}(5,1)=gear_x_3+vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        vehicle.dimensions.p_gear_coax_ls{wagontype}(7,1)=gear_x_4+vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        
        %Calculate updated center point x-position of Gear Auxiliary components
        for i = 1:2:size(vehicle.dimensions.p_gear_coax_ls_aux{wagontype},1)
            vehicle.dimensions.p_gear_coax_ls_aux{wagontype}(i,1) = vehicle.dimensions.p_gear_coax_ls_aux{wagontype}(i,1) + vehicle.dimensions.p_offset_driveshaft(wagontype,1);
        end
        
        % 1b) Motor x-pos = x-pos. 1st stage
        vehicle.dimensions.p_motor_coax{wagontype}(1,1)=round(vehicle.dimensions.p_gear_coax_ls{wagontype}(1,1));
        
        % 2) Gearbox Housing
        % Calculate offset between housing and a4 middle points        
        vehicle.dimensions.p_gear_coax_ls{wagontype}(9,1)=diff_a4_to_housing_x; %offset between housing middle point and gear        

        % 3) Powerelectronics = motor x-pos
        vehicle.dimensions.p_powerel{wagontype}(1,1)=vehicle.dimensions.p_motor_coax{wagontype}(1,1);
        
        % 4) Dimensions of drivetrain
        % 4a) Min. and max. length due gearbox housing
        if psi<90
            length_house_coax_ls_min=vehicle.dimensions.p_gear_coax_ls{wagontype}(5,1)+vehicle.dimensions.p_gear_coax_ls{wagontype}(9,1)...
                -0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(10,3);
            length_house_coax_ls_max=vehicle.dimensions.p_gear_coax_ls{wagontype}(5,1)+vehicle.dimensions.p_gear_coax_ls{wagontype}(9,1)...
            +(vehicle.dimensions.p_gear_coax_ls{wagontype}(10,1)-vehicle.dimensions.p_gear_coax_ls{wagontype}(10,3))*cosd(psi)+0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(10,3);
        elseif psi>90
            length_house_coax_ls_max=vehicle.dimensions.p_gear_coax_ls{wagontype}(5,1)+vehicle.dimensions.p_gear_coax_ls{wagontype}(9,1)...
                +0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(10,3);
            length_house_coax_ls_min=vehicle.dimensions.p_gear_coax_ls{wagontype}(5,1)+vehicle.dimensions.p_gear_coax_ls{wagontype}(9,1)...
            +(vehicle.dimensions.p_gear_coax_ls{wagontype}(10,1)-vehicle.dimensions.p_gear_coax_ls{wagontype}(10,3))*cosd(psi)-0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(10,3);
        elseif psi==90 %house is half height in both directions
            length_house_coax_ls_min=vehicle.dimensions.p_gear_coax_ls{wagontype}(5,1)+vehicle.dimensions.p_gear_coax_ls{wagontype}(9,1)-0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(10,3);
            length_house_coax_ls_max=vehicle.dimensions.p_gear_coax_ls{wagontype}(5,1)+vehicle.dimensions.p_gear_coax_ls{wagontype}(9,1)+0.5*vehicle.dimensions.p_gear_coax_ls{wagontype}(10,3);
        end
        % 4b) Min. and max. length due to gearbox housing, powerelectronics or motor
        % Maximal length due to powerelectronics, gearbox or motor
        length_min=min([(vehicle.dimensions.p_powerel{wagontype}(1,1)-0.5*vehicle.dimensions.p_powerel{wagontype}(2,1)),... %power electronics
            length_house_coax_ls_min,... %geaxbox
            (vehicle.dimensions.p_motor_coax{wagontype}(1,1)-vehicle.dimensions.p_motor_coax{wagontype}(2,1))]); %motor
        % Maximal length due to powerelectronics, gearbox or motor
        length_max=max([(vehicle.dimensions.p_powerel{wagontype}(1,1)+0.5*vehicle.dimensions.p_powerel{wagontype}(2,1)),... %power electronics
            length_house_coax_ls_max,... %geaxbox
            (vehicle.dimensions.p_motor_coax{wagontype}(1,1)+vehicle.dimensions.p_motor_coax{wagontype}(2,1))]); %motor
        
        % 5) Calculate values and write them into vehicle
        vehicle.dimensions.p_drivetrain{wagontype}(2,1)=length_max-length_min;
        vehicle.dimensions.p_drivetrain{wagontype}(1,1)=length_max;
        
end
