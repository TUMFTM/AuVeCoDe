function [vehicle] = Package_Interior_Calc(vehicle,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Felix Fahn (Technical University of Munich)
%-------------
% Created on: 26.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: %This function calculates the interior measurements needed for Package_Interior based on the seating configuration and it adjustment areas
% ------------
% Sources:        [1] Felix Fahn, “Innenraummodellierung von autonomen Elektrofahrzeugen” Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2021
%                 [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - veh:  struct with the vehicle parameters (Number of Seats (per row, total); Interior type (vis-a-vis, conv., ...); Measurements of interior)
%           - Par:  struct with fixed parameters
% ------------
% Output:   - veh:    updated struct with boundary surfaces of interior
% ------------


%% Implementation
% 1. Calculations
% 2. Case of three rows needed
% 3. Write values for boundary in vehicle struct

%% 1.) Calculations
% legroom = footlength + seat adjustment x + additional legroom
vehicle.interior.legroom(1)     = vehicle.Input.foot_length(1) + vehicle.Input.seat_adjustment_x(1) + vehicle.Input.legroom_additional(1);
vehicle.interior.legroom(2)     = vehicle.Input.foot_length(2) + vehicle.Input.seat_adjustment_x(2) + vehicle.Input.legroom_additional(2);   

% seat depth with backrest thickness
vehicle.interior.seat_br(1)     = vehicle.Input.seat_depth(1) + vehicle.Input.thickness_br(1);
vehicle.interior.seat_br(2)     = vehicle.Input.seat_depth(2) + vehicle.Input.thickness_br(2);

% maximum heigth of battery under the seat
if vehicle.Input.H30_Opt==1
    [vehicle] = Package_H30(vehicle,Parameters);    %Calculates the height if H30 Optimization is active
else
vehicle.interior.boundary_seat(1)= vehicle.Input.seat_height(1) - vehicle.Input.thickness_seat(1) - vehicle.Input.seat_adjustment_z(1);
vehicle.interior.boundary_seat(2)= vehicle.Input.seat_height(2) - vehicle.Input.thickness_seat(2) - vehicle.Input.seat_adjustment_z(2);
end

%Check infeasibilities
%Seat height is not allowed to be lower then x-adjustment+seatthickness
 if vehicle.interior.boundary_seat(1)<0 || vehicle.interior.boundary_seat(2)<0
    vehicle.Error=14;
    return
end

% Backrest length in x
vehicle.interior.int_x_br_min(1) = cosd(vehicle.Input.angle_br_min(1))*vehicle.Input.upperbody_length(1) ;
vehicle.interior.int_x_br_min(2) = cosd(vehicle.Input.angle_br_min(2))*vehicle.Input.upperbody_length(2) ;

vehicle.interior.int_x_br_max(1) = cosd(vehicle.Input.angle_br_max(1))*vehicle.Input.upperbody_length(1) ;
vehicle.interior.int_x_br_max(2) = cosd(vehicle.Input.angle_br_max(2))*vehicle.Input.upperbody_length(2) ;
% Backrest height in z
vehicle.interior.int_z_br_min(1) = sind(vehicle.Input.angle_br_min(1))*vehicle.Input.upperbody_length(1) ;
vehicle.interior.int_z_br_min(2) = sind(vehicle.Input.angle_br_min(2))*vehicle.Input.upperbody_length(2) ;

vehicle.interior.int_z_br_max(1) = sind(vehicle.Input.angle_br_max(1))*vehicle.Input.upperbody_length(1) ;
vehicle.interior.int_z_br_max(2) = sind(vehicle.Input.angle_br_max(2))*vehicle.Input.upperbody_length(2) ;

% Maximum Height of Interior = Backrest height in z + headroom + seat height (95.
% Percentil Person)
vehicle.interior.int_z_tot(1)=  vehicle.interior.int_z_br_max(1) + vehicle.Input.headroom(1) + vehicle.Input.seat_height(1);
vehicle.interior.int_z_tot(2)=  vehicle.interior.int_z_br_max(2) + vehicle.Input.headroom(2) + vehicle.Input.seat_height(2);

% Calculating Interior Width
n_seat=vehicle.Input.n_seat;
vehicle.interior.int_y_tot=zeros(1,2); % initialize width of front and back row

for n=1:2
    if n_seat(n)==1 % 1 passengers on seatrow
        %vehicle.interior.int_y_tot(n)=1*vehicle.Input.int_y_seat(n)+vehicle.Input.int_y_sdr(n)*2; %total width of interior with 1 seats per row       
        if vehicle.Input.single_ar_outside(n) ==1
           vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + vehicle.Input.backrest_width(n) + 2*vehicle.Input.armrest_width_single(n);
        else
           vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + vehicle.Input.backrest_width(n);
        end
                
    elseif n_seat(n)==2 % 2 passengers on seatrow
        %vehicle.interior.int_y_tot(n)=2*vehicle.Input.int_y_seat(n)+vehicle.Input.int_y_sdr(n)*4+ vehicle.Input.int_y_ar(n); %total width of interior with 2 seats per row        
        if vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==1 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 2*vehicle.Input.backrest_width(n) + 4*vehicle.Input.armrest_width_single(n) + vehicle.Input.seatgap(n);
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==1 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 2*vehicle.Input.backrest_width(n) + 2*vehicle.Input.armrest_width_single(n) + vehicle.Input.seatgap(n);
        elseif vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 1
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 2*vehicle.Input.backrest_width(n) + 2*vehicle.Input.armrest_width_single(n) + vehicle.Input.armrest_width_double(n) + vehicle.Input.seatgap(n);
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 1
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 2*vehicle.Input.backrest_width(n) + vehicle.Input.armrest_width_double(n) + vehicle.Input.seatgap(n);
        elseif vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 0    
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 2*vehicle.Input.backrest_width(n) + 2*vehicle.Input.armrest_width_single(n) + vehicle.Input.seatgap(n);
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 0    
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 2*vehicle.Input.backrest_width(n) + vehicle.Input.seatgap(n);          
        end
        
        
    elseif n_seat(n)==3 % 3 passengers on seatrow
        %vehicle.interior.int_y_tot(n)=3*vehicle.Input.int_y_seat(n)+vehicle.Input.int_y_sdr(n)*6+ 2*vehicle.Input.int_y_ar(n); %total width of interior with 3 seats per row
        if vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==1 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 3*vehicle.Input.backrest_width(n) + 6*vehicle.Input.armrest_width_single(n) + 2*vehicle.Input.seatgap(n);
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==1 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 3*vehicle.Input.backrest_width(n) + 4*vehicle.Input.armrest_width_single(n) + 2*vehicle.Input.seatgap(n);
        elseif vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 1
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 3*vehicle.Input.backrest_width(n) + 2*vehicle.Input.armrest_width_single(n) + 2*vehicle.Input.armrest_width_double(n) + 2*vehicle.Input.seatgap(n);
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 1
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 3*vehicle.Input.backrest_width(n) + 2*vehicle.Input.armrest_width_double(n) + 2*vehicle.Input.seatgap(n);
        elseif vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 3*vehicle.Input.backrest_width(n) + 2*vehicle.Input.armrest_width_single(n) + 2*vehicle.Input.seatgap(n);
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.int_y_tot(n) = 2*vehicle.Input.wallgap(n) + 3*vehicle.Input.backrest_width(n) + 2*vehicle.Input.seatgap(n);
            
        end
    end
end

vehicle.interior.int_y_front=vehicle.interior.int_y_tot(1); % total interior width front row
vehicle.interior.int_y_rear=vehicle.interior.int_y_tot(2); % total interior width rear row
vehicle.interior.int_y_tot=max(vehicle.interior.int_y_tot); % write interior width to object vehicle

%% 2.) Case of three rows needed
if vehicle.Input.int_3rows==1
    if strcmp(vehicle.Input.int_3rows_type,'3con')          %3 rows are in conventional direction
        length=vehicle.interior.legroom(2)+vehicle.interior.int_x_br_min(2)+vehicle.interior.seat_br(2); %length of second row
        vehicle.interior.offset_3rows=length;
        vehicle.interior.legroom(2)=vehicle.interior.legroom(2)+length; %Add length to legroom
    elseif strcmp(vehicle.Input.int_3rows_type,'3convav')   %1st row is conventional, 2nd and 3rd row is visavis
        length=vehicle.interior.legroom(2)+vehicle.interior.int_x_br_min(2)+vehicle.interior.seat_br(2); %length of second row
        vehicle.interior.offset_3rows=length;
        vehicle.interior.legroom(2)=vehicle.interior.legroom(2)+length; %Add length to legroom
    elseif strcmp(vehicle.Input.int_3rows_type,'3vavcon')   %1st and 2nd row are conventional, 3rd conventional
        length=vehicle.interior.legroom(1)+vehicle.interior.int_x_br_min(1)+vehicle.interior.seat_br(1); %length of second row
        vehicle.interior.offset_3rows=length;
        vehicle.interior.legroom(1)=vehicle.interior.legroom(1)+length; %Add length to legroom
    end
end

%% 3.) Write values in vehicle struct for boundary calculation in Package_Interior
vehicle.Input.int_z_leg         = vehicle.interior.boundary_seat;
vehicle.interior.int_z_br       = vehicle.interior.int_z_br_min;
vehicle.interior.int_z_tot      = vehicle.interior.int_z_tot;
vehicle.interior.int_x_br       = vehicle.interior.int_x_br_min;
vehicle.Input.int_x_leg         = vehicle.interior.legroom;
vehicle.Input.int_x_seat        = vehicle.interior.seat_br;
vehicle.Input.int_y_seat        = vehicle.Input.backrest_width;
vehicle.Input.int_x_overlap     = vehicle.Input.leg_overlap;
vehicle.Input.int_z_leg         = vehicle.interior.boundary_seat;
end