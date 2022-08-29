function PlotHuman(vehicle,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 11.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function plots and scales a human accordingly to the positions of seats
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
%           - Parameters: struct with input and constant values
% ------------
% Output:   - Plotted humans
% ------------

%% Implementation
% 1) Calculate y-Position of each manikin
% 2) Calculate Values for positioning of the components
% 3) Shift position and Plot Manikin
%% Inputs

load('human.mat','armstop','head','legslow','legstop','torso')
%a) Read surfaces
int_boundary_f_new=vehicle.interior.int_boundary_f_new; % initialize interior front boundary
int_boundary_r_new=vehicle.interior.int_boundary_r_new; % initialize interior rear boundary
a_1=vehicle.interior.a_1; % Side distance to interior front
a_2=vehicle.interior.a_2; % Side distance to interior rear
humancolor=[20, 20, 200]./255;


%% 1) Calculate y-Position of each manikin
% Differentiation, if there is 1, 2 or 3 seat in a seatrow
width_seat = vehicle.Input.backrest_width;
a_pos=[a_1 a_2];
n_seat=vehicle.Input.n_seat;
vehicle.interior.y_position=zeros(2,3); % initialize width of front and back row

for n=1:2
    
    if n_seat(n)==1 % 1 passengers on seatrow     
        if vehicle.Input.single_ar_outside(n) ==1
           vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + 0.5*width_seat(n) + a_pos(n);
        else
           vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + 0.5*width_seat(n) + a_pos(n);
        end
        
    elseif n_seat(n)==2 % 2 passengers on seatrow  
        if vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==1 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + 3*vehicle.Input.armrest_width_single(n) + vehicle.Input.backrest_width(n) + vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==1 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + 2*vehicle.Input.armrest_width_single(n) + vehicle.Input.backrest_width(n) + vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);                        
        elseif vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 1
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + vehicle.Input.backrest_width(n) + vehicle.Input.seatgap(n) + vehicle.Input.armrest_width_double(n) + 0.5*width_seat(n) + a_pos(n);                                   
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 1
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + vehicle.Input.backrest_width(n) + vehicle.Input.seatgap(n) + vehicle.Input.armrest_width_double(n) + 0.5*width_seat(n) + a_pos(n);      
        elseif vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 0    
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + vehicle.Input.backrest_width(n) + vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);       
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 0    
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + vehicle.Input.backrest_width(n) + vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);          
        end
               
    elseif n_seat(n)==3 % 3 passengers on seatrow
        if vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==1 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + 3*vehicle.Input.armrest_width_single(n) + vehicle.Input.backrest_width(n)   + vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);  
            vehicle.interior.y_position(n,3) = vehicle.Input.wallgap(n) + 5*vehicle.Input.armrest_width_single(n) + 2*vehicle.Input.backrest_width(n) + 2*vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);                                
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==1 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + 2*vehicle.Input.armrest_width_single(n) + vehicle.Input.backrest_width(n)   + vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);  
            vehicle.interior.y_position(n,3) = vehicle.Input.wallgap(n) + 4*vehicle.Input.armrest_width_single(n) + 2*vehicle.Input.backrest_width(n) + 2*vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);                        
        elseif vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 1
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + vehicle.Input.backrest_width(n)   + vehicle.Input.seatgap(n) + vehicle.Input.armrest_width_double(n) + 0.5*width_seat(n) + a_pos(n);  
            vehicle.interior.y_position(n,3) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + 2*vehicle.Input.backrest_width(n) + 2*vehicle.Input.seatgap(n) + 2*vehicle.Input.armrest_width_double(n) + 0.5*width_seat(n) + a_pos(n);            
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 1
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + vehicle.Input.backrest_width(n)   + vehicle.Input.seatgap(n) + vehicle.Input.armrest_width_double(n) + 0.5*width_seat(n) + a_pos(n);  
            vehicle.interior.y_position(n,3) = vehicle.Input.wallgap(n) + 2*vehicle.Input.backrest_width(n) + 2*vehicle.Input.seatgap(n) + 2*vehicle.Input.armrest_width_double(n) + 0.5*width_seat(n) + a_pos(n);           
        elseif vehicle.Input.single_ar_outside(n) ==1 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + vehicle.Input.backrest_width(n)   + vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);  
            vehicle.interior.y_position(n,3) = vehicle.Input.wallgap(n) + vehicle.Input.armrest_width_single(n) + 2*vehicle.Input.backrest_width(n) + 2*vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);      
        elseif vehicle.Input.single_ar_outside(n) ==0 && vehicle.Input.single_ar_between(n) ==0 && vehicle.Input.double_ar(n) == 0
            vehicle.interior.y_position(n,1) = vehicle.Input.wallgap(n) + 0.5*width_seat(n) + a_pos(n);
            vehicle.interior.y_position(n,2) = vehicle.Input.wallgap(n) + vehicle.Input.backrest_width(n)   + vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);  
            vehicle.interior.y_position(n,3) = vehicle.Input.wallgap(n) + 2*vehicle.Input.backrest_width(n) + 2*vehicle.Input.seatgap(n) + 0.5*width_seat(n) + a_pos(n);            
        end
    end
end



%% 2) Calculate Values for positioning of the components
% Calling functions depending on seating layout
switch vehicle.Input.int_type %(vav=vis-a-vis, btb=back-to-back, con=conventional, sr=singlerow)
    case 'vav'
        [a_pos,eu_1,eu_1_alpha,eu_2,eu_2_alpha,x_pos_seat,z_pos_seat,x_pos_br,z_pos_br,Lz_br] = PlotInterior_vav(vehicle,int_boundary_f_new,int_boundary_r_new,a_1,a_2);
        orientation=[1 -1]; % 1 = backwards, -1 = forwards
        offset_x = [100 vehicle.Input.seat_depth(2)-100+vehicle.Input.seat_adjustment_x(2)];
    case 'btb'
        [a_pos,eu_1,eu_1_alpha,eu_2,eu_2_alpha,x_pos_seat,z_pos_seat,x_pos_br,z_pos_br,Lz_br] = PlotInterior_btb(vehicle,int_boundary_f_new,int_boundary_r_new,a_1,a_2);
        orientation=[-1 1];
        offset_x = [vehicle.Input.seat_depth(1)-100+vehicle.Input.seat_adjustment_x(1) 100];
    case 'con'
        [a_pos,eu_1,eu_1_alpha,eu_2,eu_2_alpha,x_pos_seat,z_pos_seat,x_pos_br,z_pos_br,Lz_br] = PlotInterior_con(vehicle,int_boundary_f_new,int_boundary_r_new,a_1,a_2);
        orientation=[-1 -1];
        offset_x = [vehicle.Input.seat_depth(1)-100+vehicle.Input.seat_adjustment_x(1) vehicle.Input.seat_depth(2)-100+vehicle.Input.seat_adjustment_x(2)];
    case 'sr'
        [a_pos,eu_1,eu_1_alpha,x_pos_seat,z_pos_seat,x_pos_br,z_pos_br,Lz_br]                 = PlotInterior_sr(vehicle,int_boundary_f_new,int_boundary_r_new,a_1,a_2);
        orientation=[-1 -1];
        offset_x = [vehicle.Input.seat_depth(1)-100+vehicle.Input.seat_adjustment_x(1) 0];
end


%% 3) Shift position and Plot Manikin

%Read in angles
angle = vehicle.Input.angle_br_max; %max. backrest angle in °
angle_init = 90; %angle of mannquin designed in Catia

if isequal(vehicle.Input.int_type ,'sr')
    seatrow=1;
else 
    seatrow=2; %always two rows, except single row (sr)
end

% Read and calculate important measurements
int_height      =   vehicle.Input.int_height;   %interior height in mm
legroom         =   vehicle.interior.legroom;   %legroom in mm
x_seat          =   vehicle.Input.seat_depth;   %depth of seat in mm
z_legstop_lb    =   min(legstop.points(:,3));   %Lower boundary of thigh
x_legstop_lb    =   min(legstop.points(:,1));   %Lower boundary of thigh
x_legstop_ub    =   max(legstop.points(:,1));   %Upper boundary of thigh
x_legstop       =   x_legstop_ub-x_legstop_lb;  %Length of thigh
z_legslow_lb    =   min(legslow.points(:,3));   %Lower boundary of thigh
z_legslow       =   abs(z_legslow_lb);          %Distance between foot and turning point
x_legslow_lb    =   min(legslow.points(:,1));   %Lower boundary of thigh
x_legslow_ub    =   max(legslow.points(:,1));   %Upper boundary of thigh
offset_z        =   abs(z_legstop_lb); %Offset of coordination system in Catia and outer measurement and overlap due to sinking in

z_legspace_max  =   z_pos_br - int_height;      %max. legspace: upper pos. of seat cushion - interior height
z_leg_min       =   z_legslow+z_legstop_lb;    %min. length in z of legs (length of lower leg - turning point distance)
x_legspace_max  =   legroom; %max. legroom from end of cushion to end of legroom
x_leg_min       =   max(x_legstop_ub,x_legslow_ub)-min(x_legstop_lb,x_legslow_lb)-x_seat; %minimal needed space for legs
alpha_leg       =   zeros(1,seatrow);
betta_leg       =   zeros(1,seatrow);
gamma_leg       =   zeros(1,seatrow);
complex_calc    =   zeros(1,seatrow);
offset          =   zeros(2,seatrow);
%Calculation of leg kinematics
x_tp_r              =   Par.dimensions.EX.mannequin_x_tp_r;                 %Distance between turning point of torso and backrest
x_tp_f              =   Par.dimensions.EX.mannequin_x_tp_f;                 %Distance between tuerning point of upper and lower leg
z_turning_point     =   z_legspace_max + abs(z_legstop_lb);                 %distance between leg turning point and ground
x_length_tot        =   legroom + x_seat-x_tp_r;                            %length from start of cushion till end of legroom
x_length_legstop    =   x_legstop_ub-x_tp_f;                                %Distance between origin of human and turning point of lower and upper leg (62mm from CATIA)
x_tp_ff             =   Par.dimensions.EX.mannequin_x_tp_foot_f;            %distance between turning point of lower and upper leg and foot front tip
x_tp_fr             =   Par.dimensions.EX.mannequin_x_tp_foot_r;            %distance between turning point of lower and upper leg and foot rear end

% Check if enough space is available for standard posture
for n_row=1:seatrow
    for i=1:2 %Calculation, correction of foot and recalculation
        alpha=acosd((z_turning_point(n_row))/abs(z_legslow_lb)); %Needed angle to stretch leg to the front
        x_leg_max = sind(alpha)*z_legslow+cosd(alpha)*x_tp_ff; %Calculate more needed space due to streching the leg to the front
        if z_legspace_max(n_row) >= z_leg_min && x_legspace_max(n_row) >= x_leg_min(n_row)
            betta_leg(n_row)=0;
            gamma_leg(n_row)=0;            
        else           
            if x_leg_max<x_legspace_max(n_row) %leg is only stretched in x-direction
                alpha_leg(n_row)=alpha;
                betta_leg(n_row)=0;
                gamma_leg(n_row)=0;
            else %leg has to be angled since length and height are limiting
                complex_calc(n_row)=1;               
                betta_leg(n_row)=acosd((z_turning_point(n_row)^2+x_length_tot(n_row)^2-x_length_legstop^2-abs(z_legslow_lb^2))/(2*x_length_legstop*abs(z_legslow_lb))); %calculation of first needeed angle (between upper and lower leg)
                gamma_leg(n_row)=atand(z_turning_point(n_row)/x_length_tot(n_row))-asind(sind(betta_leg(n_row))*abs(z_legslow_lb)/(sqrt(z_turning_point(n_row)^2+x_length_tot(n_row)^2))); %calculation of second needed angle
                betta_leg(n_row)=90-betta_leg(n_row);
                
                %calculate offset due to rotation of turning point
                offset(n_row,1)=cosd(-gamma_leg(n_row))*x_legstop-x_legstop;
                offset(n_row,2)=sind(-gamma_leg(n_row))*x_legstop;
            end            
        end
        if i==1
            if complex_calc(n_row)==1 %angle has to be calculated
                foot_angle=gamma_leg(n_row)-betta_leg(n_row); %angle is turning both top and low leg
            else %angle = alpha
                foot_angle=alpha; %angle is turning of low leg
            end
            x_foot=cosd(foot_angle)*x_tp_ff; %additional required space in x due to foot angle
            z_foot=sind(foot_angle)*-x_tp_fr;  %additional required space in z due to foot angle
        end
        z_turning_point(n_row)  =   z_turning_point(n_row)-abs(z_foot); %reduce max. space with add. req. space
        x_length_tot(n_row)     =   x_length_tot(n_row)-abs(x_foot);    %reduce max. space with add. req. space 
    end
end

for n=1:seatrow 

    if ~(angle(n) ==0)
        angle(n) = deg2rad(angle(n)-angle_init);
    end   
    Rotmatrix=[cos(angle(n)),0,sin(angle(n));0,1,0;-sin(angle(n)),0,cos(angle(n))];
    Rotmatrix_leg_alpha=[cosd(-alpha_leg(n)),0,sind(-alpha_leg(n));0,1,0;-sind(-alpha_leg(n)),0,cosd(-alpha_leg(n))];
    Rotmatrix_leg_betta=[cosd(-betta_leg(n)),0,sind(-betta_leg(n));0,1,0;-sind(-betta_leg(n)),0,cosd(-betta_leg(n))];
    Rotmatrix_leg_gamma=[cosd(gamma_leg(n)),0,sind(gamma_leg(n));0,1,0;-sind(gamma_leg(n)),0,cosd(gamma_leg(n))];
    
    if n_seat(n) == 1 % one seat per seatrow
        %Load and rotate passenger
        [torso,head,legstop,legslow,armstop]=load_passenger(Rotmatrix,Rotmatrix_leg_alpha,Rotmatrix_leg_betta,Rotmatrix_leg_gamma,complex_calc(n),offset(n,:));
        
        % Positioning of the Body Parts
        torso.points(:,1)=orientation(n)*torso.points(:,1)+x_pos_seat(n)+offset_x(n);
        torso.points(:,2)=torso.points(:,2)+vehicle.interior.y_position(n,1);
        torso.points(:,3)=torso.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n); 
        head.points(:,1)=orientation(n)*head.points(:,1)+x_pos_seat(n)+offset_x(n);
        head.points(:,2)=head.points(:,2)+vehicle.interior.y_position(n,1);
        head.points(:,3)=head.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        legstop.points(:,1)=orientation(n)*legstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        legstop.points(:,2)=legstop.points(:,2)+vehicle.interior.y_position(n,1);
        legstop.points(:,3)=legstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        legslow.points(:,1)=orientation(n)*legslow.points(:,1)+x_pos_seat(n)+offset_x(n);
        legslow.points(:,2)=legslow.points(:,2)+vehicle.interior.y_position(n,1);
        legslow.points(:,3)=legslow.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        armstop.points(:,1)=orientation(n)*armstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        armstop.points(:,2)=armstop.points(:,2)+vehicle.interior.y_position(n,1);
        armstop.points(:,3)=armstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        
        % Plot Human
        TH=triangulation(head.cv,head.points);
        trisurf(TH,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(torso.cv,torso.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legstop.cv,legstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legslow.cv,legslow.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(armstop.cv,armstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        
    elseif n_seat(n) == 2 % two seats per seatrow
        %Load and rotate passenger
        [torso,head,legstop,legslow,armstop]=load_passenger(Rotmatrix,Rotmatrix_leg_alpha,Rotmatrix_leg_betta,Rotmatrix_leg_gamma,complex_calc(n),offset(n,:));
        
        % Positioning of the Body Parts
        torso.points(:,1)=orientation(n)*torso.points(:,1)+x_pos_seat(n)+offset_x(n);
        torso.points(:,2)=torso.points(:,2)+vehicle.interior.y_position(n,1);
        torso.points(:,3)=torso.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        head.points(:,1)=orientation(n)*head.points(:,1)+x_pos_seat(n)+offset_x(n);
        head.points(:,2)=head.points(:,2)+vehicle.interior.y_position(n,1);
        head.points(:,3)=head.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        armstop.points(:,1)=orientation(n)*armstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        armstop.points(:,2)=armstop.points(:,2)+vehicle.interior.y_position(n,1);
        armstop.points(:,3)=armstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);      
        legstop.points(:,1)=orientation(n)*legstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        legstop.points(:,2)=legstop.points(:,2)+vehicle.interior.y_position(n,1);
        legstop.points(:,3)=legstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        legslow.points(:,1)=orientation(n)*legslow.points(:,1)+x_pos_seat(n)+offset_x(n);
        legslow.points(:,2)=legslow.points(:,2)+vehicle.interior.y_position(n,1);
        legslow.points(:,3)=legslow.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        
        % Plot Human
        TH=triangulation(head.cv,head.points);
        trisurf(TH,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(torso.cv,torso.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legstop.cv,legstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legslow.cv,legslow.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(armstop.cv,armstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);      
 
        %Load and rotate passenger
        [torso,head,legstop,legslow,armstop]=load_passenger(Rotmatrix,Rotmatrix_leg_alpha,Rotmatrix_leg_betta,Rotmatrix_leg_gamma,complex_calc(n),offset(n,:));
        
        % Positioning of the Body Parts       
        torso.points(:,1)=orientation(n)*torso.points(:,1)+x_pos_seat(n)+offset_x(n);
        torso.points(:,2)=torso.points(:,2)+vehicle.interior.y_position(n,2);
        torso.points(:,3)=torso.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n); 
        head.points(:,1)=orientation(n)*head.points(:,1)+x_pos_seat(n)+offset_x(n);
        head.points(:,2)=head.points(:,2)+vehicle.interior.y_position(n,2);
        head.points(:,3)=head.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        legstop.points(:,1)=orientation(n)*legstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        legstop.points(:,2)=legstop.points(:,2)+vehicle.interior.y_position(n,2);
        legstop.points(:,3)=legstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        legslow.points(:,1)=orientation(n)*legslow.points(:,1)+x_pos_seat(n)+offset_x(n);
        legslow.points(:,2)=legslow.points(:,2)+vehicle.interior.y_position(n,2);
        legslow.points(:,3)=legslow.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        armstop.points(:,1)=orientation(n)*armstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        armstop.points(:,2)=armstop.points(:,2)+vehicle.interior.y_position(n,2);
        armstop.points(:,3)=armstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        
        % Plot Human
        TH=triangulation(head.cv,head.points);
        trisurf(TH,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(torso.cv,torso.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legstop.cv,legstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legslow.cv,legslow.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(armstop.cv,armstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        
    elseif n_seat(n) == 3 % three seats per seatrow
        %Load and rotate passenger
        [torso,head,legstop,legslow,armstop]=load_passenger(Rotmatrix,Rotmatrix_leg_alpha,Rotmatrix_leg_betta,Rotmatrix_leg_gamma,complex_calc(n),offset(n,:));
        
        % Positioning of the Body Parts
        torso.points(:,1)=orientation(n)*torso.points(:,1)+x_pos_seat(n)+offset_x(n);
        torso.points(:,2)=torso.points(:,2)+vehicle.interior.y_position(n,1);
        torso.points(:,3)=torso.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n); 
        head.points(:,1)=orientation(n)*head.points(:,1)+x_pos_seat(n)+offset_x(n);
        head.points(:,2)=head.points(:,2)+vehicle.interior.y_position(n,1);
        head.points(:,3)=head.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        legstop.points(:,1)=orientation(n)*legstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        legstop.points(:,2)=legstop.points(:,2)+vehicle.interior.y_position(n,1);
        legstop.points(:,3)=legstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        legslow.points(:,1)=orientation(n)*legslow.points(:,1)+x_pos_seat(n)+offset_x(n);
        legslow.points(:,2)=legslow.points(:,2)+vehicle.interior.y_position(n,1);
        legslow.points(:,3)=legslow.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        armstop.points(:,1)=orientation(n)*armstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        armstop.points(:,2)=armstop.points(:,2)+vehicle.interior.y_position(n,1);
        armstop.points(:,3)=armstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        
        % Plot Human
        TH=triangulation(head.cv,head.points);
        trisurf(TH,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(torso.cv,torso.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legstop.cv,legstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legslow.cv,legslow.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(armstop.cv,armstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);      
        
        %Load and rotate passenger
        [torso,head,legstop,legslow,armstop]=load_passenger(Rotmatrix,Rotmatrix_leg_alpha,Rotmatrix_leg_betta,Rotmatrix_leg_gamma,complex_calc(n),offset(n,:));
               
        % Positioning of the Body Parts      
        torso.points(:,1)=orientation(n)*torso.points(:,1)+x_pos_seat(n)+offset_x(n);
        torso.points(:,2)=torso.points(:,2)+vehicle.interior.y_position(n,2);
        torso.points(:,3)=torso.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n); 
        head.points(:,1)=orientation(n)*head.points(:,1)+x_pos_seat(n)+offset_x(n);
        head.points(:,2)=head.points(:,2)+vehicle.interior.y_position(n,2);
        head.points(:,3)=head.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        legstop.points(:,1)=orientation(n)*legstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        legstop.points(:,2)=legstop.points(:,2)+vehicle.interior.y_position(n,2);
        legstop.points(:,3)=legstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        legslow.points(:,1)=orientation(n)*legslow.points(:,1)+x_pos_seat(n)+offset_x(n);
        legslow.points(:,2)=legslow.points(:,2)+vehicle.interior.y_position(n,2);
        legslow.points(:,3)=legslow.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        armstop.points(:,1)=orientation(n)*armstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        armstop.points(:,2)=armstop.points(:,2)+vehicle.interior.y_position(n,2);
        armstop.points(:,3)=armstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        
        % Plot Human
        TH=triangulation(head.cv,head.points);
        trisurf(TH,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(torso.cv,torso.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legstop.cv,legstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legslow.cv,legslow.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(armstop.cv,armstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        
        %Load and rotate passenger
        [torso,head,legstop,legslow,armstop]=load_passenger(Rotmatrix,Rotmatrix_leg_alpha,Rotmatrix_leg_betta,Rotmatrix_leg_gamma,complex_calc(n),offset(n,:));
        
        % Positioning of the Body Parts
        torso.points(:,1)=orientation(n)*torso.points(:,1)+x_pos_seat(n)+offset_x(n);
        torso.points(:,2)=torso.points(:,2)+vehicle.interior.y_position(n,3);
        torso.points(:,3)=torso.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n); 
        head.points(:,1)=orientation(n)*head.points(:,1)+x_pos_seat(n)+offset_x(n);
        head.points(:,2)=head.points(:,2)+vehicle.interior.y_position(n,3);
        head.points(:,3)=head.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        legstop.points(:,1)=orientation(n)*legstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        legstop.points(:,2)=legstop.points(:,2)+vehicle.interior.y_position(n,3);
        legstop.points(:,3)=legstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        legslow.points(:,1)=orientation(n)*legslow.points(:,1)+x_pos_seat(n)+offset_x(n);
        legslow.points(:,2)=legslow.points(:,2)+vehicle.interior.y_position(n,3);
        legslow.points(:,3)=legslow.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        armstop.points(:,1)=orientation(n)*armstop.points(:,1)+x_pos_seat(n)+offset_x(n);
        armstop.points(:,2)=armstop.points(:,2)+vehicle.interior.y_position(n,3);
        armstop.points(:,3)=armstop.points(:,3)+z_pos_seat(n)+offset_z+vehicle.Input.thickness_seat(n);
        
        % Plot Human
        TH=triangulation(head.cv,head.points);
        trisurf(TH,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(torso.cv,torso.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legstop.cv,legstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(legslow.cv,legslow.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
        TT=triangulation(armstop.cv,armstop.points);
        trisurf(TT,'LineStyle','none','FaceColor',humancolor,'FaceAlpha',0.8);
    end

end

end
function [torso,head,legstop,legslow,armstop]=load_passenger(Rotmatrix,Rotmatrix_leg_alpha,Rotmatrix_leg_betta,Rotmatrix_leg_gamma,complex_calc,offset)
        load('human.mat','armstop','head','legslow','legstop','torso')
        % Rotation of the Upper Body Parts
        rot = (Rotmatrix*[torso.points(:,1)';torso.points(:,2)';torso.points(:,3)'])';
            torso.points(:,1) = rot(:,1);
            torso.points(:,2) = rot(:,2);
            torso.points(:,3) = rot(:,3);
            rot = (Rotmatrix*[head.points(:,1)';head.points(:,2)';head.points(:,3)'])';
            head.points(:,1) = rot(:,1);
            head.points(:,2) = rot(:,2);
            head.points(:,3) = rot(:,3);
            rot = (Rotmatrix*[armstop.points(:,1)';armstop.points(:,2)';armstop.points(:,3)'])';
            armstop.points(:,1) = rot(:,1);
            armstop.points(:,2) = rot(:,2);
            armstop.points(:,3) = rot(:,3);
        if complex_calc==0            
            %First move leg points to the right turning point, then turn, then move back
            legslow.points(:,1)=legslow.points(:,1)-470;
            legslow.points(:,2)=legslow.points(:,2);
            legslow.points(:,3)=legslow.points(:,3)-0;
            rot = (Rotmatrix_leg_alpha*[legslow.points(:,1)';legslow.points(:,2)';legslow.points(:,3)'])';
            legslow.points(:,1)=rot(:,1);
            legslow.points(:,2)=rot(:,2);
            legslow.points(:,3)=rot(:,3);
            legslow.points(:,1)=legslow.points(:,1)+470;
            legslow.points(:,2)=legslow.points(:,2);
            legslow.points(:,3)=legslow.points(:,3)+0;
        else
            %First move leg points to the right turning point, then turn, then move back
            rot = (Rotmatrix_leg_gamma*[legslow.points(:,1)';legslow.points(:,2)';legslow.points(:,3)'])';
            legslow.points(:,1)=rot(:,1);
            legslow.points(:,2)=rot(:,2);
            legslow.points(:,3)=rot(:,3);
            rot = (Rotmatrix_leg_gamma*[legstop.points(:,1)';legstop.points(:,2)';legstop.points(:,3)'])';
            legstop.points(:,1)=rot(:,1);
            legstop.points(:,2)=rot(:,2);
            legstop.points(:,3)=rot(:,3);
            %Calculate movement of turning point between legs
            legslow.points(:,1)=legslow.points(:,1)-470-offset(1);
            legslow.points(:,2)=legslow.points(:,2);
            legslow.points(:,3)=legslow.points(:,3)-0-offset(2);
            rot = (Rotmatrix_leg_betta*[legslow.points(:,1)';legslow.points(:,2)';legslow.points(:,3)'])';
            legslow.points(:,1)=rot(:,1);
            legslow.points(:,2)=rot(:,2);
            legslow.points(:,3)=rot(:,3);
            legslow.points(:,1)=legslow.points(:,1)+470+offset(1);
            legslow.points(:,2)=legslow.points(:,2);
            legslow.points(:,3)=legslow.points(:,3)+0+offset(2);
        end
end
