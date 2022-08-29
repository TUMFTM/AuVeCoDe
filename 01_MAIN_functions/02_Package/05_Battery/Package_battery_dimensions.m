function [v] = Package_battery_dimensions(v,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow (Technical University of Munich)
%-------------
% Created on: 01.05.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the avaiable space in the underfloor, considering
%               the available space calculated is always the space for the cells, so excluding the
%               housing etc.
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct including the data of the vehicle
%           - Parameters: struct with input and constant values
% ------------
% Output:   - vehicle: struct including the data of the vehicle
% ------------

%% Implementation
% 1.) Read in Parameters
% 2.) Calculate battery dimensions
% 3.) Assisgn Outputs
%   a) Assign Output Table
%   b) Assign outputs in vehicle

%% 1.) Read in Parameters
p_int_tolerance      = Parameters.package.p_int_tolerance; %General tolerance for minimum distance between components in mm
z_batt_top_cover        = v.dimensions.CZ.batt_top_cover; %Thickness of battery top cover
z_batt_bottom_cover     = v.dimensions.CZ. batt_bottom_cover; %Thickness of battery bottom cover
z_pas_comp_floor        = v.dimensions.EZ.passenger_compartement_floor; %Thickness of compartment floor
EZ_free_space_mod2cover = Parameters.dimensions.EZ.free_space_module_to_top_cover;  %Distance between module top and top cover bottom along the vertical (Z) direction in mm
int_boundary_f_new      = v.interior.int_boundary_f_new; %interior boundary surface of front wagon
int_boundary_r_new      = v.interior.int_boundary_r_new; %interior boundary surface rear
int_y_front             = max(v.interior.int_y_front,v.interior.int_y_rear); %width of second level is max interior boundary surface in mm
int_y_rear              = max(v.interior.int_y_front,v.interior.int_y_rear); %width of second level is max interior bundary surface in mm
int_x_overlap           = v.Input.int_x_overlap; %overlap of legroom in mm
int_type                = v.Input.int_type; %Interior type
width_avail             = v.dimensions.width_avail; %available width between axles [front rear] in mm
int_z_leg               = v.Input.int_z_leg; %height boundary surface in the leg area [front rear] in mm
int_height              = floor(v.Input.int_height); %height of interior in mm
small_overlap           = v.Input.small_overlap; %small overlap active or not
offset_r                = v.dimensions.offset_r; %offset of rear boundary in mm
offset_r_pow            = v.dimensions.offset_r_pow; %offset of rear powertrain boundary in mm
offset_f                = v.dimensions.offset_f; %offset of front boundary in mm
offset_f_pow            = v.dimensions.offset_f_pow; %offset of front powertrain boundary in mm
alpha                   = v.Input.angle_br_min(1); %angle of front row (needed for back2back)
beta                    = v.Input.angle_br_min(2); %angle of rear row (needed for back2back)
Bmax                    = sum(v.interior.int_x_br_min)+v.Input.int_b2b_walldistance; %max width between backrests (needed for back2back)
hmax                    = min(v.interior.int_z_br_min); %min. max height of backrest (needed for back2back)

%% 2.) Calculate battery dimensions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Underfloor %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% a) Read z-boundaries for underfloor battery

bat_uf_zmin=v.dimensions.GZ.H156+z_batt_bottom_cover; %minimal space: ground clearance + bottom_cover
bat_uf_h=v.dimensions.GZ.underbody-z_pas_comp_floor-z_batt_top_cover-z_batt_bottom_cover-EZ_free_space_mod2cover; % height = Underbody - thickness pass.compartment - top cover - bottom cover - free space module to cover

front_boundary_max=[];
rear_boundary_min=[];

for i=1:size(v.interior.p_boundaries_f,2)
    front_boundary_max=max([front_boundary_max max(v.interior.p_boundaries_f{3,i}(v.dimensions.GZ.H156:int_height,:))]); %select max. value in the z-area (H156:Int_height) of battery
end
for i=1:size(v.interior.p_boundaries_r,2)
    rear_boundary_min=min([rear_boundary_min min(v.interior.p_boundaries_r{3,i}(v.dimensions.GZ.H156:int_height,:))]); %select max. value in the z-area (H156:Int_height) of battery
end
CX_batt_underfloor=rear_boundary_min-front_boundary_max-2*p_int_tolerance; % max. length = distance between package front and rear boundaries (incl. tolerance)
CY_batt_underfloor=v.dimensions.GY.vehicle_width-2*v.dimensions.p_crash_length(3); % width - 2*side free crash length
CZ_batt_underfloor=bat_uf_h;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Second Level %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Second level 1= Front underseat battery (for b2b between seat)
% Second level 2= Rear underseat battery (for b2b not existing)
%% Height
CZ_batt_second_level_1=int_z_leg(1)-z_batt_top_cover-EZ_free_space_mod2cover-p_int_tolerance; %height boundary surface in the leg area - battery cover - Module2cover distance - general tolerance
CZ_batt_second_level_2=int_z_leg(2)-z_batt_top_cover-EZ_free_space_mod2cover-p_int_tolerance; %height boundary surface in the leg area - battery cover - Module2cover distance - general tolerance

if strcmp(int_type,'btb') % for back-to back config --> calculate z-height with highest possible Volume
    % Calculate CZ (min from both seats)
    CZ_batt_second_level_1=min(int_z_leg)-z_batt_top_cover-EZ_free_space_mod2cover-p_int_tolerance; %height boundary surface in the leg area - battery cover - Module2cover distance - general tolerance
    % Calculate minimal needed parameters, see [1]
    gamma=(1/tand(alpha)+1/tand(beta)); 
    z_opt=Bmax/2*gamma; %height were rectangle is maximum inside triangle
    if z_opt<hmax
        b_opt=Bmax-z_opt*gamma;
    else
        z_opt=hmax;
        b_opt=Bmax-hmax*gamma;
    end
    CZ_batt_second_level_2=z_opt-p_int_tolerance-z_batt_top_cover-EZ_free_space_mod2cover;
end

%% Width
if small_overlap==1 % if small overlap for second level activated --> set width to available width between axles
    CY_batt_second_level_1=width_avail(1);
    CY_batt_second_level_2=width_avail(2);
    if strcmp(int_type,'btb') %for b2b always no small overlap (Wheel will not hit battery before passenger)
        CY_batt_second_level_1=int_y_front;
        CY_batt_second_level_2=int_y_rear;
    end
    
    if strcmp(int_type,'con') % for conventional no small overlap for first row (Wheel will not hit battery before passenger)
        CY_batt_second_level_1=int_y_front;
    end        
    
elseif small_overlap==0 % if no small overlap --> width = complete seat bench length
    CY_batt_second_level_1=int_y_front;
    CY_batt_second_level_2=int_y_rear;
end

%% Length and Position of Second Level Battery 1 (Front) (depending on the interior layout)
if strcmp(int_type,'btb') % Back-to-Back
    CX_batt_second_level_1=round(max(int_boundary_r_new))-round(min(int_boundary_f_new))-2*p_int_tolerance; % Distance between front and rear legs - 2*tolerance
    EX_batt_second_level_1=round(min(int_boundary_f_new))+p_int_tolerance; %position of max. x-value + tolerance
elseif strcmp(int_type,'sr')
    if small_overlap==1
        CX_batt_second_level_1=offset_r_pow-round(min(int_boundary_f_new))-2*p_int_tolerance; % distance between front legs and rear powertrain boundary max
    elseif small_overlap==0
        CX_batt_second_level_1=offset_r-round(min(int_boundary_f_new))-2*p_int_tolerance; % distance between front legs and general rear boundary (wheels included)
    end
    EX_batt_second_level_1=min(int_boundary_f_new)+p_int_tolerance; %position of max. x-value + tolerance
elseif strcmp(int_type,'vav') % Vis-a-Vis
    if small_overlap==1
        CX_batt_second_level_1=round(max(int_boundary_f_new))-offset_f_pow-p_int_tolerance; % distance between front legs and front powertrain boundary max
    elseif small_overlap==0
        CX_batt_second_level_1=round(max(int_boundary_f_new))-offset_f-p_int_tolerance; % distance between front legs and general front boundary (wheels included)
    end
    EX_batt_second_level_1=round(max(int_boundary_f_new))-CX_batt_second_level_1-p_int_tolerance; %position of max. x-value + tolerance
elseif strcmp(int_type,'con') % Conventional
    CX_batt_second_level_1=round(max(int_boundary_f_new))-round(min(int_boundary_f_new))-p_int_tolerance-int_x_overlap; % distance between front legs and rear legs - leg overlap
    EX_batt_second_level_1=round(min(int_boundary_f_new))+p_int_tolerance;  %position of max. x-value + tolerance
end

% Length and Position of Second Level Battery 2 (Rear) (depending on the interior layout)
if strcmp(int_type,'sr') % Single row --> no second level 2
    CX_batt_second_level_2=0;    
    EX_batt_second_level_2=0;
elseif strcmp(int_type,'btb') %Back2Back => Second Level 2 is above Second Level 1
    CX_batt_second_level_2=b_opt-2*p_int_tolerance;
    offset_b2b_sl2=z_opt/tand(alpha);
    EX_batt_second_level_2=min(v.interior.int_boundary_f_new)+v.interior.seat_br(1)+p_int_tolerance+offset_b2b_sl2; %Beginning of second level battery
else % Vis-a-Vis and Conventional
    if small_overlap==1
        CX_batt_second_level_2=offset_r_pow-round(min(int_boundary_r_new))-p_int_tolerance; % distance between rear legs and rear powertrain boundary max
    elseif small_overlap==0
        CX_batt_second_level_2=offset_r-round(min(int_boundary_r_new))-p_int_tolerance; % distance between rear legs and general rear boundary (wheels included)
    end   
    EX_batt_second_level_2=round(min(int_boundary_r_new))+p_int_tolerance;%position of max. x-value + tolerance
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Small overlap (required for mass calc PAUMANI!)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v.battery.installationspace.CX_batt_small_overlap=0;
v.battery.installationspace.CY_batt_small_overlap=0;
v.battery.installationspace.CZ_batt_small_overlap=0;

%% 3a) Assign Output Table
%Check if values are negative, otherwise set it to 0
CX_batt_second_level_1=(CX_batt_second_level_1>0)*CX_batt_second_level_1;
CY_batt_second_level_1=(CY_batt_second_level_1>0)*CY_batt_second_level_1;
CZ_batt_second_level_1=(CZ_batt_second_level_1>0)*CZ_batt_second_level_1;

CX_batt_second_level_2=(CX_batt_second_level_2>0)*CX_batt_second_level_2;
CY_batt_second_level_2=(CY_batt_second_level_2>0)*CY_batt_second_level_2;
CZ_batt_second_level_2=(CZ_batt_second_level_2>0)*CZ_batt_second_level_2;

CX_batt_underfloor=(CX_batt_underfloor>0)*CX_batt_underfloor;
CY_batt_underfloor=(CY_batt_underfloor>0)*CY_batt_underfloor;
CZ_batt_underfloor=(CZ_batt_underfloor>0)*CZ_batt_underfloor;

%Battery internal dimensions
v.dimensions.EZ.free_space_module_to_top_cover=EZ_free_space_mod2cover;

% Assign underfloor outputs
EX_batt_underfloor=front_boundary_max+p_int_tolerance;
EZ_batt_underfloor=bat_uf_zmin;
U=table(CX_batt_underfloor,CY_batt_underfloor,CZ_batt_underfloor,...
    EX_batt_underfloor,EZ_batt_underfloor);
U.Properties.VariableNames={'CX','CY','CZ','EX','EZ'};
U.Properties.RowNames={'underfloor'};
v.battery.spacetable(1,:)=U;

% Assign Second Level 1 outputs
EZ_batt_second_level_1=int_height; %interior height (only with underfloor, see DESIGN_battery)
S=table(CX_batt_second_level_1,CY_batt_second_level_1,CZ_batt_second_level_1,...
    EX_batt_second_level_1, EZ_batt_second_level_1);
S.Properties.VariableNames={'CX','CY','CZ','EX','EZ'};
S.Properties.RowNames={'second level'};
v.battery.spacetable(3,:)=S;

% Assign Second Level 2 outputs
if strcmp(int_type,'btb') %for back2back: position second level 2 is above second level 1
    EZ_batt_second_level_2=int_height+v.Input.int_z_leg(1);
else
    EZ_batt_second_level_2=int_height; %interior height (only with underfloor, see DESIGN_battery)
end
T=table(CX_batt_second_level_2,CY_batt_second_level_2,CZ_batt_second_level_2,...
    EX_batt_second_level_2,EZ_batt_second_level_2);
T.Properties.VariableNames={'CX','CY','CZ','EX','EZ',};
T.Properties.RowNames={'tunnel'};
v.battery.spacetable(4,:)=T;

%% 3b) Assign outputs
%Underfloor
v.battery.installationspace.CX_batt_underfloor=CX_batt_underfloor;
v.battery.installationspace.CY_batt_underfloor=CY_batt_underfloor;
v.battery.installationspace.CZ_batt_underfloor=CZ_batt_underfloor;
v.battery.installationspace.EX_batt_underfloor=EX_batt_underfloor;
v.battery.installationspace.EZ_batt_underfloor=EZ_batt_underfloor;
%Second Level 1
v.battery.installationspace.CX_batt_second_level_1=CX_batt_second_level_1;
v.battery.installationspace.CY_batt_second_level_1=CY_batt_second_level_1;
v.battery.installationspace.CZ_batt_second_level_1=CZ_batt_second_level_1;
v.battery.installationspace.EX_batt_second_level_1=EX_batt_second_level_1;
v.battery.installationspace.EZ_batt_second_level_1=EZ_batt_second_level_1;
%Second Level 2
v.battery.installationspace.CX_batt_second_level_2=CX_batt_second_level_2;
v.battery.installationspace.CY_batt_second_level_2=CY_batt_second_level_2;
v.battery.installationspace.CZ_batt_second_level_2=CZ_batt_second_level_2;
v.battery.installationspace.EX_batt_second_level_2=EX_batt_second_level_2;
v.battery.installationspace.EZ_batt_second_level_2=EZ_batt_second_level_2;
end