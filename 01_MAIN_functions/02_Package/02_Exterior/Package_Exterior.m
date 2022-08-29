function [vehicle] = Package_Exterior(vehicle,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the package of the complete vehicle and updates
%               the exterior dimensional parameters. It also calculates the available
%               battery space
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------
%%
%% Conctent
% 1) Merge Interior to Front Wagon
%       a) Read and preallocate
%       b) Shift Interior Boundary Surface to the right height
%       c) Move interior to right position
% 2) Merge Rear Wagon to Interior
% 3) Update vehicle dimensions
%       a) Wheelstand
%       b) Battery space and position
%       c) Length
%       d) Width
%       e) Height
%       f) x distance of passenger COG to front axle

%% 1) Merge Interior to Wagons
% a) Read and preallocate
int_boundary_f      =   vehicle.interior.int_boundary_f; %Interior boundary surface front
int_boundary_r      =   vehicle.interior.int_boundary_r; %Interior boundary surface rear
length_ibf          =   length(int_boundary_f); %length of interior boundary front (ibf)
length_ibr          =   length(int_boundary_r); %length of interior boundary rear (ibr)

int_boundary_f2     =   vehicle.interior.int_boundary_f2(1:length_ibf); %Vertical front Boundary required for legroom
int_boundary_r2     =   vehicle.interior.int_boundary_r2(1:length_ibr); %Vertical rear Boundary required for legroom
length_ibf          =   ceil(length_ibf+vehicle.Input.int_height); %Length of vector with interior moved to the right height
length_ibr          =   ceil(length_ibr+vehicle.Input.int_height); %Length of vector with interior moved to the right height

p_boundaries_f      =   vehicle.dimensions.p_boundaries_f; %boundary surface of front wagon
p_boundaries_r      =   vehicle.dimensions.p_boundaries_r; %boundary surface of rear wagon
p_boundaries_f_pow  =   vehicle.dimensions.p_boundaries_f_pow; %boundary surface of front wagon (powertrain)
p_boundaries_r_pow  =   vehicle.dimensions.p_boundaries_r_pow; %boundary surface of rear wagon (powertrain)

length_wbf          =   length(p_boundaries_f{3,1}); % front wagon boundary vector length
length_wbr          =   length(p_boundaries_r{3,1}); % rear wagon height boundary vector length
length_max          =   ceil(max([length_ibf,length_ibr,length_wbf,length_wbr])); %Check if vector of wagon or interior is longer

% b) Shift Interior Boundary Surface to the right height
int_boundary_f_new=NaN(1,length_max);
int_boundary_r_new=NaN(1,length_max);
int_boundary_f2_new=NaN(1,length_max);
int_boundary_r2_new=NaN(1,length_max);

%Assign new height of interior
int_boundary_f_new(ceil(vehicle.Input.int_height+1):length_ibf)=int_boundary_f;
int_boundary_r_new(ceil(vehicle.Input.int_height+1):length_ibr)=int_boundary_r;
int_boundary_f2_new(ceil(vehicle.Input.int_height+1):length_ibf)=int_boundary_f2;
int_boundary_r2_new(ceil(vehicle.Input.int_height+1):length_ibr)=int_boundary_r2;
int_boundary_f_new=int_boundary_f_new';
int_boundary_r_new=int_boundary_r_new';
int_boundary_f2_new=int_boundary_f2_new';
int_boundary_r2_new=int_boundary_r2_new';

% c) Move interior to right position
offset_int2fw=zeros(1,length(p_boundaries_f)); %preallocate variable
vehicle.dimensions.offset_f=zeros(1,length(p_boundaries_f)); %preallocate variable

% Calculate distance of every part of front wagon boundary in relation to interior
% Attention: Interior and wagon have their x-reference at x=0 at the initial status
for i=1:length(offset_int2fw)
    p_boundary=NaN(length_max,1); %preallocate variable
    p_boundary(1:length_wbf,1)=p_boundaries_f{3,i}; %take one boundary line of wagon
    offset1=min(int_boundary_f_new-p_boundary); %calculate maximal distance between interior and boundary line
    offset2=min(int_boundary_f2_new-p_boundary); %calculate maximal distance between legroom and boundary line
    offset_int2fw(i)=min(offset1,offset2); %write value "Offset interior to frontwagon" in variable
    vehicle.dimensions.offset_f(i)=max(p_boundary); %calculate max. point in x-direction (for battery calculation!)
end

% Calculate distance of powertrain only (including free crash length)
vehicle.dimensions.offset_f_pow=zeros(1,size(p_boundaries_f_pow,2)); %preallocate powertrain offset variable
for i=1:size(p_boundaries_f_pow,2)
    vehicle.dimensions.offset_f_pow(i)=max(p_boundaries_f_pow{3,i}); %calculate max. point in x-direction (for battery calculation)
end
vehicle.dimensions.offset_f=max(vehicle.dimensions.offset_f); %max. distance front wagon to interior
vehicle.dimensions.offset_f_pow=max(vehicle.dimensions.offset_f_pow); %max. distance powertrain (front wagon) to interior

offset_int2fw=min(offset_int2fw); %check what is the maximal distance (min: the maximal "smallest" value, since it was a subtraction)
vehicle.manikin.offset_int2fw=offset_int2fw;
vehicle.interior.int_boundary_f_new=int_boundary_f_new-offset_int2fw+vehicle.Input.dist_int2fw; %move front interior line using maximal distance
vehicle.interior.int_boundary_f2_new=int_boundary_f2_new-offset_int2fw+vehicle.Input.dist_int2fw; %move front interior line using maximal distance
vehicle.interior.int_boundary_r_new=int_boundary_r_new-offset_int2fw+vehicle.Input.dist_int2fw; %move rear interior line using maximal distance
vehicle.interior.int_boundary_r2_new=int_boundary_r2_new-offset_int2fw+vehicle.Input.dist_int2fw;%move rear legroom line using maximal distance

%Calculate distance interior to complete boundary
d_1=max(cell2mat(p_boundaries_f(2,:)));
d_2=max(cell2mat(p_boundaries_r(2,:)));
vehicle.interior.a_1=(d_1-vehicle.interior.int_y_front)/2; % interior reference points for DISPLAY_vehicle
vehicle.interior.a_2=(d_2-vehicle.interior.int_y_rear)/2; % interior reference points for DISPLAY_vehicle

%% 2) Merge Rear Wagon to Interior
offset_rw2int=zeros(1,length(p_boundaries_r)); %preallocate offset variable

% Calculate Distance between every boundary line of rear wagon and interior (interior already moved with p_int2fw tolerance to front wagon)
for i=1:length(offset_rw2int)
    p_boundary=NaN(length_max,1); %preallocate variable
    p_boundaries_r{3,i}=p_boundaries_r{3,i}*-1; %Switch x-Direction of boundary surface (for rear wagon)
    p_boundary(1:length_wbr,1)=p_boundaries_r{3,i}; %take one boundary line of wagon
    offset1=max(vehicle.interior.int_boundary_r_new-p_boundary); %calculate maximal distance between interior and boundary line
    offset2=max(vehicle.interior.int_boundary_r2_new-p_boundary);%calculate maximal distance between legroom and boundary line
    %Annotation: Distance between frontwagon and interior is already included in the int_boundary_r/r2_new boundary surfaces 
    offset_rw2int(i)=max(offset1,offset2); %write value in variable
end

offset_rw2int=max(offset_rw2int); %check what is the maximal distance between rear wagon (placed into the origin of front wagon) 
                                  % and interior (already moved to the right position in relation to front wagon including p_int2fw_tolreance)

%Move boundary lines to right position
vehicle.dimensions.offset_r=zeros(1,length(p_boundaries_r)); %preallocate offset variable
for i=1:length(p_boundaries_r)
    p_boundaries_r{3,i}=p_boundaries_r{3,i}+offset_rw2int+vehicle.Input.dist_int2rw; %move rear wagon boundary line using maximal distance
    vehicle.dimensions.offset_r(i)=min(p_boundaries_r{3,i});
end

%Move boundary line (of just powertrain) to right position
vehicle.dimensions.offset_r_pow=zeros(1,size(p_boundaries_r_pow,2)); %preallocate powertrain offset variable
for i=1:size(p_boundaries_r_pow,2)
    p_boundaries_r_pow{3,i}=p_boundaries_r_pow{3,i}*-1;
    p_boundaries_r_pow{3,i}=p_boundaries_r_pow{3,i}+offset_rw2int+vehicle.Input.dist_int2rw; %move rear wagon boundary line using maximal distance
    vehicle.dimensions.offset_r_pow(i)=min(p_boundaries_r_pow{3,i});
end
vehicle.dimensions.offset_r=min(vehicle.dimensions.offset_r);
vehicle.dimensions.offset_r_pow=min(vehicle.dimensions.offset_r_pow);

%If wagon has no powertrain: Boundary limits battery (instead of powertrain)
if isempty(vehicle.dimensions.offset_r_pow)
    vehicle.dimensions.offset_r_pow=offset_rw2int- ...     %Position of rear boundary without powertrain (wheelbase+bumper_pos-0.5bumper_thickness-freecrashlength+tolerance)
    vehicle.dimensions.p_bumper{1,2}(1,1)-0.5*vehicle.dimensions.p_bumper{2}(2,1) ...
    -vehicle.dimensions.p_crash_length(2)+vehicle.Input.dist_int2rw;
end

if isempty(vehicle.dimensions.offset_f_pow)
    vehicle.dimensions.offset_r_pow=vehicle.dimensions.p_bumper{1,1}(1,1) ... %Position of front boundary without powertrain (bumper_pos+0.5bumper_thickness-freecrashlength+tolerance)
    +0.5*vehicle.dimensions.p_bumper{1}(2,1)+vehicle.dimensions.p_crash_length(1)+vehicle.Input.dist_int2rw;
end

vehicle.interior.p_boundaries_r=p_boundaries_r;
vehicle.interior.p_boundaries_f=p_boundaries_f;

%% 3) Update vehicle dimensions
%a) Wheelstand
vehicle.dimensions.GX.wheelbase=offset_rw2int+vehicle.Input.dist_int2rw; %Wheelstand is offset of rearwagon to interior (incl. distance interior 2 front wagon) + distance interior 2 rear wagon

%c) Length
vehicle.dimensions.GX.vehicle_length = vehicle.dimensions.GX.wheelbase + vehicle.dimensions.GX.vehicle_overhang_f + vehicle.dimensions.GX.vehicle_overhang_r;

%d) Width
vehicle.dimensions.GY.vehicle_width=max([d_1, d_2, vehicle.interior.int_y_tot+2*vehicle.dimensions.p_crash_length(3)]);

%e) Height
vehicle.dimensions.GZ.underbody_height = vehicle.Input.int_height - vehicle.dimensions.GZ.H156;
if Parameters.Equipment.sliding_roof==1
    d_roof=Parameters.dimensions.CZ.sliding_roof_thickness; %roof thickness for sliding roof
elseif Parameters.Equipment.panorama_roof==1
    d_roof=Parameters.dimensions.CZ.panoramic_roof_thickness; %roof thickness for panoramic roof
else
    d_roof=Parameters.dimensions.CZ.roof_thickness; %roof thickness for normal roof
end
vehicle.dimensions.GZ.vehicle_height = max([vehicle.dimensions.p_drivetrain{1}(1,3),vehicle.dimensions.p_drivetrain{2}(1,3),length_ibf,length_ibr])+d_roof;

%f) x distance of passenger COG to front axle (Pos. of interior boundary rear/front +half length of seat) 
vehicle.manikin.SgRP_X_front_axle=min(vehicle.interior.int_boundary_f_new)+(max(vehicle.interior.int_boundary_f_new)-min(vehicle.interior.int_boundary_f_new))/2;
vehicle.manikin.SgRP2_X_front_axle=min(vehicle.interior.int_boundary_r_new)+(max(vehicle.interior.int_boundary_r_new)-min(vehicle.interior.int_boundary_r_new))/2;

end

