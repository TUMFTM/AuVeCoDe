function [vehicle] = Package_Front(vehicle,drivetrain_type,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow (Technical University of Munich)
%-------------
% Created on: 01.01.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the package of the frontal area of the vehicle 
%               creates a boundary-line of the relevant components
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - vehicle: struct with includes the data of the vehicle
%           - Drivertrain type (see Package_assign_topology)
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------

%% Implementation
% 1) Calculate y-Measurements
% 2) Calculate x- and z-Measurements
% 3) Calculate crash-system and cooler position
% 4) Calculate bumper and cooler x-pos.
% 5) Create Border-Line with xz-Layers
%       a) Select bodies that need block-building and determine boundary
%       b) Select bodies that do not need block-building and determine boundary
%       c) Merge both boundaries and select maximum
%       d) Assign Outputs
% 6) Update bumper position and overhang

%% Load/Write Parameters for Front Wagon
wagontype   =   1;    %1=Front, 2=Rear (To use same functions for front and rear wagon)
psi         =   vehicle.Input.psi_motor(1); %Read Angle of Motor to Gear in Degree

%% 0) Measurements:
%Minimum distance between plate and cross member along the X direction (in mm)
CX_plate_cross_member_front     =   Parameters.bumper.dimensions.CX.plate_cross_member_front;

%% 1) Calculate y-Measurements
[vehicle]=Package_WagonWidth(vehicle,wagontype,drivetrain_type,Parameters);
if vehicle.Error>0
    return
end

%% 2) Calculate x- and z-Measurements
[vehicle]=Package_WagonHeightLength(vehicle,wagontype,drivetrain_type,psi);

%% 3) Calculate crash-system and Cooler position
[vehicle]=Package_CCSystem(vehicle,wagontype,Parameters);

%% 4) Calculate bumper and cooler x-pos.
% min. required length = cooler length + cooling distance + half bumper length
min_length_needed   =   vehicle.dimensions.p_cooler_ctr(2,1)+vehicle.dimensions.p_cooling_distance+vehicle.dimensions.p_bumper{wagontype}(2,1)*0.5; 
% max. length offset of powertrain components in -x direction
offset_x            =   vehicle.dimensions.p_drivetrain{wagontype}(1,1)-vehicle.dimensions.p_drivetrain{wagontype}(2,1); 
% temporary bumper pos. = wheelhouse pos. + distance wheelhouse-bumper
p_bumper_temp_f     =   -(vehicle.Input.dist_wheelhouse2bumper_f+vehicle.dimensions.wheelhouse_width(wagontype)/2); 
% available length for cooler = offset - temp. bumper pos.
length_available    =   offset_x-p_bumper_temp_f; 

if (min_length_needed > length_available) && (drivetrain_type(1)>0) % available length is too small
    % update bumper x-pos. = offset - cooler length - cooling distance - half bumper length
    vehicle.dimensions.p_bumper{wagontype}(1,1) =   offset_x-vehicle.dimensions.p_cooler_ctr(2,1)-vehicle.dimensions.p_cooling_distance-vehicle.dimensions.p_bumper{wagontype}(2,1)*0.5;

    % cooler x-pos = bumper pos. + half bumper length + cooling distance + half cooler length
    vehicle.dimensions.p_cooler_ctr(1,1)        =   vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_cooler_ctr(2,1)+vehicle.dimensions.p_cooling_distance+vehicle.dimensions.p_bumper{wagontype}(2,1)*0.5;
else
    % bumper x-pos = temporary pos - half length bumper
    vehicle.dimensions.p_bumper{1}(1,1)         =   p_bumper_temp_f-vehicle.dimensions.p_bumper{wagontype}(2,1)*0.5;
    
    % cooler x-pos = bumper pos. + half bumper length + cooling distance + half cooler length
    vehicle.dimensions.p_cooler_ctr(1,1)        =   vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_cooler_ctr(2,1)+vehicle.dimensions.p_cooling_distance+vehicle.dimensions.p_bumper{wagontype}(2,1)*0.5;
end

%% 5) Create Border-Line with xz-Layers
%% a) Select bodies that need block-building and determine boundary
% Legend for drivetrain_type
% 0: no motor
% 1: motor with parallel gear
% 2: motor with coaxial gear (planetary)
% 3: two motors in coaxial position
% 4: motor with coaxial gear (layshaft)
if drivetrain_type(wagontype)==0 %no motor
    p_bodies="vehicle.dimensions.p_cooler_ctr";
    
elseif drivetrain_type(wagontype)==1 %motor with parallel gear
    vehicle.dimensions.p_powertrain{1}=cat(1,vehicle.dimensions.p_motor_paral{1},vehicle.dimensions.p_gear_paral{1});
    p_bodies=["vehicle.dimensions.p_powertrain{1}","vehicle.dimensions.p_powerel{1}","vehicle.dimensions.p_cooler_ctr"];
    
elseif  drivetrain_type(wagontype)==2 %motor with coaxial gear (planetary)
    vehicle.dimensions.p_powertrain{1}=cat(1,vehicle.dimensions.p_motor_coax{1},vehicle.dimensions.p_gear_coax{1});
    p_bodies=["vehicle.dimensions.p_powertrain{1}","vehicle.dimensions.p_powerel{1}","vehicle.dimensions.p_cooler_ctr"];
    
elseif drivetrain_type(wagontype)==3 %two motors in coaxial position
    vehicle.dimensions.p_powertrain_1{1}=cat(1,vehicle.dimensions.p_motor_nax{1}(1,:),vehicle.dimensions.p_motor_nax{1}(2,:),vehicle.dimensions.p_gear_nax{1}(1,:),vehicle.dimensions.p_gear_nax{1}(2,:));
    vehicle.dimensions.p_powertrain_2{1}=cat(1,vehicle.dimensions.p_motor_nax{1}(3,:),vehicle.dimensions.p_motor_nax{1}(2,:),vehicle.dimensions.p_gear_nax{1}(3,:),vehicle.dimensions.p_gear_nax{1}(2,:));
    p_bodies=["vehicle.dimensions.p_powertrain_1{1}","vehicle.dimensions.p_powertrain_2{1}","vehicle.dimensions.p_powerel{1}","vehicle.dimensions.p_cooler_ctr"];   %every body included into the front wagon, which is part of border to interior and block at crash

elseif drivetrain_type(wagontype)==4 %motor with coaxial gear (layshaft)
    vehicle.dimensions.p_powertrain{1}=cat(1,vehicle.dimensions.p_motor_coax{1},vehicle.dimensions.p_gear_coax_ls{1});
    p_bodies=["vehicle.dimensions.p_powertrain{1}","vehicle.dimensions.p_powerel{1}","vehicle.dimensions.p_cooler_ctr"];
end

% Find layer position and names
[Layer_ypos,Layer_names]    =   body_position(vehicle,p_bodies,drivetrain_type,wagontype);
Layer_boundary              =   cell(1,length(Layer_ypos)); %Preallocate boundary

for i =1:length(Layer_ypos)
    if strcmp(Layer_names{1,i},'-')
        Boundary_Front_max          =   zeros(length(Layer_boundary{1,1}),1);
    else
        [vehicle,Boundary_Front_max] =  Package_Layer(vehicle,Layer_names{1,i},psi,wagontype,1);
    end
    
    % Adding bumper distance to front axle in x-direction from Package_Layer_Calculate function
    Boundary_Front_max      =   Boundary_Front_max+(vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_bumper{wagontype}(2,1));
    Layer_boundary{1,i}     =   Boundary_Front_max;
end

%% b) Select bodies that do not need block-building and determine boundary
p_bodies2 = ["vehicle.dimensions.p_wheelhouse_left{1}","vehicle.dimensions.p_wheelhouse_right{1}"];

% Find layer position and names
[Layer_ypos2,Layer_names2]  = body_position(vehicle,p_bodies2,drivetrain_type,wagontype);
Layer_boundary2             =   cell(1,length(Layer_ypos2)); %Preallocate boundary

for i =1:length(Layer_ypos2)
    if strcmp(Layer_names2{1,i},'-')
        Boundary_Front_max2           = zeros(length(Layer_boundary2{1,1}),1);
    else
        [vehicle,Boundary_Front_max2] = Package_Layer(vehicle,Layer_names2{1,i},psi,wagontype,0);
    end
    
    % Adding bumper distance to front axle in x-direction from Package_Layer_Calculate function
    Boundary_Front_max2     =   Boundary_Front_max2+(vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_bumper{wagontype}(2,1));
    Layer_boundary2{1,i}    =   Boundary_Front_max2;
end

%% c) Merge both boundaries and select maximum
idx2_help           =   find(strcmp(Layer_names2,'-')); % find empty y-positions
Layer_ypos3         =   unique([Layer_ypos Layer_ypos2]); % create total unique y_pos
Layer_boundary3     =   cell(1,length(Layer_ypos3)); %Preallocate boundary
Layer_names3        =   cell(1,length(Layer_ypos3)); %Preallocate names

for i=1:length(Layer_ypos3)-1
    idx1    =   find(Layer_ypos<Layer_ypos3(i+1));
    idx2    =   find(Layer_ypos2<Layer_ypos3(i+1));
    if isempty(idx1) || length(idx1)==length(Layer_ypos)
        if ismember(idx2,idx2_help)
            Layer_boundary3{1,i}=   zeros(length(Layer_boundary3{1,1}),1);
            Layer_names3{1,i}   =   '-';
        else
            Layer_boundary3{1,i}=   Layer_boundary2{1,idx2(end)};
            Layer_names3{1,i}   =   Layer_names2{1,idx2(end)};
        end
    else
        Layer_boundary3{1,i}    =   max([Layer_boundary{1,idx1(end)} Layer_boundary2{1,idx2(end)}],[],2);
        if strcmp(Layer_names2{1,idx2(end)},'-')
            Layer_names3{1,i}   =   Layer_names{1,idx1(end)};
        else
            Layer_names3{1,i}   =   [Layer_names{1,idx1(end)},Layer_names2{1,idx2(end)}];
        end
    end
    if  any(Layer_boundary3{1,i}<Layer_boundary2{1}(end))
        Layer_boundary3{1,i}(Layer_boundary3{1,i}<Layer_boundary2{1}(end))  =   Layer_boundary2{1}(end);
    end
end

Layer_names3{1,end} =   Layer_names2{1,end};
Layer_boundary3(end)=   Layer_boundary2(end);

%% d) Assign Outputs
% Complete boundaries (wheelhouses, powertrain)
p_boundaries                        =   zeros(3,length(Layer_ypos3));
p_boundaries(2,:)                   =   Layer_ypos3;
p_boundaries                        =   num2cell(p_boundaries);
p_boundaries(1,:)                   =   Layer_names3;
p_boundaries(3,:)                   =   Layer_boundary3;
vehicle.dimensions.p_boundaries_f   =   p_boundaries;

% Powertrain boundaries (without wheelhouses)
p_boundaries_pow                    =   zeros(3,length(Layer_ypos));
p_boundaries_pow(2,:)               =   Layer_ypos;
p_boundaries_pow                    =   num2cell(p_boundaries_pow);
p_boundaries_pow(1,:)               =   Layer_names;
p_boundaries_pow(3,:)               =   Layer_boundary;
vehicle.dimensions.p_boundaries_f_pow=  p_boundaries_pow;

%% 6) Update bumper position and overhang
% update bumper pos if temp. pos. changed
vehicle.dimensions.p_bumper{1}(1,1)     =   min([vehicle.dimensions.p_bumper{1}(1,1)  p_boundaries{3}(end)-0.5*vehicle.dimensions.p_bumper{1}(2,1)]);

% overhang = bumper pos + half bumper length
vehicle.dimensions.GX.vehicle_overhang_f=   -(vehicle.dimensions.p_bumper{wagontype}(1,1)-0.5*vehicle.dimensions.p_bumper{wagontype}(2,1))+CX_plate_cross_member_front;

%% 99) Plot
%Optional
%Plot Front Wagon Boundary
% figure
% hold on
% for i=1:length(Layer_boundary3)-1
%     z_help=(1:1:(size(Layer_boundary3{1,i})));
%     surf([Layer_boundary3{1,i}';Layer_boundary3{1,i}'],[Layer_ypos3(i)*ones(size(Layer_boundary3{1,i}))';Layer_ypos3(i+1)*ones(size(Layer_boundary3{1,i}))'],[z_help;z_help],'Linestyle', 'none','FaceColor',[0 0 1],'FaceAlpha',0.5)
%
% end
% axis([-1000 3000 0 2000 0 3000])
% view(3)
% hold off
% axis equal
end

function [Layer_ypos,Layer_names] = body_position(vehicle,p_bodies,drivetrain_type,wagontype)
%Prealocate Boundary Surface from all bodies
Boundary_Front_Surf=cell(3,length(p_bodies)*2);

%Expression, where position and width is included in bodies
position='(1,2)';
width='(2,2)';

%Special variables for coaxial gearbox
position_gear='(3,2)';
width_gear='(4,2)';

% Special variables for parallel gearbox (last stage)
position_gear_paral_3='(11,2)';
width_gear_paral_3='(12,2)';

% Special variables for coaxial layshaft gearbox (second stage)
position_gear_coax_ls_2='(11,2)';
width_gear_coax_ls_2='(12,2)';

%Write all bodies and their edge y-positions in Boundary_Front_Surf
for i=1:length(p_bodies)
    Boundary_Front_Surf{1,i*2-1}=p_bodies(i);
    Boundary_Front_Surf{1,i*2}=p_bodies(i);
    
    if drivetrain_type(wagontype)==1 && strcmp(p_bodies(i),'vehicle.dimensions.p_powertrain{1}')
        % Parallel powertrain
        position_body_first_obj=append(p_bodies(i),position); % y-pos. of engine
        position_body_last_obj=append(p_bodies(i),position_gear_paral_3); % y-pos. housing
        width_body_first_obj=append(p_bodies(i),width); % width of engine
        width_body_last_obj=append(p_bodies(i),width_gear_paral_3); % width of housing
        
        Boundary_Front_Surf{2,i*2-1}=eval(position_body_first_obj)-0.5*eval(width_body_first_obj);
        Boundary_Front_Surf{2,i*2}=eval(position_body_last_obj)+0.5*eval(width_body_last_obj);
        
    elseif (drivetrain_type(wagontype)==2 || drivetrain_type(wagontype)==3) && (strcmp(p_bodies(i),'vehicle.dimensions.p_powertrain{1}') || strcmp(p_bodies(i),'vehicle.dimensions.p_powertrain_2{1}'))
        % Coaxial or left near-axle powertrain
        position_body_first_obj=append(p_bodies(i),position); % y-pos. of left engine
        position_body_last_obj=append(p_bodies(i),position_gear); % y-pos. of left gears
        width_body_first_obj=append(p_bodies(i),width); % width of left engine
        width_body_last_obj=append(p_bodies(i),width_gear); % width of left gears
        
        Boundary_Front_Surf{2,i*2-1}=eval(position_body_first_obj)-0.5*eval(width_body_first_obj);
        Boundary_Front_Surf{2,i*2}=eval(position_body_last_obj)+0.5*eval(width_body_last_obj);
        
    elseif strcmp(p_bodies(i),'vehicle.dimensions.p_powertrain_1{1}')
        % Right near-axle powertrain
        position_body_first_obj=append(p_bodies(i),position_gear); % y-pos. of left engine
        position_body_last_obj=append(p_bodies(i),position); % y-pos. of right gears
        width_body_first_obj=append(p_bodies(i),width_gear); % width of right engine
        width_body_last_obj=append(p_bodies(i),width); % width of right gears
        
        Boundary_Front_Surf{2,i*2-1}=eval(position_body_first_obj)-0.5*eval(width_body_first_obj);
        Boundary_Front_Surf{2,i*2}=eval(position_body_last_obj)+0.5*eval(width_body_last_obj);
        
    elseif drivetrain_type(wagontype)==4 && strcmp(p_bodies(i),'vehicle.dimensions.p_powertrain{1}')
        % Coaxial layshaft powertrain
        position_body_first_obj=append(p_bodies(i),position); % y-pos. of engine
        position_body_last_obj=append(p_bodies(i),position_gear_coax_ls_2); % y-pos. housing
        width_body_first_obj=append(p_bodies(i),width); % width of engine
        width_body_last_obj=append(p_bodies(i),width_gear_coax_ls_2); % width of housing
        
        Boundary_Front_Surf{2,i*2-1}=eval(position_body_first_obj)-0.5*eval(width_body_first_obj);
        Boundary_Front_Surf{2,i*2}=eval(position_body_last_obj)+0.5*eval(width_body_last_obj);
    else
        position_body=append(p_bodies(i),position);
        width_body=append(p_bodies(i),width);
        
        Boundary_Front_Surf{2,i*2-1}=eval(position_body)-0.5*eval(width_body);
        Boundary_Front_Surf{2,i*2}=eval(position_body)+0.5*eval(width_body);
    end
end

%All positions of Layers
Layer_ypos=zeros(1,length(p_bodies)*2); %Preallocate y_pos string
Layer_ypos(1,:)=ceil([Boundary_Front_Surf{2,1:length(p_bodies)*2}]); %Use Values of bodies
Layer_ypos=sort(Layer_ypos); %Sort values of bodies
Layer_ypos=unique(Layer_ypos); %delete double values
Layer_names=cell(1,length(Layer_ypos)); %Preallocate name string

for i=1:length(Layer_ypos)
    names='-'; %Reset Name string after every loop
    for j=1:length(p_bodies)
        
        if Boundary_Front_Surf{2,j*2-1}<=Layer_ypos(i) && Boundary_Front_Surf{2,j*2}>Layer_ypos(i) %Compare if body is at this y-position
            if strcmp(names,'-')
                names=Boundary_Front_Surf{1,j*2-1}; %for the first time writing a name in the string
            else
                names=[names,Boundary_Front_Surf{1,j*2-1}]; %for every further name
            end
        end
        
    end
    Layer_names{1,i}=names; %write all names of bodies at this y-position into the Layer_names
end
end