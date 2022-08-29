function [vehicle] = INITIALIZE_vehicle(Parameters,varargin)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow (Technical University of Munich)
%-------------
% Created on: 01.01.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function initializes the vehicle concept for the first time. It estimates the weight and executes a first Longitudinal simulation.
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - varargin: if optimization is active, optimization variables
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------

%% Implementation
% 1. Initialization of structure
% 2. Linking optimization variables into vehicle struct
% 3. Allocation of user input/requirements
% 4. Initialization of temp. parameters for first LDS
% 5. First LDS

%% 1. Initialization of vehicle structure
clear vehicle
vehicle=struct;
vehicle.feasible=[]; % Concept feasible=1, not feasible=0
vehicle.iteration=1; % Start iteration counter
vehicle.Error=0; % Set error initialiy to 0

%% 2. Linking optimization variables into vehicle struct
%%---------------------------Motor Topology--------------------------------
if isempty(Parameters.input.motor_topology)
    switch varargin{1}{1}(1)
        case 1
            vehicle.Input.topology ='GM_X';
        case 2
            vehicle.Input.topology ='X_GM';
        case 3
            vehicle.Input.topology ='GM_GM';
        case 4
            vehicle.Input.topology ='2G2M_X';
        case 5
            vehicle.Input.topology ='X_2G2M';
        case 6
            vehicle.Input.topology ='2G2M_GM';
        case 7
            vehicle.Input.topology ='GM_2G2M';
        case 8
            vehicle.Input.topology ='2G2M_2G2M';
    end
else
    vehicle.Input.topology = Parameters.input.motor_topology;
end

%Determine drive type according to the number of machines
if count(vehicle.Input.topology,'G')>1
    
    vehicle.topology.drive='AWD';
    
else
    
    if strcmp(vehicle.Input.topology(end),'X')
        vehicle.topology.drive='FWD';
    else
        vehicle.topology.drive='RWD';
    end
    
end

% Enter weight distribution according to powertrain topology
vehicle.masses.optional_extras.front_repartition  = Parameters.masses.loads.axle_load_front.(vehicle.topology.drive); %Give initial weight repartition on front axle

% ---------------------------Engine Type-----------------------------------
if isempty(Parameters.input.engine_type)
    switch varargin{1}{1}(2)
        case 1
            vehicle.Input.machine_type_f = 'PSM';
            vehicle.Input.machine_type_r = 'PSM';
        case 2
            vehicle.Input.machine_type_f = 'ASM';
            vehicle.Input.machine_type_r = 'ASM';
    end
else
    vehicle.Input.machine_type_f=Parameters.input.engine_type;
    vehicle.Input.machine_type_r=Parameters.input.engine_type;
end

% ---------------------------Gear type front-------------------------------
if isempty(Parameters.input.gear_type_front)
    switch varargin{1}{1}(3)
        case 1
            vehicle.Input.gear_type_f = 'Parallel';
        case 2
            vehicle.Input.gear_type_f = 'Coaxial';
        case 3
            vehicle.Input.gear_type_f = 'Coaxial-layshaft';
    end
else
    vehicle.Input.gear_type_f=Parameters.input.gear_type_front;
end

% ---------------------------Gear type rear-------------------------------
if isempty(Parameters.input.gear_type_rear)
    switch varargin{1}{1}(4)
        case 1
            vehicle.Input.gear_type_r = 'Parallel';
        case 2
            vehicle.Input.gear_type_r = 'Coaxial';
        case 3
            vehicle.Input.gear_type_r = 'Coaxial-layshaft';
    end
else
    vehicle.Input.gear_type_r=Parameters.input.gear_type_rear;
end

% ---------------------------Battery cell-type----------------------------
if isempty(Parameters.input.cell_type)
    switch varargin{1}{1}(6)
        case 1
            vehicle.Input.cell_type = 'prismatic';
        case 2
            vehicle.Input.cell_type = 'cylindrical';
        case 3
            vehicle.Input.cell_type = 'pouch';
    end
else
    vehicle.Input.cell_type = Parameters.input.cell_type;
end

% ---------------------------Rear axle type-------------------------------
if isempty(Parameters.input.rear_axis_type)
    switch varargin{1}{1}(11)
        case 1
            vehicle.Input.axis_type_r = 'torsion_beam';
            vehicle.topology.spring_layout_r = 'Feder in X vor Daempfer';
            
        case 2
            vehicle.Input.axis_type_r = 'trapezoidal_link';
            vehicle.topology.spring_layout_r = 'Feder in X vor Daempfer';
            
        case 3
            vehicle.Input.axis_type_r = 'sword_arm_link';
            vehicle.topology.spring_layout_r = 'Feder in X vor Daempfer';
            
        case 4
            vehicle.Input.axis_type_r = 'five_link';
            vehicle.topology.spring_layout_r = 'Feder in X vor Daempfer';
    end
else
    vehicle.Input.axis_type_r = Parameters.input.rear_axis_type;
    vehicle.topology.spring_layout_r = 'Feder in X vor Daempfer';
end
vehicle.topology.axis_type_r=vehicle.Input.axis_type_r;

% ---------------------------Gearbox optimization goal--------------------
if isempty(Parameters.input.gearbox_opt)
    switch varargin{1}{1}(14)
        case 1
            vehicle.Input.gearbox_opt='Length';
        case 2
            vehicle.Input.gearbox_opt='Height';
    end
else
    vehicle.Input.gearbox_opt=Parameters.input.gearbox_opt;
end

%---------------------------Small Overlap Second Level Battery-----------
if isempty(Parameters.input.small_overlap)
    switch varargin{1}{1}(17)
        case 1
            vehicle.Input.small_overlap=1;
        case 2
            vehicle.Input.small_overlap=0;
    end
else
    vehicle.Input.small_overlap=Parameters.input.small_overlap ; %small overlap (=1: battery is not wider then wheelhouse)
end

%---------------------------Exterior - Trunk Type -----------
%Front
if isempty(Parameters.input.trunk_type_front)
    switch varargin{1}{1}(26)
        case 1
            vehicle.Input.trunk_type_front='integrated';
        case 2
            vehicle.Input.trunk_type_front='hooded';
    end
else
    vehicle.Input.trunk_type_front=Parameters.input.trunk_type_front ;
end
%Rear
if isempty(Parameters.input.trunk_type_rear)
    switch varargin{1}{1}(27)
        case 1
            vehicle.Input.trunk_type_rear='integrated';
        case 2
            vehicle.Input.trunk_type_rear='hooded';
    end
else
    vehicle.Input.trunk_type_rear=Parameters.input.trunk_type_rear ;
end

% ---------------------------Single vehicle or optimization---------------
if Parameters.input.single_vehicle %Read values if single vehicle is active
    % Wheel radius (conversion from diameter to radius)
    vehicle.Input.r_tire = Parameters.input.tire_diam_min/2;
    
    % Drive-shaft-axle angles [Front-xy Front-xz Rear-xy Rear-xz]
    vehicle.Input.alpha_axle = [Parameters.input.axle_angle_1_min Parameters.input.axle_angle_2_min;...
        Parameters.input.axle_angle_1_min Parameters.input.axle_angle_2_min];
    
    % Engine-gearbox angle[Front Rear]
    vehicle.Input.psi_motor = [Parameters.input.engine_angle_front_min Parameters.input.engine_angle_rear_min];
    
    % Gear ratios 
    vehicle.Input.i_gearbox_f = [Parameters.input.gear_ratio_1_front_min];
    vehicle.Input.i_gearbox_r = [Parameters.input.gear_ratio_1_rear_min];
    vehicle.Input.eta_gearbox_f = [Parameters.LDS.eta_gearbox];
    vehicle.Input.eta_gearbox_r = [Parameters.LDS.eta_gearbox];

    
    % Tunk ratio front/rear [%]
    vehicle.Input.trunk_ratio = Parameters.input.trunk_ratio_min/100;
    
    % Interior height [mm]
    vehicle.Input.int_height = round(Parameters.input.int_height_min); %round for indexing
    
    % Distance wheelbase to bumper [mm]
    vehicle.Input.dist_wheelhouse2bumper_f=Parameters.input.dist_wheelhouse2bumper_f_min;
    vehicle.Input.dist_wheelhouse2bumper_r=Parameters.input.dist_wheelhouse2bumper_r_min;
    
    % Distance between interior boundary surface and front/rear wagon boundary surface [mm]
    vehicle.Input.dist_int2fw = Parameters.input.dist_int2fw_min;
    vehicle.Input.dist_int2rw = Parameters.input.dist_int2rw_min;    
    
    % Steering ratio front/total [0-1]
    vehicle.Input.steering_ratio = Parameters.input.steering_ratio_min;
    
    % Torque ratio front/total [%]
    vehicle.Input.torque_ratio = Parameters.input.torque_ratio_min;
    
    %H30 measurement [mm]    
    vehicle.Input.H30_f     = Parameters.input.H30_f_min;
    vehicle.Input.H30_r     = Parameters.input.H30_r_min;
    
    %Window angle front/rear [°]
    vehicle.Input.window_angle_front   = Parameters.input.window_angle_front_min;
    vehicle.Input.window_angle_rear    = Parameters.input.window_angle_rear_min;

else %Read values if optimization is active
    % Wheel radius (conversion from diameter to radius)
    vehicle.Input.r_tire = varargin{1}{1}(5)/2;
    
    % Drive-shaft-axle angles [Front-xy Front-xz Rear-xy Rear-xz]
    vehicle.Input.alpha_axle=[varargin{1}{1}(7) varargin{1}{1}(8);varargin{1}{1}(7) varargin{1}{1}(8)];
    
    % Engine-gearbox angle[Front Rear]
    vehicle.Input.psi_motor=[varargin{1}{1}(9) varargin{1}{1}(10)];
    vehicle.topology.gearbox_orientation=vehicle.Input.psi_motor;
    
    % Gear ratios (dependend if 2 speed or 1 speed gearbox)
    vehicle.Input.i_gearbox_f = varargin{1}{1}(12);
    vehicle.Input.i_gearbox_r = varargin{1}{1}(13);
    vehicle.Input.eta_gearbox_f = [Parameters.LDS.eta_gearbox];
    vehicle.Input.eta_gearbox_r = [Parameters.LDS.eta_gearbox];
    
    
    % Tunk ratio front/rear [%]
    vehicle.Input.trunk_ratio = varargin{1}{1}(15)/100;
    
    % Interior height [mm]
    vehicle.Input.int_height = round(varargin{1}{1}(16));
    
    % Distance wheelbase to bumper [mm]
    vehicle.Input.dist_wheelhouse2bumper_f=varargin{1}{1}(18);
    vehicle.Input.dist_wheelhouse2bumper_r=varargin{1}{1}(19);
    
    % Steering ratio front/total [0-1]
    vehicle.Input.steering_ratio = varargin{1}{1}(20);
    
    % Torque ratio front/total [%]
    vehicle.Input.torque_ratio = varargin{1}{1}(21);
    
    %H30 measurement [mm]
    vehicle.Input.H30_f= varargin{1}{1}(22);
    vehicle.Input.H30_r= varargin{1}{1}(23);
    
    %Window angle front/rear [°]
    vehicle.Input.window_angle_front   = varargin{1}{1}(24);
    vehicle.Input.window_angle_rear    = varargin{1}{1}(25);
    
    % Distance between interior boundary surface and front/rear wagon boundary surface [mm]
    vehicle.Input.dist_int2fw = varargin{1}{1}(28);
    vehicle.Input.dist_int2rw = varargin{1}{1}(29);   
end

%% 3. Allocation of user input/requirements
vehicle.Input.range                         = Parameters.input.range;                 % min. required range [km]
vehicle.Input.int_type                      = Parameters.input.seating_layout;        % interior layout
vehicle.Input.n_seat                        = Parameters.input.n_passengers;          % num. passengers
vehicle.Input.number_of_seats               = sum(Parameters.input.n_passengers);     % total num. passengers
vehicle.topology.number_of_seats            = vehicle.Input.number_of_seats;          % write num. seats (for calc_interior_weight)
vehicle.Input.int_z_tot                     = Parameters.interior.total_height;       % interior height [mm]
vehicle.Input.ground_clearance              = Parameters.input.ground_clearance;      % type of ground clearance
vehicle.Input.driving_cycle                 = Parameters.input.driving_cycle;         % driving cycle
vehicle.Input.max_speed                     = Parameters.input.max_speed;             % max speed [km/h]
vehicle.Input.acceleration_time_req         = Parameters.input.acceleration_time_req; % req. time for 0-100km/h [s]
vehicle.Input.H30_Opt                       = Parameters.input.H30_Opt;               %H30 Switch
vehicle.settings.plot_boundary              = Parameters.settings.plot_boundary;      %Display boundary in plot
vehicle.Input.timeframe                     = Parameters.timeframe;                   %Timeframe of vehicle, 1=2020,2=2025,3=2030
vehicle.Input.steering_radius               = Parameters.input.steering_radius;

vehicle.dimensions.p_crash_length = [Parameters.input.free_crash_length(1),...
    Parameters.input.free_crash_length(2), Parameters.input.free_crash_length(3)]; % free crash length [front,rear,sides]

if strcmp(vehicle.Input.ground_clearance,'flatfloor')
    vehicle.dimensions.GZ.H156 = Parameters.general_dimensions.ground_clearance.flatfloor; % Ground clearance space for flatfloor [mm]
else
    vehicle.dimensions.GZ.H156 = Parameters.general_dimensions.ground_clearance.highfloor; % Ground clearance space for highfloor [mm]
end

%% 4. Temporary parameters for first LDS

%%---------------------------Motor Inputs-----------------------------------------------------
vehicle.Input.T_max_Mot_f               =  NaN;                            % max torque of front machine in Nm
vehicle.Input.T_max_Mot_r               =  NaN;                            % max torque of rear machine in Nm
if isfield(Parameters.input,'characteristic_forced')                       % check if characteristic forced is an input
else
    Parameters.input.characteristic_forced = {NaN,NaN};
end
vehicle.Input.characteristic_forced     =  Parameters.input.characteristic_forced;% select forced characteristic per axle, otherwise NaN, f.e. characteristic_forced = {'PSM_M270_n12000_0.2.mat',NaN};
vehicle.Input.weight_T                  =  Parameters.motor.weight_T;       % select Torque weight factor per axle
vehicle.Input.ratio_req                 =  Parameters.motor.ratio_req;      % select required angular speed ratio per axle
vehicle.Input.weight_n                  =  Parameters.motor.weight_n;       % select speed weight factor per axle
vehicle.Input.weight_ratio              =  Parameters.motor.weight_ratio;   % select angular speed ratio weight factor per axle

%%---------------------------LDS-------------------------------------------------------------------
vehicle.Input.e_i                       =   NaN;                           % e_i value (optional)-> Inertia factor
vehicle.Input.c_r                       =   NaN;                           % roll resistance coefficient, (optional)
vehicle.Input.c_d                       =   NaN;                           % air resistance coefficient, (optional)
vehicle.Input.eta_gearbox_r             =   NaN;                           % efficiency of rear gearbox, optional
vehicle.Input.eta_gearbox_f             =   NaN;                           % efficiency of front gearbox, optional
try
    vehicle.aux.base = Parameters.input.aux_base;    %given auxiliary consumption in kW
    vehicle.Input.power_auxiliaries = vehicle.aux.base; %take base auxiliary consumption for LDS
catch
    vehicle.aux.base = 0;    %given auxiliary consumption in kW
    vehicle.Input.power_auxiliaries         =   NaN;                           % set NaN or power auxiliary consumption in kW
end
vehicle.settings.suppress_warnings      =   Parameters.settings.suppress_warnings; % Set to 1 to suppress the LDS warnings and to 0 to show them
vehicle.LDS.settings.suppress_LDS_warnings= Parameters.settings.suppress_warnings; % Set to 1 to suppress the LDS warnings and to 0 to show them

%%---------------------------Battery-------------------------------------------------------------------
vehicle.settings.fill_tunnel = Parameters.settings.fill_tunnel;       % fill tunnel space with cells
vehicle.settings.fill_smalloverlapspace = Parameters.settings.fill_smalloverlapspace; % fill smalloverlap with cells
%------NEEDS TO BE UPDATED------
vehicle.settings.allow_cell_length_in_z=0; %Allow rotation
vehicle.settings.allow_cell_width_in_z=0; %Allow rotation
%------END-----
vehicle.topology.battery_topology = Parameters.battery.bat_topology; % highfloor, mixedfloor or lowfloor
vehicle.battery.energy_is_gross_in_kWh=0; % gross battery capacity [kWh]
vehicle.battery.spacetable=Parameters.spacetable; % space table for PAUMANI calc battery
vehicle.battery.filltable=Parameters.filltable; % fill table for PAUMANI calc battery
vehicle.dimensions.CZ.batt_top_cover=Parameters.dimensions.CZ.batt_top_cover; % thickness battery top cover
vehicle.dimensions.EZ.passenger_compartement_floor=Parameters.dimensions.EZ.pass_compartement_battery; % thickness of interior compartment
vehicle.battery.installationspace.EX_batt_underfloor_front_axle = Parameters.general_dimensions.initial_dist_bat2axle; % initial distance to axle (for PAUMANI calc)
vehicle.battery.installationspace.EX_battery_rear_axle_underfloor_space = Parameters.general_dimensions.initial_dist_bat2axle; % initial distance to axle (for PAUMANI calc)
vehicle.dimensions.GZ.underbody=vehicle.Input.int_height-vehicle.dimensions.GZ.H156; % underbody thickness (for Package Battery dimensions)
vehicle.settings.cooling_in_z=Parameters.input.battery_cooling_in_z; % Binary value: 1=there is a cooling plate underneath the modules; 0: There is no cooling plate underneath the modules

%%---------------------------Dimensions---------------------------------------------------------------
vehicle.dimensions.GX.vehicle_overhang_r=vehicle.dimensions.p_crash_length(1); % Rear overhang [mm]
vehicle.dimensions.GX.vehicle_overhang_f=vehicle.dimensions.p_crash_length(2); % Front overhang [mm]
vehicle.Input.frameform=Parameters.general_dimensions.frameform; % Frameform (SUV or Sedan) for chassis weight calc
vehicle.topology.frameform=Parameters.general_dimensions.frameform; % Frameform (SUV or Sedan) for exterior weight calc
vehicle.dimensions.p_toleranz=Parameters.package.general_tolerance; % Package tolerance (space between components)

%%---------------------------Interior-------------------------------------------------------------------------
vehicle.manikin.SgRP_X_front_axle =  Parameters.interior.SgRP_X_front_axle;   % x-distance in mm between front axle and SgRP of first row of seats
vehicle.manikin.SgRP2_X_front_axle =   Parameters.interior.SgRP2_X_front_axle; % x-distance in mm between front axle and SgRP of second row of seats
vehicle.dimensions.MP.number_of_seats=sum(Parameters.input.n_passengers); %Total number of seats for update weight
%Write values in vehicle (either raw boundary surface measurements or values for interior calculation)
vehicle.Input.upperbody_length          = Parameters.input.upperbody_length;  % upperbody length
vehicle.Input.angle_br_max              = Parameters.input.angle_br_max    ;  % backrest angle at maximum height
vehicle.Input.angle_br_min              = Parameters.input.angle_br_min    ;  % backrest angle at minimum height
vehicle.Input.seat_depth                = Parameters.input.seat_depth      ;  % seat depth

vehicle.Input.seat_height               = Parameters.input.seat_height     ;  % seat height
vehicle.Input.foot_length               = Parameters.input.foot_length     ;  % foot length
vehicle.Input.leg_overlap               = Parameters.input.leg_overlap     ;  % leg overlap for vav and con
vehicle.Input.backrest_width            = Parameters.input.backrest_width  ;  % backrest width
vehicle.Input.armrest_width_single      = Parameters.input.armrest_width_single   ;  % armrest width single
vehicle.Input.armrest_width_double      = Parameters.input.armrest_width_double   ;  % armrest width
vehicle.Input.seatgap                   = Parameters.input.seatgap         ;  % seatgap between seats of a seatrow
vehicle.Input.wallgap                   = Parameters.input.wallgap         ;  % gap between wall and seat on the outside
vehicle.Input.int_b2b_walldistance      = Parameters.input.int_b2b_walldistance;  % distance between seatrows for btb

vehicle.Input.legroom_additional        = Parameters.input.legroom_additional  ;   % additional legroom
vehicle.Input.seat_adjustment_x         = Parameters.input.seat_adjustment_x   ;     % seat adjustment in x
vehicle.Input.seat_adjustment_z         = Parameters.input.seat_adjustment_z  ;     % seat adjustment in z
vehicle.Input.headroom                  = Parameters.input.headroom      ;                      % headspace
vehicle.Input.thickness_seat            = Parameters.input.thickness_seat ;           % seat thickness
vehicle.Input.thickness_br              = Parameters.input.thickness_br   ;               % backrest thickness
vehicle.Input.length_br                 = Parameters.input.length_br       ;         % backrest length

vehicle.Input.single_ar_outside         = Parameters.input.single_ar_outside  ;     % =1, if single armrest on the outside
vehicle.Input.single_ar_between         = Parameters.input.single_ar_between ;      % =1, if single armrest between the seats
vehicle.Input.double_ar                 = Parameters.input.double_ar       ;        % =1, if double armrest between the seats

%Mode for 3 rows
if isfield(Parameters.input,'int_3rows')
    vehicle.Input.int_3rows                 = Parameters.input.int_3rows;
    vehicle.Input.int_3rows_type            = Parameters.input.int_3rows_type;
    %Correction of fitting interior type
    switch vehicle.Input.int_3rows_type
        case '3con'
            vehicle.Input.int_type          = 'con';
        case '3convav'
            vehicle.Input.int_type          = 'con';
        case '3vavcon'
            vehicle.Input.int_type          = 'vav';
    end
else
    vehicle.Input.int_3rows             = 0; %standard mode
end


%%---------------------------Masses-------------------------------------------------------------------------
vehicle.masses.vehicle_sim_cons_weight  =   NaN;            % Fixed starting weight for simulation
vehicle.masses.payload.vehicle_payload = Parameters.masses.payload.additional_payload; % Payload weight [kg]
vehicle.masses.powertrain.perc_gearbox_steel=Parameters.powertrain.perc_gearbox_steel; % Gearbox steel percentage
vehicle.masses.powertrain.perc_gearbox_alu=Parameters.powertrain.perc_gearbox_alu; % Gearbox aluminium percentage
vehicle.masses.payload.extra_equipement =   Parameters.masses.extra_equipement; % Starting Weight for extra equipement in kg
vehicle.Input.extra_equipement=Parameters.masses.extra_equipement;
vehicle.masses.iterations=[]; % requred for PAUMANI weight calc
%%---------------------------Wheels and axles-----------------------------------------------------------------------------
vehicle.Input.w_tire = (2*vehicle.Input.r_tire*0.2)/0.7;                       % Width of tire (calculated with approximated ratios) for initial LDS
vehicle.dimensions.CX.rim_diameter_r_inch = round((2*vehicle.Input.r_tire*Parameters.wheels.felge2wheel_rat)/25.4); % initial front rim diameter in inch for initial tire calc
vehicle.dimensions.CX.rim_diameter_f_inch = round((2*vehicle.Input.r_tire*Parameters.wheels.felge2wheel_rat)/25.4); % initial rear rim diameter in inch for initial tire calc
vehicle.dimensions.EY.rim_inset_r = Parameters.wheels.rim_inset_r; % Rim inset [mm]
vehicle.dimensions.EZ.max_deflection_r = Parameters.wheels.max_deflection_rim; % Wheel max. deflection [mm]
vehicle.dimensions.p_axle_height = vehicle.Input.r_tire; % Approximated height of axle [mm]
if Parameters.Equipment.tire_type.Value==1
    vehicle.Input.tire_type='EL'; %Extra Load of tires
else
    vehicle.Input.tire_type='NL'; %Normal Load of tires
end


if  vehicle.Input.steering_ratio==1
    vehicle.masses.optional_extras.AWS=0; % No all wheel steering
else
    vehicle.masses.optional_extras.AWS=1; % All wheel steering
end

%% ---------------------------HVAC/Cooling--------------------------------------------------------------------------
% Cooler
vehicle.aux.cooler_type=Parameters.aux.cooler_type; % Front=1 or side cooler=2
vehicle.dimensions.p_cooler_ctr=[0 0 0;Parameters.aux.cooler_dim_ctr.x Parameters.aux.cooler_dim_ctr.y Parameters.aux.cooler_dim_ctr.z]; % Dimensions of center cooler
vehicle.dimensions.p_cooler_side=[0 0 0;Parameters.aux.cooler_dim_side.x Parameters.aux.cooler_dim_side.y Parameters.aux.cooler_dim_side.z]; % Dimensions of side cooler
vehicle.dimensions.p_cooling_distance=Parameters.aux.cooling_distance; % Distance between front bumper and cooler
% HVAC settings
vehicle.Input.HVAC_calc     = Parameters.input.HVAC_calc;      %HVAC calculation switch
if vehicle.Input.HVAC_calc
    vehicle.Input.HVAC_t_amb    = Parameters.input.HVAC_t_amb;     %ambient temperature in °C
    vehicle.Input.HVAC_scenario = Parameters.input.HVAC_scenario;  %weather scenario (name)
    vehicle.Input.HVAC_m_flow   = Parameters.input.HVAC_m_flow;    %massflow rate in kg/min    
    vehicle.Input.HVAC_perc_ca  = Parameters.input.HVAC_perc_circair; %Percentage of circulated air in %/100
    vehicle.Input.HVAC_T_set    = Parameters.input.HVAC_T_set;    %Temperature set (only Winter)
end
%% ---------------------------Powerelectronics---------------------------------------------------------------
vehicle.dimensions.p_powerel{1}=[0 0 0;Parameters.powertrain.powerel_dim.x Parameters.powertrain.powerel_dim.y Parameters.powertrain.powerel_dim.z]; % Dimensions of front powerelectronics
vehicle.dimensions.p_powerel{2}=[0 0 0;Parameters.powertrain.powerel_dim.x Parameters.powertrain.powerel_dim.y Parameters.powertrain.powerel_dim.z]; % Dimensions of rear powerelectronics

%% ---------------------------Trunk----------------------------------------------------------------------------
vehicle.dimensions.trunk_vol_front  =   10e5*vehicle.Input.trunk_ratio*Parameters.input.trunk_volume; % Front trunk volume [mm3]
vehicle.dimensions.trunk_vol_rear   =   10e5*(1-vehicle.Input.trunk_ratio)*Parameters.input.trunk_volume; % Rear trunk volume [mm3]
vehicle.Input.trunk_volume          =   Parameters.input.trunk_volume; %Total trunk volume

%% ---------------------------Extras-------------------------------------------------------------------
vehicle.masses.optional_extras.cluster_type=        Parameters.extras.cluster_type; %Instrument cluster behind steering wheel. 'D'=digital; 'A'=analog
vehicle.masses.optional_extras.infotainment=        Parameters.extras.infotainment; % Infotainment (yes=1, no=0)
vehicle.masses.optional_extras.HUD=                 Parameters.extras.HUD; % HUD (yes=1, no=0)
vehicle.masses.optional_extras.subwoofer=           Parameters.extras.subwoofer; % Subwoofer (yes=1, no=0)
vehicle.masses.optional_extras.doors_number=        Parameters.input.n_doors; % Number of doors
vehicle.masses.optional_extras.hood_material=       Parameters.masses.material.hood; % Hood Material (aluminium or steel)
vehicle.masses.optional_extras.doors_material=      Parameters.masses.material.door; % Door Material (aluminium or steel)
vehicle.masses.optional_extras.tailgate_material=   Parameters.masses.material.tailgate; % Tailgate Material (aluminium or steel)
vehicle.masses.optional_extras.headlights_tech=     Parameters.Equipment.headlight_tech; % Headlight technology (LED, Halogen or Xenon)
vehicle.masses.optional_extras.fenders_material=    Parameters.masses.material.fenders; % Fenders Material (aluminium or steel)
vehicle.masses.optional_extras.panorama_roof=       Parameters.Equipment.panorama_roof; % Roof Material (aluminium or steel)
vehicle.masses.optional_extras.sliding_roof=        Parameters.Equipment.sliding_roof; % Sliding roof (yes=1, no=0)
vehicle.masses.optional_extras.spare_tire=          Parameters.Equipment.spare_tire; % Spare tire (yes=1, no=0)
vehicle.masses.optional_extras.park_assist=         Parameters.Equipment.park_assist; % Park assist (yes=1, no=0)
vehicle.masses.optional_extras.ACC=                 Parameters.Equipment.acc; % ACC technology (yes=1, no=0)
vehicle.masses.optional_extras.BSM=                 Parameters.Equipment.bsm; % BSM technology (yes=1, no=0)
vehicle.masses.optional_extras.LKS=                 Parameters.Equipment.lks; % LKS technology (yes=1, no=0)
vehicle.masses.optional_extras.keyless=             Parameters.Equipment.keyless_go; % Keyless go (yes=1, no=0)
vehicle.masses.optional_extras.night_vision=        Parameters.Equipment.night_vision; % Night vision (yes=1, no=0)
vehicle.masses.optional_extras.trunk_assist=        Parameters.Equipment.trunk_assist; % Trunk assist (yes=1, no=0)
vehicle.masses.optional_extras.phone_connectivity=  Parameters.Equipment.phone_connectivity; % Phone connectivity (yes=1, no=0)
vehicle.masses.optional_extras.alu_perc_BIW=        Parameters.input.biw_alu_perc; % Percentage of aluminium in BIW [%]
vehicle.masses.optional_extras.tow_hitch=           Parameters.extras.tow_hitch; % Tow hitch (yes=1, no=0)
vehicle.masses.optional_extras.AS=                  Parameters.extras.AS; %Air Suspension (yes=1, no=0)

%% ---------------------------Equipment-------------------------------------------------------------------
vehicle.Equipment.Powertrain.driveshaft.driveshaft_mass=                            Parameters.Equipment.Powertrain.driveshaft.driveshaft_mass; % Weight of drive-shaft [kg]
vehicle.Equipment.Powertrain.differential.material_distribution.alu=                Parameters.Equipment.Powertrain.differential.material_distribution.alu; % Percentage of aluminium in differential [%]
vehicle.Equipment.Powertrain.differential.material_distribution.steel=              Parameters.Equipment.Powertrain.differential.material_distribution.steel; % Percentage of steel in differential [%]
vehicle.Equipment.Powertrain.differential.mass_differential=                        Parameters.Equipment.Powertrain.differential.mass_differential; % Weight of differential [kg]
vehicle.Equipment.Powertrain.wheels.Alu_perc_rim=                                   Parameters.Equipment.Powertrain.wheels.Alu_perc_rim; % Percentage of aluminium in rim [%]
vehicle.Equipment.Powertrain.wheels.Steel_perc_rim=                                 Parameters.Equipment.Powertrain.wheels.Steel_perc_rim; % Percentage of steel in rim [%]
vehicle.Equipment.Electronics.front_headlight=                                      Parameters.Equipment.Electronics.front_headlight; % Numbers of front headlights [-]
vehicle.Equipment.Electronics.back_headlight=                                       Parameters.Equipment.Electronics.back_headlight; % Numbers of back headlights [-]
vehicle.Equipment.Electronics.back_up_battery=                                      Parameters.Equipment.Electronics.back_up_battery; % Back up battery (yes=1, no=0)
vehicle.Equipment.Exterior.Interior_panelling_Weight=                               Parameters.Equipment.Exterior.Interior_panelling_Weight; % Weight of interior panels [kg]
vehicle.Equipment.Exterior.Closures_Material_Distribution.Interior_panelling_abs=   Parameters.Equipment.Exterior.Closures_Material_Distribution.Interior_panelling_abs;   %Percentage of abs in interior [%]
vehicle.Equipment.Exterior.Closures_Material_Distribution.Interior_panelling_pp=    Parameters.Equipment.Exterior.Closures_Material_Distribution.Interior_panelling_pp; %Percentage of pp in interior [%]
vehicle.Equipment.Assembly.h_assembly=                                              Parameters.Equipment.Assembly.h_assembly; % Hours of assembly [h]
vehicle.Equipment.Sensors.C2X=                                                      Parameters.Equipment.Sensors.C2X; % Car-to-X technology (yes=1, no=0)
vehicle.Equipment.Sensors.FiveG=                                                    Parameters.Equipment.Sensors.FiveG; % 5-G Technology (yes=1, no=0)
vehicle.Equipment.Sensors.GPS.GPS_Correction_Service=                               Parameters.Equipment.Sensors.GPS.GPS_Correction_Service; % GPS correction device (yes=1, no=0)
vehicle.Equipment.Sensors.GPS.GPS_receiver=                                         Parameters.Equipment.Sensors.GPS.GPS_receiver; % GPS reciever (yes=1, no=0)
vehicle.Equipment.Sensors.Computer=                                                 Parameters.Equipment.Sensors.Computer; % On-board computer (yes=1, no=0)
vehicle.Equipment.Sensors.Front_Camera=                                             Parameters.Equipment.Sensors.Front_Camera; % Number of Front cameras [-]
vehicle.Equipment.Sensors.Surround_Camera=                                          Parameters.Equipment.Sensors.Surround_Camera; % Number of surround cameras [-]
vehicle.Equipment.Sensors.Lidar.n_lidar_mems=                                       Parameters.Equipment.Sensors.Lidar.n_lidar_mems; % Number of MEMS Lidar [-]
vehicle.Equipment.Sensors.Lidar.n_lidar_mech=                                       Parameters.Equipment.Sensors.Lidar.n_lidar_mech; % Number of MECH Lidar [-]
vehicle.Equipment.Sensors.Radar=                                                    Parameters.Equipment.Sensors.Radar; % Number of radars [-]
vehicle.Equipment.Sensors.Sonar=                                                    Parameters.Equipment.Sensors.Sonar; % Number of sonars [-]
vehicle.Equipment.Exterior.Closures_Material_Distribution.alu_closures=             Parameters.Equipment.Exterior.Closures_Material_Distribution.alu_closures; % Percentage of aluminium in closures [%]
vehicle.Equipment.Exterior.Closures_Material_Distribution.steel_closures=           Parameters.Equipment.Exterior.Closures_Material_Distribution.steel_closures; % Percentage of aluminium in closures [%]
vehicle.Equipment.Exterior.n_windshield_wiper=                                      Parameters.Equipment.Exterior.n_windshield_wiper; % Number of windshield wipers [-]
vehicle.Equipment.Exterior.n_windows_lifter=                                        Parameters.Equipment.Exterior.n_windows_lifter; % Number of window lifter [-]
vehicle.Equipment.Exterior.Pneumatic_doors=                                         Parameters.Equipment.Exterior.Pneumatic_doors; % Pneumatic doors (yes=1, no=0)
vehicle.Equipment.Interior.HVAC=                                                    Parameters.Equipment.Interior.HVAC; % HVAC (yes=1, no=0)
vehicle.Equipment.Interior.n_displays=                                              Parameters.Equipment.Interior.n_displays; % Number of displays in interior [-]
vehicle.Equipment.Interior.n_airbags=                                               Parameters.Equipment.Interior.n_airbags; % Number of front airbags [-]
vehicle.Equipment.Interior.n_side_airbags=                                          Parameters.Equipment.Interior.n_side_airbags; % Number of side airbags {-]
vehicle.Equipment.Interior.n_seat_warmers=                                          Parameters.Equipment.Interior.n_seat_warmers; % Number of seat warmers [-]
vehicle.Equipment.Interior.seat_type=                                               Parameters.Equipment.Interior.seat_type; % Seat type (automatic ('aut') or manual adjustable ('mech'))
vehicle.Equipment.Powertrain.on_board_charger=                                      Parameters.Equipment.Powertrain.on_board_charger; %On-board charger (yes=1, no=0)
vehicle.Equipment.Powertrain.electromech_steering=                                  Parameters.Equipment.Powertrain.electromech_steering; %Electromech steering (yes=1, no=0)
vehicle.Equipment.Powertrain.steer_by_wire=                                         Parameters.Equipment.Powertrain.steer_by_wire; % Steer-by-wire (yes=1, no=0)
vehicle.Equipment.Powertrain.electromech_braking=                                   Parameters.Equipment.Powertrain.electromech_braking; % Electromech braking (yes=1, no=0)

%% 5. First LDS
[vehicle,Parameters] = Non_Linear_Constraints(vehicle,Parameters); % check non-linear constraints

if vehicle.iteration == 1 % Perform first LDS only if non-lin. constraints are satisfied
    
    %%------------Calculate Interior boundaries to aproximate vehicle dimensions----------------------------
    [vehicle] = Package_Interior_Calc(vehicle,Parameters);
    if vehicle.Error>0
        return
    end    
    [vehicle] = Package_Interior(vehicle);    

    % Move the boundaries vertically to achieve interior height 
    height_ibf=ceil(length(vehicle.interior.int_boundary_f)+vehicle.Input.int_height); % Max. z-value + req. interior height
    height_ibr=ceil(length(vehicle.interior.int_boundary_r)+vehicle.Input.int_height); % Max. z-value + req. interior height
    
    % Define vehicle height as max. value in z of boundaries
    if Parameters.Equipment.sliding_roof==1
        d_roof=Parameters.dimensions.CZ.sliding_roof_thickness; %roof thickness for sliding roof
    elseif Parameters.Equipment.panorama_roof==1
        d_roof=Parameters.dimensions.CZ.panoramic_roof_thickness; %roof thickness for panoramic roof
    else
        d_roof=Parameters.dimensions.CZ.roof_thickness; %roof thickness for normal roof
    end
    vehicle.dimensions.GZ.vehicle_height=max([height_ibf height_ibr])+d_roof; % Max of interior boundaries + top space
    
    % Determine min an max values in x of interior
    min_x_ibf=min([vehicle.interior.int_boundary_r vehicle.interior.int_boundary_f2]); % min --> negative direction of x
    max_x_ibr=max([vehicle.interior.int_boundary_r vehicle.interior.int_boundary_r2]); % max --> positive direction of x
    
    % Approximate the wheelbase as length of interior (distance between min. front and max rear)
    vehicle.dimensions.GX.wheelbase = abs(min_x_ibf)+max_x_ibr;
    
    % Calculate width
    vehicle.dimensions.GY.vehicle_width = vehicle.interior.int_y_tot+2*vehicle.dimensions.p_crash_length(3); % Total vehicle width [mm]
    
    % Calculate length (overhang is initialized as temp until first calculation of Package_Front and Package_Rear)
    vehicle.dimensions.GX.vehicle_length = vehicle.dimensions.GX.wheelbase+vehicle.dimensions.GX.vehicle_overhang_r+vehicle.dimensions.GX.vehicle_overhang_f; % Total vehicle length [mm]
    
    %Calculate empirical reduced weight (no battery included) in kg (see Paper Nicoletti & Felgenhauer)
    volume=vehicle.dimensions.GZ.vehicle_height*vehicle.dimensions.GY.vehicle_width*(vehicle.dimensions.GX.vehicle_length)/(1000^3); %Volume for initial vehicle (length*width*height)in m^2
    reduced_weight = -169.497+139.463*volume; %Regression from Paper
    vehicle.masses.vehicle_empty_weight = reduced_weight; % Empty vehicle weight with EU calculation [kg]
    vehicle.masses.vehicle_empty_weight_EU = reduced_weight+75; % Empty vehicle weight with EU calculation [kg]
    vehicle.masses.vehicle_max_weight = reduced_weight +...
    Parameters.masses.payload.weight_passenger*sum(Parameters.input.n_passengers); % Max vehicle weight [kg]
    
    [vehicle] = calc_longitudinal_simulation(vehicle,Parameters);
    [vehicle] = store_temp_data(vehicle); % save current data for comparison in next iteration
end
end
