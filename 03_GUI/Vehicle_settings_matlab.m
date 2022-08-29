function app=Vehicle_settings_matlab(app,sett_aut,sett_conv)
%% Description:
%Set userform settings depending on selected value
    %% Fixed Vehicle Parameters
    %Complete vehicle
    app.input_turning_diameter.Value    = 11; % Maximum steering radius [m]
    app.input_free_crash_length.Value   = 500;
    app.input_free_crash_length_2.Value = 400;
    app.input_free_crash_length_3.Value = 250; % Free crash length (mm)
    app.input_trunk_volume.Value        = 0; % Total trunk volume (front + rear) [L]
    app.input_ground_clearance.Value    = 'flatfloor'; %Ground clearance
    
    % Powertrain requirements/settings
    app.input_driving_cycle.Value       = 'WLTP'; % driving cycle
    app.input_maximal_acceleration.Value= 10; %Required/Desired acceleration [s]
    app.input_maximal_speed.Value       = 150; % Maximum velocity [km/h]
    app.input_range.Value               = 300; % Minimum range [km]
    app.input_tire_load.Value           = 1; %Extra Load Tires
    
    %% HVAC settings
    app.HVAC_calc.Value                 = 0;    %Temperature set (only Winter)
    
    %% Interior
    % H30 Optimization (1=optimization, 0=no H30 optimization)
    app.input_H30Opt.Value              = 0;
    
    %Seating layout and doors
    app.input_seating_layout.Value      = 'Vis-a-Vis';
    app.input_n_doors.Value             = 4;
    app.input_int_b2b_walldistance.Value= 200; %only for back to back, distance between backrests
    app.input_int_x_overlap.Value       = 100; %Overlap of legs
    app.input_int_usecase.Value         = 'Individual'; 
    app.input_int_work.Value            = 0;
    app.input_int_sleep.Value           = 0;
    
    % Passenger distribution depended on amount of rows
    app.input_seats_row_1.Value         = 2;
    app.input_seats_row_2.Value         = 2;
    
    app.input_int_single_ar_outside_1.Value         = 1;
    app.input_int_single_ar_outside_2.Value         = 1; % =1, if single armrest on the outside
    app.input_int_single_ar_between_1.Value         = 1;
    app.input_int_single_ar_between_2.Value         = 1; % =1, if single armrest between the seats
    app.input_int_double_ar_1.Value                 = 0;
    app.input_int_double_ar_2.Value                 = 0; % =1, if double armrest between the seats
    
    app.input_int_upperbody_length_1.Value          = 980;
    app.input_int_upperbody_length_2.Value          = 980;  % upperbody length
    app.input_int_angle_br_max_1.Value              = 80;
    app.input_int_angle_br_max_2.Value              = 80;   % backrest angle at maximum height
    app.input_int_angle_br_min_1.Value              = 70;
    app.input_int_angle_br_min_2.Value              = 70;   % backrest angle at minimum height
    app.input_int_seat_depth_1.Value                = 500;
    app.input_int_seat_depth_2.Value                = 500;   % seat depth
    
    app.input_int_seat_height_1.Value               = 400;
    app.input_int_seat_height_2.Value               = 400;    % seat height
    app.input_int_foot_length_1.Value               = 310;
    app.input_int_foot_length_2.Value               = 310;      % foot length
    app.input_int_backrest_width_1.Value            = 530;
    app.input_int_backrest_width_2.Value            = 530;  % backrest width
    app.input_int_armrest_width_single_1.Value      = 80;
    app.input_int_armrest_width_single_2.Value      = 80;% armrest width single
    app.input_int_armrest_width_double_1.Value      = 200;
    app.input_int_armrest_width_double_2.Value      = 200;% armrest width double
    app.input_int_seatgap_1.Value                   = 40;
    app.input_int_seatgap_2.Value                   = 40;      % seatgap between seats of a seatrow
    app.input_int_gaptowall_1.Value                 = 40;
    app.input_int_gaptowall_2.Value                 = 40;      % gap between wall and seat on the outside
    
    app.input_int_legroom_additional_1.Value        = 200;
    app.input_int_legroom_additional_2.Value        = 200;% additional legroom
    app.input_int_seat_adjustment_x_1.Value         = 0;
    app.input_int_seat_adjustment_x_2.Value         = 0;% seat adjustment in x
    app.input_int_seat_adjustment_z_1.Value         = 0;
    app.input_int_seat_adjustment_z_2.Value         = 0;    % seat adjustment in z
    app.input_int_headroom_1.Value                  = 150;
    app.input_int_headroom_2.Value                  = 150;    % headspace
    app.input_int_thickness_seat_1.Value            = 100;
    app.input_int_thickness_seat_2.Value            = 100; % seat thickness
    app.input_int_thickness_br_1.Value              = 100;
    app.input_int_thickness_br_2.Value              = 100; % backrest thickness
    app.input_int_backrest_length_1.Value           = 800;
    app.input_int_backrest_length_2.Value           = 800;% backrest length
   
    %% Equipment
    app.input_mech_lidar.Value          = 2;
    app.input_mems_lidar.Value          = 0;
    app.input_sonar.Value               = 8;
    app.input_surround_cameras.Value    = 4;
    app.input_front_cameras.Value       = 4;
    app.input_c2x.Value                 = 1;
    app.input_5g.Value                  = 1;
    app.input_onboard_pc.Value          = 1;
    app.input_gps_reciever.Value        = 1;
    app.input_gps_correction.Value      = 0;
    app.input_displays.Value            = 2;
    app.input_hvac.Value                = 1;
    app.input_side_airbags.Value        = 4;
    app.input_pneumatic_doors.Value     = 0;
    app.input_emech_steering.Value      = 1;
    app.input_sbw.Value                 = 1;
    app.input_emech_brakes.Value        = 1;
    app.input_obc.Value                 = 1;
    app.input_window_lift.Value         = 1;
    app.input_adjustable_seats.Value    = 1;
    app.input_seat_warmers.Value        = 1;
    app.input_headlight_tech.Value      = 'LED';
    app.input_phone_connectivity.Value  = 1;
    app.input_trunk_assist.Value        = 1;
    app.input_night_vision.Value        = 0;
    app.input_park_assist.Value         = 1;
    app.input_bsm.Value                 = 1;
    app.input_lks.Value                 = 1;
    app.input_acc.Value                 = 1;
    app.input_keyless_go.Value          = 1;
    app.input_spare_tire.Value          = 0;
    app.input_panorama_roof.Value       = 1;
    app.input_sliding_roof.Value        = 0;
    app.input_tire_load.Value           = 1;    
    
    %% Optimization 
    %Differentiate between single vehicle calculation or optimization
    if app.sing_veh.Value==1
        % -------------Engine-------------
        % Topology type
        app.input_drive_type.Value          = 'X_GM';
        
        % Engine type (PSM or ASM)
        app.input_engine_type.Value         = 'PSM';
        
        % Engine-gearbox angle front [deg]
        app.input_engine_angle_front_min.Value =0;
        app.input_engine_angle_front_max.Value =0;
        
        % Engine-gearbox angle rear [deg]
        app.input_engine_angle_rear_min.Value =135;
        app.input_engine_angle_rear_max.Value =180;
        
        % -------------Gearbox-------------
        % Gear ratio 1 rear [-]
        app.input_gear_ratio_1_front_max.Value = 10.0;
        app.input_gear_ratio_1_front_min.Value = 10.0;
        
        % Gear ratio 1 rear [-]
        app.input_gear_ratio_1_rear_max.Value  = 10.0;
        app.input_gear_ratio_1_rear_min.Value  = 10.0;
        
        % Gear type front (coaxial or parallel)
        app.input_gear_type_front.Value        = 'Parallel';
        
        % Gear type rear (coaxial or parallel)
        app.input_gear_type_rear.Value         = 'Parallel';
        
        % Gearbox optimization goal (length or height)
        app.input_gearbox_opt.Value            = 'Length';
        
        % -------------Axle-------------
        % Rear axle type
        app.input_rear_axis_type.Value      = 'five_link';
        
        % Axle-engine angle (xy) [deg]
        app.input_axle_angle_1_min.Value    = 0;
        app.input_axle_angle_1_max.Value    = 0;
        
        % Axle-engine angle (xz) [deg]
        app.input_axle_angle_2_min.Value    = 0;
        app.input_axle_angle_2_max.Value    = 0;
        
        % -------------Battery-------------
        % Battery cell type (Pouch, cylindrical or prismatic)
        app.input_cell_type.Value           = 'pouch';
        
        % Small Overlap Second Level Battery
        app.input_small_overlap.Value       = 'No';
        
        % -------------Exterior-------------
        app.input_window_angle_front_min.Value  = 20;
        app.input_window_angle_rear_min.Value   = 15;
        app.input_trunk_type_front.Value        = 'integrated';
        app.input_trunk_type_rear.Value         = 'integrated';
                       
        % Distance between wheelhouse and bumper [mm]
        app.input_dist_wheelhouse2bumper_f_min.Value = 100;
        app.input_dist_wheelhouse2bumper_f_max.Value = 100;
        app.input_dist_wheelhouse2bumper_r_min.Value = 100;
        app.input_dist_wheelhouse2bumper_r_max.Value = 100;
        
        % Distance between wheelhouse and bumper [mm]
        app.input_dist_int2fw_min.Value = 50;
        app.input_dist_int2fw_max.Value = 50;
        app.input_dist_int2rw_min.Value = 50;
        app.input_dist_int2rw_max.Value = 50;
        
        % -------------Interior-------------
        % Trunk volume ratio (front/total) [%]
        app.input_trunk_ratio_min.Value       = 40;
        app.input_trunk_ratio_max.Value       = 40;
        
        % Interior height [mm]
        app.input_int_height_min.Value        = 420;
        app.input_int_height_max.Value        = 420;
        
        % H30 Optimization
        app.input_minH30_front.Value                    = 250;
        app.input_maxH30_front.Value                    = 250;
        app.input_minH30_rear.Value                     = 250;
        app.input_maxH30_rear.Value                     = 300;
        
        % -------------Others-------------
        % Tire diameter [mm]
        app.input_tire_diam_min.Value       = 700;
        app.input_tire_diam_max.Value       = 700;
        
        % Steering Ratio Front/Total [%]
        app.input_steering_ratio_min.Value          = 70;
        app.input_steering_ratio_max.Value          = 70;
        
        % Steering Ratio Front/Total [%]
        app.input_torque_ratio_min.Value            = 30;
        app.input_torque_ratio_max.Value            = 30;
    
    else %Optimization is selected
        % -------------Engine-------------
        % Topology type
        app.input_drive_type.Value          = 'All Combinations';
        
        % Engine type (PSM or ASM)
        app.input_engine_type.Value         = 'PSM';
        
        % Engine-gearbox angle front [deg]
        app.input_engine_angle_front_min.Value =0;
        app.input_engine_angle_front_max.Value =180;
        
        % Engine-gearbox angle rear [deg]
        app.input_engine_angle_rear_min.Value =0;
        app.input_engine_angle_rear_max.Value =180;
        
        % -------------Gearbox-------------
        % Gear ratio 1 rear [-]
        app.input_gear_ratio_1_front_min.Value = 6.0;
        app.input_gear_ratio_1_front_max.Value = 10.0;        
        
        % Gear ratio 1 rear [-]
        app.input_gear_ratio_1_rear_min.Value  = 6.0;
        app.input_gear_ratio_1_rear_max.Value  = 10.0;        
        
        % Gear type front (coaxial or parallel)
        app.input_gear_type_front.Value        = 'All Combinations';
        
        % Gear type rear (coaxial or parallel)
        app.input_gear_type_rear.Value         = 'All Combinations';
        
        % Gearbox optimization goal (length or height)
        app.input_gearbox_opt.Value            = 'All Combinations';
        
        % -------------Axle-------------
        % Rear axle type
        app.input_rear_axis_type.Value      = 'All Combinations';
        
        % Axle-engine angle (xy) [deg]
        app.input_axle_angle_1_min.Value    = 0;
        app.input_axle_angle_1_max.Value    = 13;
        
        % Axle-engine angle (xz) [deg]
        app.input_axle_angle_2_min.Value    = 0;
        app.input_axle_angle_2_max.Value    = 13;
        
        % -------------Battery-------------
        % Battery cell type (Pouch, cylindrical or prismatic)
        app.input_cell_type.Value           = 'All Combinations';
        
        % Small Overlap Second Level Battery
        app.input_small_overlap.Value       = 'All Combinations';
        
        % -------------Exterior-------------
        app.input_window_angle_front_min.Value  = 5;
        app.input_window_angle_front_max.Value  = 85;
        app.input_window_angle_rear_min.Value   = 5;
        app.input_window_angle_rear_max.Value   = 85;
        app.input_trunk_type_front.Value        = 'All Combinations';
        app.input_trunk_type_rear.Value         = 'All Combinations';
        
        % Distance between wheelhouse and bumper [mm]
        app.input_dist_wheelhouse2bumper_f_min.Value = 0;
        app.input_dist_wheelhouse2bumper_f_max.Value = 600;
        app.input_dist_wheelhouse2bumper_r_min.Value = 0;
        app.input_dist_wheelhouse2bumper_r_max.Value = 600;
        
        % Distance between wheelhouse and bumper [mm]
        app.input_dist_int2fw_min.Value = 0;
        app.input_dist_int2fw_max.Value = 50;
        app.input_dist_int2rw_min.Value = 0;
        app.input_dist_int2rw_max.Value = 50;
        
        % -------------Interior-------------
        % Trunk volume ratio (front/total) [%]
        app.input_trunk_ratio_min.Value       = 0;
        app.input_trunk_ratio_max.Value       = 100;
        
        % Interior height [mm]
        app.input_int_height_min.Value        = 220;
        app.input_int_height_max.Value        = 500;
        
        % H30 Optimization
        app.input_minH30_front.Value                    = 250;
        app.input_maxH30_front.Value                    = 700;
        app.input_minH30_rear.Value                     = 250;
        app.input_maxH30_rear.Value                     = 700;
        
        % -------------Others-------------
        % Tire diameter [mm]
        app.input_tire_diam_min.Value       = 600;
        app.input_tire_diam_max.Value       = 800;
                
        % Steering Ratio Front/Total [%]
        app.input_steering_ratio_min.Value          = 50;
        app.input_steering_ratio_max.Value          = 100;
        
        % Steering Ratio Front/Total [%]
        app.input_torque_ratio_min.Value            = 5;
        app.input_torque_ratio_max.Value            = 95;
        
    end

%Change specific values for conventional or autonomous pre-setting
if sett_conv
    %% Seating layout and doors
    app.input_seating_layout.Value      = 'Conventional';
    app.input_n_doors.Value             = 4;
    
    %% Passenger distribution depended on amount of rows
    app.input_seats_row_1.Value         = 2;
    app.input_seats_row_2.Value         = 3;
    
    %% Fixed Vehicle Parameters
    app.input_turning_diameter.Value    = 11; % Maximum steering radius [m]
    app.input_free_crash_length.Value   = 800;
    app.input_free_crash_length_2.Value = 350;
    app.input_free_crash_length_3.Value = 160; % Free crash length (mm)
    app.input_trunk_volume.Value        = 0;%350; % Total trunk volume (front + rear) [L]
    
    %% Powertrain requirements/settings
    app.input_driving_cycle.Value       = 'WLTP'; % driving cycle
    app.input_maximal_acceleration.Value= 7.3; %Required/Desired acceleration [s]
    app.input_maximal_speed.Value       = 160; % Maximum velocity [km/h]
    app.input_range.Value               = 420; % Minimum range [km]
    
    %% HVAC settings
    app.HVAC_calc.Value                 = 0;    %Temperature set (only Winter)
    
    
    %% Engine
    % Topology type
    app.input_drive_type.Value          = 'X_GM';
    
    % Engine type (PSM or ASM)
    app.input_engine_type.Value         = 'PSM';
    
    % Engine-gearbox angle front [deg]
    app.input_engine_angle_front_min.Value =0;
    app.input_engine_angle_front_max.Value =0;
    
    % Engine-gearbox angle rear [deg]
    app.input_engine_angle_rear_min.Value =0;
    app.input_engine_angle_rear_max.Value =0;
    
    %% Axle
    % Rear axle type
    app.input_rear_axis_type.Value      = 'five_link';
    
    % Axle-engine angle (xy) [deg]
    app.input_axle_angle_1_min.Value    = 0;
    app.input_axle_angle_1_max.Value    = 0;
    
    % Axle-engine angle (xz) [deg]
    app.input_axle_angle_2_min.Value    = 0;
    app.input_axle_angle_2_max.Value    = 0;
    
    %% Gearbox
    % Gear ratio 1 rear [-]
    app.input_gear_ratio_1_front_max.Value = 11.53;
    app.input_gear_ratio_1_front_min.Value = 11.53;
    
    % Gear ratio 1 rear [-]
    app.input_gear_ratio_1_rear_max.Value  = 11.53;
    app.input_gear_ratio_1_rear_min.Value  = 11.53;
    
    % Gear type front (coaxial or parallel)
    app.input_gear_type_front.Value        = 'Parallel';
    
    % Gear type rear (coaxial or parallel)
    app.input_gear_type_rear.Value         = 'Parallel';
    
    % Gearbox optimization goal (length or height)
    app.input_gearbox_opt.Value            = 'Length';
    
    %% Interior
    % Trunk volume ratio (front/total) [%]
    app.input_trunk_ratio_min.Value       = 0;
    app.input_trunk_ratio_max.Value       = 0;
    
    % Interior height [mm]
    app.input_int_height_min.Value        = 370;
    app.input_int_height_max.Value        = 370;
    
    % H30 Optimization (1=optimization, 0=no H30 optimization)
    app.input_H30Opt.Value              = 0;
    
    %% Exterior
    app.input_window_angle_front_min.Value  = 60;
    app.input_window_angle_rear_min.Value   = 50;
    app.input_trunk_type_front.Value        = 'hooded';
    app.input_trunk_type_rear.Value         = 'integrated';
    
    %% Battery
    % Battery cell type (Pouch, cylindrical or prismatic)
    app.input_cell_type.Value           = 'pouch';
    
    % Small Overlap Second Level Battery
    app.input_small_overlap.Value       = 'No';
    
    %% Others
    % Tire diameter [mm]
    app.input_tire_diam_min.Value       = 701;
    app.input_tire_diam_max.Value       = 701;
    
    % Distance between wheelhouse and bumper [mm]
    app.input_dist_wheelhouse2bumper_f_min.Value = 172;
    app.input_dist_wheelhouse2bumper_f_max.Value = 172;
    app.input_dist_wheelhouse2bumper_r_min.Value = 247;
    app.input_dist_wheelhouse2bumper_r_max.Value = 247;
    
    % Steering Ratio Front/Total [%]
    app.input_steering_ratio_min.Value          = 100;
    app.input_steering_ratio_max.Value          = 100;
    
    % Steering Ratio Front/Total [%]
    app.input_torque_ratio_min.Value            = 30;
    app.input_torque_ratio_max.Value            = 30;
    
    %% Interior
    app.input_int_upperbody_length_1.Value          = 850;
    app.input_int_upperbody_length_2.Value          = 729;  % upperbody length
    app.input_int_angle_br_max_1.Value              = 70;
    app.input_int_angle_br_max_2.Value              = 65;   % backrest angle at maximum height
    app.input_int_angle_br_min_1.Value              = 70;
    app.input_int_angle_br_min_2.Value              = 65;   % backrest angle at minimum height
    app.input_int_seat_depth_1.Value                = 449;
    app.input_int_seat_depth_2.Value                = 435;   % seat depth
    
    app.input_int_seat_height_1.Value               = 200;
    app.input_int_seat_height_2.Value               = 250;    % seat height
    app.input_int_foot_length_1.Value               = 258;
    app.input_int_foot_length_2.Value               = 0;      % foot length
    app.input_int_x_overlap.Value                   = 308;         % leg overlap for vav and con
    app.input_int_backrest_width_1.Value            = 562;
    app.input_int_backrest_width_2.Value            = 373;  % backrest width
    app.input_int_armrest_width_single_1.Value      = 69;
    app.input_int_armrest_width_single_2.Value      = 56;% armrest width single
    app.input_int_armrest_width_double_1.Value      = 200;
    app.input_int_armrest_width_double_2.Value      = 0;% armrest width double
    app.input_int_seatgap_1.Value                   = 0;
    app.input_int_seatgap_2.Value                   = 0;      % seatgap between seats of a seatrow
    app.input_int_gaptowall_1.Value                 = 46;
    app.input_int_gaptowall_2.Value                 = 77;      % gap between wall and seat on the outside
    app.input_int_b2b_walldistance.Value            = 0;                                         % distance between seatrows for btb
    
    app.input_int_legroom_additional_1.Value        = 250;
    app.input_int_legroom_additional_2.Value        = 260;% additional legroom
    app.input_int_seat_adjustment_x_1.Value         = 0;
    app.input_int_seat_adjustment_x_2.Value         = 0;% seat adjustment in x
    app.input_int_seat_adjustment_z_1.Value         = 50;
    app.input_int_seat_adjustment_z_2.Value         = 0;    % seat adjustment in z
    app.input_int_headroom_1.Value                  = 160;
    app.input_int_headroom_2.Value                  = 100;    % headspace
    app.input_int_thickness_seat_1.Value            = 109;
    app.input_int_thickness_seat_2.Value            = 160; % seat thickness
    app.input_int_thickness_br_1.Value              = 109;
    app.input_int_thickness_br_2.Value              = 86; % backrest thickness
    app.input_int_backrest_length_1.Value           = 850;
    app.input_int_backrest_length_2.Value           = 729;% backrest length
    
    app.input_int_single_ar_outside_1.Value         = 1;
    app.input_int_single_ar_outside_2.Value         = 1; % =1, if single armrest on the outside
    app.input_int_single_ar_between_1.Value         = 0;
    app.input_int_single_ar_between_2.Value         = 0; % =1, if single armrest between the seats
    app.input_int_double_ar_1.Value                 = 1;
    app.input_int_double_ar_2.Value                 = 0; % =1, if double armrest between the seats
    
    app.input_minH30_front.Value                    = 250;
    app.input_maxH30_front.Value                    = 250;
    app.input_minH30_rear.Value                     = 250;
    app.input_maxH30_rear.Value                     = 300;
    
    
    %% Equipment
    app.input_mech_lidar.Value          = 0;
    app.input_mems_lidar.Value          = 0;
    app.input_sonar.Value               = 12;
    app.input_surround_cameras.Value    = 1;
    app.input_front_cameras.Value       = 1;
    app.input_c2x.Value                 = 0;
    app.input_5g.Value                  = 0;
    app.input_onboard_pc.Value          = 0;
    app.input_gps_reciever.Value        = 1;
    app.input_gps_correction.Value      = 0;
    app.input_displays.Value            = 1;
    app.input_hvac.Value                = 1;
    app.input_side_airbags.Value        = 4;
    app.input_pneumatic_doors.Value     = 0;
    app.input_emech_steering.Value      = 1;
    app.input_sbw.Value                 = 0;
    app.input_emech_brakes.Value        = 0;
    app.input_obc.Value                 = 1;
    app.input_window_lift.Value         = 1;
    app.input_adjustable_seats.Value    = 1;
    app.input_seat_warmers.Value        = 1;
    app.input_headlight_tech.Value      = 'LED';
    app.input_phone_connectivity.Value  = 1;
    app.input_trunk_assist.Value        = 1;
    app.input_night_vision.Value        = 0;
    app.input_park_assist.Value         = 1;
    app.input_bsm.Value                 = 1;
    app.input_lks.Value                 = 1;
    app.input_acc.Value                 = 1;
    app.input_keyless_go.Value          = 1;
    app.input_spare_tire.Value          = 0;
    app.input_panorama_roof.Value       = 1;
    app.input_sliding_roof.Value        = 0;
    app.input_tire_load.Value           = 1;
       
    elseif sett_aut
        %Seating layout and doors
        app.input_seating_layout.Value      = 'Vis-a-Vis';
        app.input_n_doors.Value             = 4;
        
        % Passenger distribution depended on amount of rows
        app.input_seats_row_1.Value         = 2;
        app.input_seats_row_2.Value         = 2;
        
        %% Fixed Vehicle Parameters
        app.input_turning_diameter.Value    = 11; % Maximum steering radius [m]
        app.input_free_crash_length.Value   = 500;
        app.input_free_crash_length_2.Value = 400;
        app.input_free_crash_length_3.Value = 250; % Free crash length (mm)
        app.input_trunk_volume.Value        = 0;%350; % Total trunk volume (front + rear) [L]
        
        %% Powertrain requirements/settings
        app.input_driving_cycle.Value       = 'WLTP'; % driving cycle
        app.input_maximal_acceleration.Value= 10; %Required/Desired acceleration [s]
        app.input_maximal_speed.Value       = 150; % Maximum velocity [km/h]
        app.input_range.Value               = 250; % Minimum range [km]
        
        %% HVAC settings
        app.HVAC_calc.Value                 = 0;    %Temperature set (only Winter)
        
        
        %% Engine
        % Topology type
        app.input_drive_type.Value          = 'X_GM';
        
        % Engine type (PSM or ASM)
        app.input_engine_type.Value         = 'PSM';
        
        % Engine-gearbox angle front [deg]
        app.input_engine_angle_front_min.Value =70;
        app.input_engine_angle_front_max.Value =0;
        
        % Engine-gearbox angle rear [deg]
        app.input_engine_angle_rear_min.Value =0;
        app.input_engine_angle_rear_max.Value =0;
        
        %% Axle
        % Rear axle type
        app.input_rear_axis_type.Value      = 'torsion_beam';
        
        % Axle-engine angle (xy) [deg]
        app.input_axle_angle_1_min.Value    = 0;
        app.input_axle_angle_1_max.Value    = 0;
        
        % Axle-engine angle (xz) [deg]
        app.input_axle_angle_2_min.Value    = 0;
        app.input_axle_angle_2_max.Value    = 0;
        
        %% Gearbox
        % Gear ratio 1 rear [-]
        app.input_gear_ratio_1_front_max.Value = 10.0;
        app.input_gear_ratio_1_front_min.Value = 10.0;
        
        % Gear ratio 1 rear [-]
        app.input_gear_ratio_1_rear_max.Value  = 10.0;
        app.input_gear_ratio_1_rear_min.Value  = 10.0;
        
        % Gear type front (coaxial or parallel)
        app.input_gear_type_front.Value        = 'Parallel';
        
        % Gear type rear (coaxial or parallel)
        app.input_gear_type_rear.Value         = 'Parallel';
        
        % Gearbox optimization goal (length or height)
        app.input_gearbox_opt.Value            = 'Length';
        
        %% Interior
        % Trunk volume ratio (front/total) [%]
        app.input_trunk_ratio_min.Value       = 0;
        app.input_trunk_ratio_max.Value       = 0;
        
        % Interior height [mm]
        app.input_int_height_min.Value        = 450;
        app.input_int_height_max.Value        = 450;
        
        % H30 Optimization (1=optimization, 0=no H30 optimization)
        app.input_H30Opt.Value              = 0;
        
        %% Exterior
        app.input_window_angle_front_min.Value  = 20;
        app.input_window_angle_rear_min.Value   = 20;
        app.input_trunk_type_front.Value        = 'integrated';
        app.input_trunk_type_rear.Value         = 'integrated';
        
        %% Battery
        % Battery cell type (Pouch, cylindrical or prismatic)
        app.input_cell_type.Value           = 'pouch';
        
        % Small Overlap Second Level Battery
        app.input_small_overlap.Value       = 'No';
        
        %% Others
        % Tire diameter [mm]
        app.input_tire_diam_min.Value       = 700;
        app.input_tire_diam_max.Value       = 700;
        
        % Distance between wheelhouse and bumper [mm]
        app.input_dist_wheelhouse2bumper_f_min.Value = 200;
        app.input_dist_wheelhouse2bumper_f_max.Value = 200;
        app.input_dist_wheelhouse2bumper_r_min.Value = 200;
        app.input_dist_wheelhouse2bumper_r_max.Value = 200;
        
        % Steering Ratio Front/Total [%]
        app.input_steering_ratio_min.Value          = 70;
        app.input_steering_ratio_max.Value          = 70;
        
        % Steering Ratio Front/Total [%]
        app.input_torque_ratio_min.Value            = 30;
        app.input_torque_ratio_max.Value            = 30;
        
        %% Interior
        app.input_int_upperbody_length_1.Value          = 980;
        app.input_int_upperbody_length_2.Value          = 980;  % upperbody length
        app.input_int_angle_br_max_1.Value              = 80;
        app.input_int_angle_br_max_2.Value              = 80;   % backrest angle at maximum height
        app.input_int_angle_br_min_1.Value              = 70;
        app.input_int_angle_br_min_2.Value              = 70;   % backrest angle at minimum height
        app.input_int_seat_depth_1.Value                = 460;
        app.input_int_seat_depth_2.Value                = 460;   % seat depth
        
        app.input_int_seat_height_1.Value               = 400;
        app.input_int_seat_height_2.Value               = 400;    % seat height
        app.input_int_foot_length_1.Value               = 310;
        app.input_int_foot_length_2.Value               = 310;      % foot length
        app.input_int_x_overlap.Value                   = 100;         % leg overlap for vav and con
        app.input_int_backrest_width_1.Value            = 530;
        app.input_int_backrest_width_2.Value            = 530;  % backrest width
        app.input_int_armrest_width_single_1.Value      = 80;
        app.input_int_armrest_width_single_2.Value      = 80;% armrest width single
        app.input_int_armrest_width_double_1.Value      = 200;
        app.input_int_armrest_width_double_2.Value      = 200;% armrest width double
        app.input_int_seatgap_1.Value                   = 40;
        app.input_int_seatgap_2.Value                   = 40;      % seatgap between seats of a seatrow
        app.input_int_gaptowall_1.Value                 = 40;
        app.input_int_gaptowall_2.Value                 = 40;      % gap between wall and seat on the outside
        app.input_int_b2b_walldistance.Value            = 0;                                         % distance between seatrows for btb
        
        app.input_int_legroom_additional_1.Value        = 200;
        app.input_int_legroom_additional_2.Value        = 200;% additional legroom
        app.input_int_seat_adjustment_x_1.Value         = 120;
        app.input_int_seat_adjustment_x_2.Value         = 120;% seat adjustment in x
        app.input_int_seat_adjustment_z_1.Value         = 100;
        app.input_int_seat_adjustment_z_2.Value         = 100;    % seat adjustment in z
        app.input_int_headroom_1.Value                  = 150;
        app.input_int_headroom_2.Value                  = 150;    % headspace
        app.input_int_thickness_seat_1.Value            = 150;
        app.input_int_thickness_seat_2.Value            = 150; % seat thickness
        app.input_int_thickness_br_1.Value              = 150;
        app.input_int_thickness_br_2.Value              = 150; % backrest thickness
        app.input_int_backrest_length_1.Value           = 800;
        app.input_int_backrest_length_2.Value           = 800;% backrest length
        
        app.input_int_single_ar_outside_1.Value         = 1;
        app.input_int_single_ar_outside_2.Value         = 1; % =1, if single armrest on the outside
        app.input_int_single_ar_between_1.Value         = 1;
        app.input_int_single_ar_between_2.Value         = 1; % =1, if single armrest between the seats
        app.input_int_double_ar_1.Value                 = 0;
        app.input_int_double_ar_2.Value                 = 0; % =1, if double armrest between the seats
        
        app.input_minH30_front.Value                    = 250;
        app.input_maxH30_front.Value                    = 250;
        app.input_minH30_rear.Value                     = 250;
        app.input_maxH30_rear.Value                     = 300;
        
        
        %% Equipment
        app.input_mech_lidar.Value          = 2;
        app.input_mems_lidar.Value          = 0;
        app.input_sonar.Value               = 8;
        app.input_surround_cameras.Value    = 4;
        app.input_front_cameras.Value       = 4;
        app.input_c2x.Value                 = 1;
        app.input_5g.Value                  = 1;
        app.input_onboard_pc.Value          = 1;
        app.input_gps_reciever.Value        = 1;
        app.input_gps_correction.Value      = 0;
        app.input_displays.Value            = 2;
        app.input_hvac.Value                = 1;
        app.input_side_airbags.Value        = 4;
        app.input_pneumatic_doors.Value     = 0;
        app.input_emech_steering.Value      = 1;
        app.input_sbw.Value                 = 1;
        app.input_emech_brakes.Value        = 1;
        app.input_obc.Value                 = 1;
        app.input_window_lift.Value         = 1;
        app.input_adjustable_seats.Value    = 1;
        app.input_seat_warmers.Value        = 1;
        app.input_headlight_tech.Value      = 'LED';
        app.input_phone_connectivity.Value  = 1;
        app.input_trunk_assist.Value        = 1;
        app.input_night_vision.Value        = 0;
        app.input_park_assist.Value         = 1;
        app.input_bsm.Value                 = 1;
        app.input_lks.Value                 = 1;
        app.input_acc.Value                 = 1;
        app.input_keyless_go.Value          = 1;
        app.input_spare_tire.Value          = 0;
        app.input_panorama_roof.Value       = 1;
        app.input_sliding_roof.Value        = 0;
        app.input_tire_load.Value           = 1;
    
end

end

