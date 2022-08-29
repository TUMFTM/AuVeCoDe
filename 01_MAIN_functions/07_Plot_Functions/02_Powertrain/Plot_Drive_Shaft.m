function Plot_Drive_Shaft(vehicle)
%% Description:
% Designed by: Korbinian Moller, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.10.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function plots the drive shafts at driven axles
% ------------
% Sources:      [1] Korbinian Moller, “Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,” Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2022
%               [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - Plot of driveshafts
% ------------


%% Implementation
% 1) Assign local variables
% 2) Load and calculate needed references
% 3) Assign drive shaft coordinates
% 4) Plot Drive Shaft

%% 1) Assign local variables

r_w = vehicle.dimensions.CX.wheel_f_diameter/2;         % Radius wheel
h_w = vehicle.dimensions.CY.wheel_f_width;              % width wheel
veh_width = vehicle.dimensions.GY.vehicle_width;                % vehicle width
veh_wb = vehicle.dimensions.GX.wheelbase;               % vehicle wheelbase
sc = 40;                                                % number of sections in cylinder
color = [64 64 64]./255;                                % color
filled_axles = vehicle.LDS.settings.filled_axles;       % Find which axles are filled: [front_axle, rear_axle]

%% 2) Load and calculate needed references

[drivetrain_type,~] = Package_assign_topology(vehicle);         % Evaluate current topology

for axle = find(filled_axles)
    
    % load drive shaft radius
    r_shaft = (vehicle.gearbox{axle}.shafts.d_sh_3 + 20)/2;     % radius of drive shaft
    R2 = r_shaft * 2;                                           % Radius of beginning/end
    R1 = r_shaft * 1.5;                                         % Radius of interfaces   
    R_i = [R2, R2, R1, R1];                                     % Radius array (needed for Object3d_Drive_Shaft)
    R_o = [R1, R1, R2, R2];                                     % Radius array (needed for Object3d_Drive_Shaft)
    
    switch drivetrain_type(axle)
    
    case 0 % no motor
        break
        
    case 1 % motor with parallel gear
        
        %reference left (motor and tire left)
        p_ref_l_i = vehicle.dimensions.p_gear_paral{axle}(7,:); %inner (gear 4)
        size_ref_l_i = vehicle.dimensions.p_gear_paral{axle}(10,:); %inner (gearbox)

        %inner side of left tire
        p_ref_l_o = [veh_wb * (axle - 1), h_w, r_w]; %outer (auto adjusted for RWD)

        %reference right (gearbox and tire right)
        p_ref_r_i = vehicle.dimensions.p_gear_paral{axle}(7,:); %inner (gear 4)
        size_ref_r_i = vehicle.dimensions.p_gear_paral{axle}(10,:); %inner (gearbox)

        %inner side of right tire
        p_ref_r_o = [veh_wb * (axle - 1), veh_width - h_w, r_w]; %outer (auto adjusted for RWD)
        
        % adjust inner points (y-coordinates have to be at gearbox center)
        p_ref_l_i(2) =  vehicle.dimensions.p_gear_paral{axle}(9,2) - vehicle.dimensions.p_motor_paral{axle}(2,2);
        p_ref_r_i(2) =  vehicle.dimensions.p_gear_paral{axle}(9,2);
        
    case 2 % motor with coaxial gear (planetary)
        
        %reference left (motor and tire left)
        p_ref_l_i = vehicle.dimensions.p_motor_coax{axle}(1,:); %inner
        size_ref_l_i = vehicle.dimensions.p_motor_coax{axle}(2,:); %inner

        %inner side of left tire
        p_ref_l_o = [veh_wb * (axle - 1), h_w, r_w]; %outer (auto adjusted for RWD)

        %reference right (gearbox and tire right)
        p_ref_r_i = vehicle.dimensions.p_gear_coax{axle}(1,:); %inner
        size_ref_r_i = vehicle.dimensions.p_gear_coax{axle}(2,:); %inner

        %inner side of right tire
        p_ref_r_o = [veh_wb * (axle - 1), veh_width - h_w, r_w]; %outer (auto adjusted for RWD)
        
    case 3 % two motors in coaxial position (planetary)
        
        %reference left (motor and tire left)
        p_ref_l_i = vehicle.dimensions.p_gear_nax{axle}(1,:); %inner
        size_ref_l_i = vehicle.dimensions.p_gear_nax{axle}(2,:); %inner

        %inner side of left tire
        p_ref_l_o = [veh_wb * (axle - 1), h_w, r_w]; %outer (auto adjusted for RWD)

        %reference right (gearbox and tire right)
        p_ref_r_i = vehicle.dimensions.p_gear_nax{axle}(3,:); %inner
        size_ref_r_i = size_ref_l_i; %inner

        %inner side of right tire
        p_ref_r_o = [veh_wb * (axle - 1), veh_width - h_w, r_w]; %outer (auto adjusted for RWD)

        
    case 4 % motor with coaxial gear (layshaft)
               
        %reference left (motor and tire left)
        p_ref_l_i = vehicle.dimensions.p_motor_coax{axle}(1,:); %inner
        size_ref_l_i = vehicle.dimensions.p_motor_coax{axle}(2,:); %inner

        %inner side of left tire
        p_ref_l_o = [veh_wb * (axle - 1), h_w, r_w]; %outer (auto adjusted for RWD)

        %reference right (gearbox and tire right)
        p_ref_r_i = vehicle.dimensions.p_motor_coax{axle}(1,:) + [0, 0.5* vehicle.dimensions.p_motor_coax{axle}(2,2) + 0.5 * vehicle.dimensions.p_gear_coax_ls{axle}(10,2), 0]; %inner (gearbox outlet)
        size_ref_r_i = vehicle.dimensions.p_gear_coax_ls{axle}(10,:); %inner

        %inner side of right tire
        p_ref_r_o = [veh_wb * (axle - 1), veh_width - h_w, r_w]; %outer (auto adjusted for RWD)
        
    end
    
    
%% 3) Assign drive shaft coordinates
    
    % Adjust inner reference points for rear axle
    if axle == 2
        p_ref_l_i(1) = veh_wb - p_ref_l_i(1);
        p_ref_r_i(1) = veh_wb - p_ref_r_i(1);
    end
    
    % position Drive shaft left (driver side)
    pos_l_i = p_ref_l_i + [0, -0.5 * size_ref_l_i(2), r_w]; %inner
    pos_l_o = p_ref_l_o; %outer

    % position Drive shaft right (co-driver side)
    pos_r_i = p_ref_r_i + [0, 0.5 * size_ref_r_i(2), r_w]; %inner
    pos_r_o = p_ref_r_o ; %outer
    
    % additional points for knuckles
    ds_length = norm(pos_l_i - pos_l_o);
    knuckle_length = 0.1 * ds_length;
    pos_k_l_i = pos_l_i + [0,-knuckle_length,0];
    pos_k_l_o = pos_l_o + [0,knuckle_length,0];
    
    pos_k_r_i = pos_r_i + [0,knuckle_length,0];
    pos_k_r_o = pos_r_o + [0,-knuckle_length,0];

    % Calculate Cylinder from specified points
    % left (driver side)
    [X_l_1, Y_l_1, Z_l_1] = Object3d_Drive_Shaft(R_i, sc, pos_l_i, pos_k_l_i); %inner knuckle
    [X_l_2, Y_l_2, Z_l_2] = Object3d_Drive_Shaft(r_shaft, sc, pos_k_l_i, pos_k_l_o); %drive shaft from inner to outer knuckle
    [X_l_3, Y_l_3, Z_l_3] = Object3d_Drive_Shaft(R_o, sc, pos_k_l_o, pos_l_o); %outer knuckle
    
    X_l = [X_l_1;X_l_2;X_l_3]; Y_l = [Y_l_1;Y_l_2;Y_l_3]; Z_l = [Z_l_1;Z_l_2;Z_l_3]; %combine matrices for easy plotting
    
    % right (co-driver side)
    [X_r_1, Y_r_1, Z_r_1] = Object3d_Drive_Shaft(R_i, sc, pos_r_i, pos_k_r_i); %inner knuckle
    [X_r_2, Y_r_2, Z_r_2] = Object3d_Drive_Shaft(r_shaft, sc, pos_k_r_i, pos_k_r_o); %drive shaft from inner to outer knuckle
    [X_r_3, Y_r_3, Z_r_3] = Object3d_Drive_Shaft(R_o, sc, pos_k_r_o, pos_r_o); %outer knuckle
    
    X_r = [X_r_1;X_r_2;X_r_3]; Y_r = [Y_r_1;Y_r_2;Y_r_3]; Z_r = [Z_r_1;Z_r_2;Z_r_3]; %combine matrices for easy plotting    
    
%% 4) Plot Drive Shaft
    % plot left drive shaft
    surf(X_l, Y_l, Z_l, 'FaceColor', color);
    fill3(X_l(1,:),Y_l(1,:),Z_l(1,:),color);
    fill3(X_l(end,:),Y_l(end,:),Z_l(end,:),color);

    % plot right drive shaft
    surf(X_r, Y_r, Z_r, 'FaceColor', color);
    fill3(X_r(1,:),Y_r(1,:),Z_r(1,:),color);
    fill3(X_r(end,:),Y_r(end,:),Z_r(end,:),color);
    
    % add connection shaft between gearbox and drive shaft (only for parallel gearbox)
    if drivetrain_type(axle) == 1      
        pos_con_i = pos_l_i + [0, vehicle.dimensions.p_motor_paral{axle}(2,2), 0];
        pos_con_o = pos_l_i - [0, 10, 0];
        [X_con, Y_con, Z_con] = Object3d_Drive_Shaft(0.8*r_shaft, sc, pos_con_i, pos_con_o);
        surf(X_con, Y_con, Z_con, 'FaceColor', color);
        fill3(X_con(1,:),Y_con(1,:),Z_con(1,:),color);
        fill3(X_con(end,:),Y_con(end,:),Z_con(end,:),color);       
    end

end

end