function PlotBattery_housing(axis,v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function plots the housing of the battery consisting of either underfloor, second level or both
% ------------
% Sources:      [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
%           - Parameters: struct with input and constant values
%           - axis: figure where vehicle is plotted
% ------------
% Output:   - Plot battary housing
% ------------

%% 1) Read measurments of housing
%Sizing of Battery Underfloor
Lx_b=v.battery.installationspace.CX_batt_underfloor; %Length of battery underfloor (see DESIGN_battery)
Ly_b=v.battery.installationspace.CY_batt_underfloor; %Width of battery underfloor (see DESIGN_battery)
Lz_b=v.battery.installationspace.CZ_batt_underfloor+v.dimensions.CZ.batt_top_cover+v.dimensions.EZ.free_space_module_to_top_cover+v.dimensions.CZ.batt_bottom_cover; %Height of battery underfloor (see DESIGN_battery)

% Positioning of underfloor battery (x-start = EX - length/2, y-start = vehicle_width/2 - width/2, z-start = underbody height)
p_b=[v.battery.installationspace.EX_batt_underfloor; 0.5*(v.dimensions.GY.vehicle_width-Ly_b); v.dimensions.GZ.H156];
eu_b=[0;0;0];

%Sizing of Battery Second Level 1
Lx_b_s1=v.battery.installationspace.CX_batt_second_level_1; %Length of battery Second Level 1 (see DESIGN_battery)
Ly_b_s1=v.battery.installationspace.CY_batt_second_level_1; %Width of battery Second Level 1 (see DESIGN_battery)
Lz_b_s1=v.battery.installationspace.CZ_batt_second_level_1+v.dimensions.CZ.batt_top_cover+Par.dimensions.EZ.free_space_module_to_top_cover; %Height of battery Second Level 1 (see DESIGN_battery)
height_s1=v.Input.int_height; %height if underfloor is available (no bottom cover needed)
if v.battery.installationspace.CZ_batt_underfloor==0 %No underfloor below second level => bottom cover
    Lz_b_s1=Lz_b_s1+v.dimensions.CZ.batt_bottom_cover; %Height of battery Second Level 1 (see DESIGN_battery)
end

%Sizing of Battery Second Level 2
Lx_b_s2=v.battery.installationspace.CX_batt_second_level_2; %Length of battery Second Level 2 (see DESIGN_battery)
Ly_b_s2=v.battery.installationspace.CY_batt_second_level_2; %Width of battery Second Level 2 (see DESIGN_battery)
Lz_b_s2=v.battery.installationspace.CZ_batt_second_level_2+v.dimensions.CZ.batt_top_cover+Par.dimensions.EZ.free_space_module_to_top_cover; %Height of battery Second Level 2 (see DESIGN_battery)
if strcmp(v.Input.int_type,'btb')
    height_s2=v.Input.int_height+v.Input.int_z_leg(1)-Par.package.p_int_tolerance; %for back2back: second level2 is above second level 1
    Lz_b_s2=Lz_b_s2+Par.package.p_int_tolerance; %Add tolerance to connect second and 1 Level battery
else
    height_s2=v.Input.int_height; %height if underfloor is available (no bottom cover needed)
end
if v.battery.installationspace.CZ_batt_underfloor==0 %No underfloor below second level => bottom cover
    Lz_b_s2=Lz_b_s2+v.dimensions.CZ.batt_bottom_cover; %Height of battery Second Level 1 (see DESIGN_battery)
end

%% 2) Calculate positions
% Positioning of Second Level 1
% adjustment of position due to plot functions (see sources)
if strcmp(v.Input.int_type,'vav')
    p_b_s1=[v.battery.installationspace.EX_batt_second_level_1; 0.5*(v.dimensions.GY.vehicle_width-Ly_b_s1); height_s1]; % x-start = legs - length, y-start = vehicle_width/2 - width/2, z-start = interior height
elseif strcmp(v.Input.int_type,'btb')
    p_b_s1=[v.battery.installationspace.EX_batt_second_level_1; 0.5*(v.dimensions.GY.vehicle_width-Ly_b_s1); height_s1]; % x-start = wheelbase/2 - length/2, y-start = vehicle_width/2 - width/2, z-start = interior height
else % single row or conventional
    p_b_s1=[v.battery.installationspace.EX_batt_second_level_1; 0.5*(v.dimensions.GY.vehicle_width-Ly_b_s1); height_s1]; % x-start = front legs, y-start = vehicle_width/2 - width/2, z-start = interior height
end

% Positioning of Second Level 2
% adjustment of position due to plot functions (see sources)
p_b_s2=[v.battery.installationspace.EX_batt_second_level_2; 0.5*(v.dimensions.GY.vehicle_width-Ly_b_s2); height_s2]; % x-start = rear legs, y-start = vehicle_width/2 - width/2, z-start = interior height

%% 3) Plot batteries
%Plot battery underfloor
[F_b,V_b]=Object3d_Block(p_b,eu_b,Lx_b,Ly_b,Lz_b);
patch(axis,'FaceAlpha',0.7,'Faces', F_b, 'Vertices', V_b, 'FaceColor', [0 101 189]./255); %use patch to plot battery underfloor
%Plot battery second level 1
if table2array(v.battery.filltable('second level','NumcellZ'))>0 % Plot just if cells are inside module
    [F_b_s1,V_b_s1]=Object3d_Block(p_b_s1,eu_b,Lx_b_s1,Ly_b_s1,Lz_b_s1);
    patch(axis,'FaceAlpha',0.7,'Faces', F_b_s1, 'Vertices', V_b_s1, 'FaceColor', [0 82 147]./255); %use patch to plot battery second level 1
end

%Plot battery second level 2
if table2array(v.battery.filltable('tunnel','NumcellZ'))>0 % Plot just if cells are inside module
    [F_b_s2,V_b_s2]=Object3d_Block(p_b_s2,eu_b,Lx_b_s2,Ly_b_s2,Lz_b_s2);
    patch(axis,'FaceAlpha',0.7,'Faces', F_b_s2, 'Vertices', V_b_s2, 'FaceColor', [0 82 147]./255); %use patch to plot battery second level 2
end
        
end

